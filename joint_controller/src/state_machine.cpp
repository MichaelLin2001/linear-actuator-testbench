#include "joint_controller/state_machine.hpp"

#include <cmath>

namespace jc {

StateMachine::StateMachine(StateConfig config, ControllerResetHook& reset_hook) noexcept
    : config_(config), reset_hook_(reset_hook) {}

void StateMachine::initialize() noexcept {
    if (state_ == State::Init) {
        state_ = State::Idle;
    }
}

void StateMachine::align_and_reset(const FeedbackSnapshot& feedback) noexcept {
    for (std::size_t i = 0; i < kJointCount; ++i) {
        hold_[i] = (config_.active_mask & (std::uint32_t{1} << i)) != 0
                       ? feedback.joints[i].position : 0.0;
    }
    pending_.fill(0.0);
    reset_hook_.reset();
}

void StateMachine::latch_fault(Fault fault) noexcept {
    if (state_ != State::Fault) {
        fault_ = fault;
    }
    state_ = State::Fault;
    run_entry_cycle_ = false;
    pending_.fill(0.0);
}

void StateMachine::stop() noexcept {
    if (state_ != State::Fault) {
        state_ = State::Stop;
    }
    run_entry_cycle_ = false;
    pending_.fill(0.0);
}

CommandResult StateMachine::step(const FeedbackSnapshot& feedback, const ReadStatistics& reads,
                                 bool snapshot_timeout, const Command& command) noexcept {
    run_entry_cycle_ = false;
    pending_.fill(0.0);
    bool health_fault = false;
    if (feedback.read_status == ReadStatus::ReleaseError) {
        latch_fault(Fault::SemaphoreRelease);
        health_fault = true;
    } else if (feedback.read_status == ReadStatus::ClockError) {
        latch_fault(Fault::Clock);
        health_fault = true;
    } else if (feedback.snapshot_read_ok && !feedback.feedback_valid) {
        latch_fault(Fault::NonFiniteFeedback);
        health_fault = true;
    } else if (reads.consecutive_failed_reads > config_.max_consecutive_failed_reads) {
        latch_fault(Fault::ConsecutiveReadFailures);
        health_fault = true;
    } else if (snapshot_timeout) {
        latch_fault(Fault::SnapshotTimeout);
        health_fault = true;
    }

    switch (command.type) {
    case CommandType::None:
        return CommandResult::None;
    case CommandType::Stop:
        stop();
        return CommandResult::Accepted;
    case CommandType::Reset:
        if (state_ != State::Stop && state_ != State::Fault) {
            return CommandResult::RejectedState;
        }
        if (health_fault || !feedback.feedback_valid) {
            return CommandResult::RejectedFeedback;
        }
        if (fault_ == Fault::Clock || fault_ == Fault::Sleep || fault_ == Fault::Logger ||
            fault_ == Fault::SemaphoreRelease) {
            return CommandResult::RejectedState;
        }
        align_and_reset(feedback);
        fault_ = Fault::None;
        state_ = State::Idle;
        return CommandResult::Accepted;
    case CommandType::Arm:
        if (state_ != State::Idle) {
            return CommandResult::RejectedState;
        }
        if (!feedback.feedback_valid) {
            return CommandResult::RejectedFeedback;
        }
        align_and_reset(feedback);
        state_ = State::Armed;
        return CommandResult::Accepted;
    case CommandType::Run:
        if (state_ != State::Armed) {
            return CommandResult::RejectedState;
        }
        if (!feedback.feedback_valid) {
            return CommandResult::RejectedFeedback;
        }
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const double threshold = is_linear(i) ? config_.max_arm_linear_speed
                                                  : config_.max_arm_angular_speed;
            if ((config_.active_mask & (std::uint32_t{1} << i)) != 0 &&
                std::abs(feedback.joints[i].speed) > threshold) {
                return CommandResult::RejectedVelocity;
            }
        }
        align_and_reset(feedback);
        state_ = State::Run;
        run_entry_cycle_ = true;
        return CommandResult::Accepted;
    case CommandType::Move:
        return state_ == State::Run ? CommandResult::Accepted
                                    : CommandResult::RejectedState;
    case CommandType::ForceZero:
    case CommandType::Configure:
    case CommandType::Preload:
    case CommandType::BeginLoad:
    case CommandType::CommitLoad:
    case CommandType::ResumeCsp:
    case CommandType::Unload:
        return state_ != State::Fault ? CommandResult::Accepted : CommandResult::RejectedState;
    }
    return CommandResult::RejectedState;
}

}  // namespace jc
