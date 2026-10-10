#pragma once

#include "joint_controller/types.hpp"

namespace jc {

class ControllerResetHook {
public:
    virtual ~ControllerResetHook() = default;
    virtual void reset() noexcept = 0;
};

// There is no control algorithm or integral state in this phase.
class DryRunControllerState final : public ControllerResetHook {
public:
    void reset() noexcept override { ++reset_count_; }
    std::uint64_t reset_count() const noexcept { return reset_count_; }
private:
    std::uint64_t reset_count_{};
};

class StateMachine {
public:
    StateMachine(StateConfig config, ControllerResetHook& reset_hook) noexcept;
    void initialize() noexcept;
    CommandResult step(const FeedbackSnapshot& feedback, const ReadStatistics& reads,
                       bool snapshot_timeout, const Command& command) noexcept;
    void latch_fault(Fault fault) noexcept;
    void stop() noexcept;

    State state() const noexcept { return state_; }
    Fault fault() const noexcept { return fault_; }
    bool run_entry_cycle() const noexcept { return run_entry_cycle_; }
    const std::array<double, kJointCount>& hold_reference() const noexcept { return hold_; }
    const std::array<double, kJointCount>& pending_effort() const noexcept { return pending_; }

private:
    void align_and_reset(const FeedbackSnapshot& feedback) noexcept;
    StateConfig config_;
    ControllerResetHook& reset_hook_;
    State state_{State::Init};
    Fault fault_{Fault::None};
    bool run_entry_cycle_{};
    std::array<double, kJointCount> hold_{};
    std::array<double, kJointCount> pending_{};
};

}  // namespace jc
