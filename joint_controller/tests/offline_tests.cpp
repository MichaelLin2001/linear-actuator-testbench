#include "joint_controller/command_manager.hpp"
#include "joint_controller/state_machine.hpp"
#include "joint_controller/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::exit(1);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

jc::FeedbackSnapshot valid_feedback() {
    jc::FeedbackSnapshot snapshot{};
    snapshot.snapshot_read_ok = true;
    snapshot.read_status = jc::ReadStatus::Ok;
    snapshot.raw[0].position = 1.25F;
    snapshot.raw[3].position = 125.0F;
    snapshot.raw[9].speed = 2.0F;
    snapshot.raw[3].effort = 42.0F;
    jc::convert_feedback(snapshot);
    return snapshot;
}

void test_units_and_validity() {
    auto snapshot = valid_feedback();
    CHECK(snapshot.feedback_valid);
    CHECK(snapshot.joints[0].position == 1.25);
    CHECK(std::abs(snapshot.joints[3].position - 0.125) < 1e-12);
    CHECK(std::abs(snapshot.joints[9].speed - 0.002) < 1e-12);
    CHECK(snapshot.joints[3].effort == 42.0);
    CHECK(snapshot.raw[3].position == 125.0F);
    CHECK(snapshot.source_feedback_freshness == jc::SourceFreshness::Unknown);
    CHECK(snapshot.source_feedback_timeout == jc::SourceTimeout::Unsupported);
    snapshot.raw[14].position = std::numeric_limits<float>::quiet_NaN();
    jc::convert_feedback(snapshot);
    CHECK(snapshot.feedback_valid);
    CHECK((snapshot.finite_mask & (1U << 14)) == 0);
    snapshot.raw[1].effort = std::numeric_limits<float>::infinity();
    jc::convert_feedback(snapshot);
    CHECK(!snapshot.feedback_valid);
    snapshot.snapshot_read_ok = false;
    jc::convert_feedback(snapshot);
    CHECK(!snapshot.feedback_valid && snapshot.finite_mask == 0);
}

void test_state_machine() {
    jc::DryRunControllerState controller;
    jc::StateConfig config{};
    jc::StateMachine state(config, controller);
    jc::ReadStatistics reads{};
    state.initialize();
    CHECK(state.state() == jc::State::Idle);
    jc::FeedbackSnapshot invalid{};
    CHECK(state.step(invalid, reads, false, {jc::CommandType::Arm, 1, 1}) ==
          jc::CommandResult::RejectedFeedback);
    auto snapshot = valid_feedback();
    CHECK(state.step(snapshot, reads, false, {jc::CommandType::Run, 2, 2}) ==
          jc::CommandResult::RejectedState);
    CHECK(state.step(snapshot, reads, false, {jc::CommandType::Arm, 3, 3}) ==
          jc::CommandResult::Accepted);
    CHECK(state.state() == jc::State::Armed && controller.reset_count() == 1);
    CHECK(state.hold_reference()[0] == snapshot.joints[0].position);
    snapshot.joints[0].speed = 0.1;
    CHECK(state.step(snapshot, reads, false, {jc::CommandType::Run, 4, 4}) ==
          jc::CommandResult::RejectedVelocity);
    snapshot.joints[0].speed = 0.0;
    snapshot.joints[0].position = 2.5;
    CHECK(state.step(snapshot, reads, false, {jc::CommandType::Run, 5, 5}) ==
          jc::CommandResult::Accepted);
    CHECK(state.state() == jc::State::Run && state.run_entry_cycle());
    CHECK(state.hold_reference()[0] == 2.5 && controller.reset_count() == 2);
    for (const auto effort : state.pending_effort()) { CHECK(effort == 0.0); }
    invalid.read_status = jc::ReadStatus::Contended;
    for (std::uint64_t failures = 1; failures <= config.max_consecutive_failed_reads; ++failures) {
        reads.consecutive_failed_reads = failures;
        state.step(invalid, reads, false, {});
        CHECK(state.state() == jc::State::Run);
        CHECK(!state.run_entry_cycle());
        for (const auto effort : state.pending_effort()) { CHECK(effort == 0.0); }
    }
    ++reads.consecutive_failed_reads;
    state.step(invalid, reads, false, {});
    CHECK(state.state() == jc::State::Fault && state.fault() == jc::Fault::ConsecutiveReadFailures);
    reads.consecutive_failed_reads = 0;
    state.step(snapshot, reads, false, {});
    CHECK(state.state() == jc::State::Fault);
    CHECK(state.step(snapshot, reads, false, {jc::CommandType::Reset, 6, 6}) ==
          jc::CommandResult::Accepted);
    CHECK(state.state() == jc::State::Idle);
    state.step(snapshot, reads, false, {jc::CommandType::Stop, 7, 7});
    CHECK(state.state() == jc::State::Stop);
    state.step(snapshot, reads, false, {jc::CommandType::Reset, 8, 8});
    state.step(invalid, reads, true, {});
    CHECK(state.fault() == jc::Fault::SnapshotTimeout);
}

void test_terminal_faults() {
    jc::DryRunControllerState controller;
    jc::StateMachine state({}, controller);
    state.initialize();
    auto snapshot = valid_feedback();
    state.latch_fault(jc::Fault::Logger);
    CHECK(state.step(snapshot, {}, false, {jc::CommandType::Reset, 1, 1}) ==
          jc::CommandResult::RejectedState);
    state.stop();
    CHECK(state.state() == jc::State::Fault);
    jc::StateMachine numeric({}, controller);
    numeric.initialize();
    snapshot.feedback_valid = false;
    numeric.step(snapshot, {}, false, {});
    CHECK(numeric.fault() == jc::Fault::NonFiniteFeedback);
}

void test_ring_and_commands() {
    jc::SpscRing<jc::Command, 2> ring;
    jc::Command value{};
    CHECK(!ring.try_pop(value));
    auto* slot = ring.reserve();
    CHECK(slot != nullptr);
    *slot = {jc::CommandType::Arm, 10, 1};
    CHECK(!ring.try_pop(value));
    ring.publish();
    CHECK(ring.try_push({jc::CommandType::Run, 11, 2}));
    CHECK(!ring.try_push({jc::CommandType::Stop, 12, 3}));
    CHECK(ring.try_pop(value) && value.sequence == 10);
    CHECK(ring.try_push({jc::CommandType::Stop, 12, 3}));
    CHECK(ring.try_pop(value) && value.sequence == 11);
    CHECK(ring.try_pop(value) && value.sequence == 12);
    CHECK(!ring.try_pop(value));
    jc::CommandManager commands;
    for (std::uint64_t i = 1; i <= 32; ++i) {
        CHECK(commands.submit(jc::CommandType::Arm, i));
    }
    CHECK(!commands.submit(jc::CommandType::Run, 33));
    for (std::uint64_t i = 1; i <= 32; ++i) {
        CHECK(commands.poll(value));
        CHECK(value.sequence == i && value.received_ns == i);
    }
}

void test_schedule() {
    constexpr std::uint64_t scheduled = 10 * jc::kPeriodNs;
    auto next = jc::advance_schedule(scheduled, scheduled + 50'000);
    CHECK(next.next_ns == scheduled + jc::kPeriodNs && next.skipped == 0);
    next = jc::advance_schedule(scheduled, scheduled + jc::kPeriodNs);
    CHECK(next.next_ns == scheduled + 2 * jc::kPeriodNs && next.skipped == 1);
    next = jc::advance_schedule(scheduled, scheduled + 3 * jc::kPeriodNs + 17);
    CHECK(next.next_ns == scheduled + 4 * jc::kPeriodNs && next.skipped == 3);
    for (std::uint64_t duration = 0; duration < 10 * jc::kPeriodNs; duration += 379) {
        next = jc::advance_schedule(scheduled, scheduled + duration);
        CHECK(next.next_ns > scheduled + duration);
        CHECK((next.next_ns - scheduled) % jc::kPeriodNs == 0);
    }
}

void test_trajectory_peak_velocity_and_fixed_origin() {
    jc::TrajectoryGenerator trajectory;
    constexpr std::uint64_t start = 1'000'000'000ULL;
    const jc::LoopConfig config{};
    CHECK(config.maximum_velocity_mps == 0.015);
    CHECK(trajectory.start(start, 0.015, 0.025, config.maximum_velocity_mps, true, 42));
    double maximum_speed = 0.0;
    double maximum_position = 0.015;
    jc::TrajectorySample sample{};
    for (std::uint64_t t = start; t < start + 33'000'000'000ULL; t += jc::kPeriodNs) {
        sample = trajectory.sample(t);
        maximum_speed = std::max(maximum_speed, std::abs(sample.velocity));
        maximum_position = std::max(maximum_position, sample.position);
        if (sample.complete) { break; }
    }
    CHECK(maximum_speed <= config.maximum_velocity_mps + 1e-9);
    CHECK(maximum_speed >= config.maximum_velocity_mps - 1e-6);
    CHECK(std::abs(maximum_position - 0.025) < 1e-9);
    CHECK(sample.complete && std::abs(sample.position - 0.015) < 1e-12);
    CHECK(trajectory.command_sequence() == 42);
}

void test_nonreturn_trajectory_holds_target_and_allows_next_move() {
    jc::TrajectoryGenerator trajectory;
    constexpr std::uint64_t start = 1'000'000'000ULL;
    CHECK(trajectory.start(start, 0.016, 0.026, 0.001, false, 50));

    const auto completed = trajectory.sample(start + 16'000'000'000ULL);
    CHECK(completed.complete);
    CHECK(completed.phase == jc::MotionPhase::Complete);
    CHECK(std::abs(completed.position - 0.026) < 1e-12);

    const auto held = trajectory.sample(start + 17'000'000'000ULL);
    CHECK(held.complete);
    CHECK(held.phase == jc::MotionPhase::Complete);
    CHECK(std::abs(held.position - 0.026) < 1e-12);
    CHECK(held.velocity == 0.0);

    constexpr std::uint64_t second_start = start + 18'000'000'000ULL;
    CHECK(trajectory.start(second_start, held.position, 0.031, 0.001, false, 51));
    const auto second_completed = trajectory.sample(second_start + 9'000'000'000ULL);
    CHECK(second_completed.complete);
    CHECK(std::abs(second_completed.position - 0.031) < 1e-12);
    CHECK(trajectory.command_sequence() == 51);
}
}  // namespace

int main() {
    test_units_and_validity();
    test_state_machine();
    test_terminal_faults();
    test_ring_and_commands();
    test_schedule();
    test_trajectory_peak_velocity_and_fixed_origin();
    test_nonreturn_trajectory_holds_target_and_allows_next_move();
    std::puts("Offline contracts passed. No IPC or hardware access was performed.");
}
