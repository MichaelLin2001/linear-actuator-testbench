#pragma once
#include "joint_controller/experiment.hpp"
#include "joint_controller/force_processor.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

namespace jc {

constexpr std::size_t kJointCount = 30;
constexpr std::size_t kJ3 = 3;
constexpr std::uint64_t kPeriodNs = 1'000'000;
constexpr std::size_t kLogCapacity = 8192;
constexpr std::uint32_t kJ3Mask = std::uint32_t{1} << kJ3;
static_assert(std::atomic<bool>::is_always_lock_free);
static_assert(std::atomic<int>::is_always_lock_free);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

struct RawFeedback {
    std::uint8_t run_mode{};
    std::uint8_t motor_type{};
    std::uint8_t reserved[2]{};
    float speed{};
    float position{};
    float effort{};
};
using RawFeedbackArray = std::array<RawFeedback, kJointCount>;
static_assert(std::is_standard_layout<RawFeedback>::value);
static_assert(std::is_trivially_copyable<RawFeedback>::value);
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static_assert(sizeof(RawFeedback) == 16);
static_assert(sizeof(RawFeedbackArray) == 480);
static_assert(offsetof(RawFeedback, run_mode) == 0);
static_assert(offsetof(RawFeedback, speed) == 4);
static_assert(offsetof(RawFeedback, position) == 8);
static_assert(offsetof(RawFeedback, effort) == 12);
#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "The verified jc01 ABI requires a little-endian host."
#endif

constexpr bool is_linear(std::size_t joint) noexcept { return joint == 3 || joint == 9; }
constexpr bool is_controllable(std::size_t joint) noexcept { return joint != 14 && joint != 22; }
constexpr std::uint32_t kControllableMask =
    ((std::uint32_t{1} << kJointCount) - 1U) &
    ~(std::uint32_t{1} << 14) & ~(std::uint32_t{1} << 22);

enum class SourceFreshness : std::uint8_t { Unknown };
enum class SourceTimeout : std::uint8_t { Unsupported };
enum class ReadStatus : std::uint8_t {
    Ok, Contended, Interrupted, SemaphoreError, ReleaseError, NotAttached, ClockError
};
enum class State : std::uint8_t { Init, Idle, Armed, Run, Stop, Fault };
enum class Fault : std::uint8_t {
    None, ConsecutiveReadFailures, SnapshotTimeout, NonFiniteFeedback,
    SemaphoreRelease, Clock, Sleep, Logger, TrackingError, TargetWrite, ForceSensor, ExperimentSafety, EmergencyStop, Heartbeat
};
enum class CommandType : std::uint8_t { None, Arm, Run, Stop, Reset, Move, ForceZero, Configure, Preload, Unload };
enum class CommandResult : std::uint8_t {
    None, Accepted, RejectedState, RejectedFeedback, RejectedVelocity, RejectedArgument
};
enum class ControlMode : std::uint8_t { ReadOnlyDryRun, CspBuiltin };
enum class MotionPhase : std::uint8_t {
    Idle, InitialHold, Outbound, TargetHold, Return, FinalHold, Complete, Stopped
};

struct JointFeedback { double position{}; double speed{}; double effort{}; };

struct FeedbackSnapshot {
    RawFeedbackArray raw{};
    std::array<JointFeedback, kJointCount> joints{};
    std::uint64_t snapshot_read_timestamp_ns{};
    std::uint64_t snapshot_read_duration_ns{};
    std::uint32_t finite_mask{};
    int error_number{};
    ReadStatus read_status{ReadStatus::NotAttached};
    SourceFreshness source_feedback_freshness{SourceFreshness::Unknown};
    SourceTimeout source_feedback_timeout{SourceTimeout::Unsupported};
    bool snapshot_read_ok{};
    bool feedback_valid{};
};

inline void convert_feedback(FeedbackSnapshot& snapshot) noexcept {
    snapshot.finite_mask = 0;
    snapshot.feedback_valid = false;
    if (!snapshot.snapshot_read_ok) { return; }
    for (std::size_t i = 0; i < kJointCount; ++i) {
        const double scale = is_linear(i) ? 0.001 : 1.0;
        snapshot.joints[i] = {static_cast<double>(snapshot.raw[i].position) * scale,
                              static_cast<double>(snapshot.raw[i].speed) * scale,
                              static_cast<double>(snapshot.raw[i].effort)};
        const auto& value = snapshot.joints[i];
        if (std::isfinite(value.position) && std::isfinite(value.speed) &&
            std::isfinite(value.effort)) {
            snapshot.finite_mask |= std::uint32_t{1} << i;
        }
    }
    snapshot.feedback_valid = (snapshot.finite_mask & kControllableMask) == kControllableMask;
}

struct ReadStatistics {
    std::uint64_t total_reads{};
    std::uint64_t successful_reads{};
    std::uint64_t failed_reads{};
    std::uint64_t consecutive_failed_reads{};
    std::uint64_t semaphore_contention_count{};
    std::uint64_t maximum_read_duration_ns{};
    std::uint64_t last_success_timestamp_ns{};
};

struct Command {
    CommandType type{CommandType::None};
    std::uint64_t sequence{};
    std::uint64_t received_ns{};
    std::size_t joint{kJ3};
    double position{};
    double max_velocity{};
    bool relative{};
    bool return_to_start{};
    ControlMode mode{ControlMode::CspBuiltin};
    ExperimentConfig experiment{};
};
static_assert(std::is_trivially_copyable<Command>::value);

struct StateConfig {
    std::uint64_t max_consecutive_failed_reads{20};
    std::uint64_t snapshot_timeout_ns{20'000'000};
    double max_arm_angular_speed{0.05};
    double max_arm_linear_speed{0.005};
    std::uint32_t active_mask{kControllableMask};
};

struct LoopConfig {
    StateConfig state{};
    std::uint64_t duration_ns{30'000'000'000ULL};
    int cpu{-1};
    int fifo_priority{};
    bool lock_memory{};
    bool output_enabled{};
    double maximum_move_m{0.010};
    double maximum_velocity_mps{0.015};
    double maximum_tracking_error_m{0.00025};
};

struct CycleRecord {
    std::uint64_t cycle_index{};
    std::uint64_t scheduled_wakeup_ns{};
    std::uint64_t actual_wakeup_ns{};
    std::int64_t wakeup_jitter_ns{};
    std::uint64_t actual_dt_ns{};
    std::uint64_t cycle_execution_ns{};
    std::uint64_t snapshot_age_ns{};
    std::uint64_t skipped_before_cycle{};
    std::uint64_t skipped_after_cycle{};
    std::uint64_t dropped_record_count{};
    std::uint64_t controller_reset_count{};
    std::uint64_t last_command_sequence{};
    std::uint64_t last_command_received_ns{};
    State state{State::Init};
    Fault fault{Fault::None};
    CommandType command{CommandType::None};
    CommandResult command_result{CommandResult::None};
    ControlMode control_mode{ControlMode::ReadOnlyDryRun};
    MotionPhase motion_phase{MotionPhase::Idle};
    bool actual_dt_valid{};
    bool deadline_miss{};
    bool cycle_valid{};
    bool snapshot_read_timeout{};
    bool run_entry_cycle{};
    bool control_compute_executed{};
    bool target_write_ok{};
    bool trajectory_complete{};
    bool actual_at_target{};
    FeedbackSnapshot feedback{};
    ReadStatistics reads{};
    std::array<double, kJointCount> hold_reference{};
    std::array<double, kJointCount> pending_effort{};
    double reference_position{};
    double reference_velocity{};
    double submitted_csp_position{};
    double tracking_error{};
    std::uint64_t target_write_failures{};
    ProcessedForce force{};
    ExperimentStage experiment_stage{ExperimentStage::Idle};
    std::uint64_t experiment_id{};
    unsigned experiment_level{};
    int experiment_direction{};
    bool actual_current_valid{};
    double commanded_current_a{}, actual_current_a{};

};
static_assert(std::is_trivially_copyable<CycleRecord>::value);

struct LoopSummary {
    std::uint64_t cycles{};
    std::uint64_t valid_cycles{};
    std::uint64_t deadline_misses{};
    std::uint64_t skipped_periods{};
    std::uint64_t max_wakeup_jitter_ns{};
    std::uint64_t max_cycle_execution_ns{};
    std::uint64_t dt_samples{};
    std::uint64_t min_actual_dt_ns{std::numeric_limits<std::uint64_t>::max()};
    std::uint64_t max_actual_dt_ns{};
    long double sum_actual_dt_ns{};
    long double sum_wakeup_jitter_ns{};
    long double sum_cycle_execution_ns{};
    ReadStatistics reads{};
    State final_state{State::Init};
    Fault final_fault{Fault::None};
};

struct ScheduleAdvance { std::uint64_t next_ns{}; std::uint64_t skipped{}; };
constexpr ScheduleAdvance advance_schedule(std::uint64_t scheduled,
                                           std::uint64_t completed) noexcept {
    const auto next = scheduled + kPeriodNs;
    if (completed < next) { return {next, 0}; }
    const auto skipped = (completed - next) / kPeriodNs + 1;
    return {next + skipped * kPeriodNs, skipped};
}

inline std::uint64_t double_bits(double value) noexcept {
    std::uint64_t bits{}; std::memcpy(&bits, &value, sizeof(bits)); return bits;
}
inline double bits_double(std::uint64_t bits) noexcept {
    double value{}; std::memcpy(&value, &bits, sizeof(value)); return value;
}

}  // namespace jc
