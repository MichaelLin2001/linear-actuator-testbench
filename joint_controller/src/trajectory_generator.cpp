#include "joint_controller/trajectory_generator.hpp"
#include <algorithm>
#include <cmath>
namespace jc {
namespace {
constexpr std::uint64_t kInitialHoldNs = 200'000'000;
constexpr std::uint64_t kTargetHoldNs = 500'000'000;
constexpr std::uint64_t kFinalHoldNs = 200'000'000;
double smoothstep(double x) noexcept { x = std::clamp(x, 0.0, 1.0); return x*x*(3.0-2.0*x); }
double smoothstep_d(double x) noexcept { x = std::clamp(x, 0.0, 1.0); return 6.0*x*(1.0-x); }
}
bool TrajectoryGenerator::start(std::uint64_t now_ns, double q0, double target,
                                double max_velocity, bool return_to_start,
                                std::uint64_t command_sequence) noexcept {
    if (now_ns == 0 || !std::isfinite(q0) || !std::isfinite(target) ||
        !std::isfinite(max_velocity) || max_velocity <= 0.0 || target == q0) { return false; }
    start_ns_ = now_ns; q0_ = q0; target_ = target; delta_ = target - q0;
    return_to_start_ = return_to_start; command_sequence_ = command_sequence;
    const double seconds = 1.5 * std::abs(delta_) / max_velocity;
    ramp_ns_ = std::max<std::uint64_t>(kPeriodNs,
        static_cast<std::uint64_t>(std::ceil(seconds * 1.0e9)));
    active_ = true; stopped_ = false; return true;
}
TrajectorySample TrajectoryGenerator::sample(std::uint64_t now_ns) noexcept {
    if (stopped_) { return {q0_, 0.0, MotionPhase::Stopped, true}; }
    const double final_position = return_to_start_ ? q0_ : target_;
    if (!active_) { return {final_position, 0.0, MotionPhase::Complete, true}; }
    std::uint64_t t = now_ns > start_ns_ ? now_ns - start_ns_ : 0;
    if (t < kInitialHoldNs) { return {q0_, 0.0, MotionPhase::InitialHold, false}; }
    t -= kInitialHoldNs;
    if (t < ramp_ns_) {
        const double x = static_cast<double>(t) / static_cast<double>(ramp_ns_);
        return {q0_ + delta_ * smoothstep(x),
                delta_ * smoothstep_d(x) * 1.0e9 / static_cast<double>(ramp_ns_),
                MotionPhase::Outbound, false};
    }
    t -= ramp_ns_;
    if (t < kTargetHoldNs) { return {target_, 0.0, MotionPhase::TargetHold, false}; }
    t -= kTargetHoldNs;
    if (return_to_start_) {
        if (t < ramp_ns_) {
            const double x = static_cast<double>(t) / static_cast<double>(ramp_ns_);
            return {target_ - delta_ * smoothstep(x),
                    -delta_ * smoothstep_d(x) * 1.0e9 / static_cast<double>(ramp_ns_),
                    MotionPhase::Return, false};
        }
        t -= ramp_ns_;
    }
    if (t < kFinalHoldNs) { return {final_position, 0.0, MotionPhase::FinalHold, false}; }
    active_ = false;
    return {final_position, 0.0, MotionPhase::Complete, true};
}
void TrajectoryGenerator::stop(double hold_position, std::uint64_t command_sequence) noexcept {
    q0_ = hold_position; target_ = hold_position; delta_ = 0.0;
    command_sequence_ = command_sequence; active_ = false; stopped_ = true;
}
}  // namespace jc
