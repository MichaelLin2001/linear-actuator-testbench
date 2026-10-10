#pragma once
#include "joint_controller/types.hpp"
namespace jc {
struct TrajectorySample {
    double position{};
    double velocity{};
    MotionPhase phase{MotionPhase::Idle};
    bool complete{};
};
class TrajectoryGenerator {
public:
    bool start(std::uint64_t now_ns, double q0, double target, double max_velocity,
               bool return_to_start, std::uint64_t command_sequence) noexcept;
    TrajectorySample sample(std::uint64_t now_ns) noexcept;
    void stop(double hold_position, std::uint64_t command_sequence) noexcept;
    bool active() const noexcept { return active_; }
    std::uint64_t command_sequence() const noexcept { return command_sequence_; }
    double start_position() const noexcept { return q0_; }
    double target_position() const noexcept { return target_; }
private:
    std::uint64_t start_ns_{};
    std::uint64_t ramp_ns_{};
    std::uint64_t command_sequence_{};
    double q0_{};
    double target_{};
    double delta_{};
    bool return_to_start_{};
    bool active_{};
    bool stopped_{};
};
}  // namespace jc
