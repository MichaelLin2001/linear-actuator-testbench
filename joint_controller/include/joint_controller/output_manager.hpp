#pragma once
#include "joint_controller/jc_target_adapter.hpp"
namespace jc {
class OutputManager {
public:
    explicit OutputManager(JcTargetAdapter& adapter) noexcept : adapter_(adapter) {}
    TargetWriteStatus write_csp(double position) noexcept;
    double last_submitted() const noexcept { return last_submitted_; }
    bool has_submitted() const noexcept { return has_submitted_; }
    std::uint64_t failures() const noexcept { return failures_; }
    std::uint64_t consecutive_failures() const noexcept { return consecutive_failures_; }
private:
    JcTargetAdapter& adapter_;
    double last_submitted_{};
    std::uint64_t failures_{};
    std::uint64_t consecutive_failures_{};
    bool has_submitted_{};
};
}  // namespace jc
