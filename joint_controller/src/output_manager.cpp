#include "joint_controller/output_manager.hpp"
namespace jc {
TargetWriteStatus OutputManager::write_csp(double position) noexcept {
    const auto status = adapter_.write_csp(kJ3, position);
    if (status == TargetWriteStatus::Ok) {
        last_submitted_ = position; has_submitted_ = true; consecutive_failures_ = 0;
    } else { ++failures_; ++consecutive_failures_; }
    return status;
}
}  // namespace jc
