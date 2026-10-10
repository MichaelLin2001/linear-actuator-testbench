#pragma once
#include "joint_controller/force_sensor_reader.hpp"
#include <cmath>
#include <cstdint>
namespace jc {
enum class ZeroState : std::uint8_t { None, Collecting, Complete, Failed };
struct ProcessedForce {
    double raw{}, zero{}, external{};
    std::uint64_t timestamp_ns{}, zero_samples{};
    std::uint16_t package_no{};
    ZeroState zero_state{ZeroState::None};
    bool valid{}, zeroed{}, new_sample{};
};
// Owned exclusively by ControlLoop. Commands and status cross threads separately.
class ForceProcessor {
public:
    static constexpr std::uint64_t timeout_ns=100'000'000;
    void begin_zero(std::uint64_t now) noexcept;
    void cancel_zero() noexcept;
    ProcessedForce update(const ForceSample& sample,std::uint64_t now) noexcept;
    const ProcessedForce& value() const noexcept { return value_; }
private:
    ProcessedForce value_{};
    std::uint64_t zero_started_{}, first_zero_sample_{};
    double sum_{}, minimum_{}, maximum_{};
};
}
