#pragma once

#include "joint_controller/types.hpp"

#include <semaphore.h>
#include <string>

namespace jc {

struct RawTarget {
    std::uint8_t run_mode{};
    std::uint8_t motor_type{};
    std::uint8_t reserved[2]{};
    float target_speed{};
    float target_position{};
    float target_effort{};
};
using RawTargetArray = std::array<RawTarget, kJointCount>;

static_assert(std::is_standard_layout<RawTarget>::value);
static_assert(std::is_trivially_copyable<RawTarget>::value);
static_assert(sizeof(RawTarget) == 16);
static_assert(sizeof(RawTargetArray) == 480);
static_assert(offsetof(RawTarget, run_mode) == 0);
static_assert(offsetof(RawTarget, target_speed) == 4);
static_assert(offsetof(RawTarget, target_position) == 8);
static_assert(offsetof(RawTarget, target_effort) == 12);

inline RawTarget csp_target(RawTarget previous,double position_m) noexcept {
    previous.run_mode=8;previous.target_speed=0;
    previous.target_position=static_cast<float>(position_m*1000.0);previous.target_effort=0;
    return previous;
}

inline RawTarget cst_target(RawTarget previous,double raw) noexcept {
    previous.run_mode=10;previous.target_speed=0;previous.target_effort=static_cast<float>(std::round(raw));
    const auto sequence=static_cast<std::uint16_t>((static_cast<unsigned>(previous.reserved[0])|
        (static_cast<unsigned>(previous.reserved[1])<<8))+1U);
    previous.reserved[0]=static_cast<std::uint8_t>(sequence);previous.reserved[1]=static_cast<std::uint8_t>(sequence>>8);
    return previous;
}

enum class TargetWriteStatus : std::uint8_t {
    Ok,
    Contended,
    Interrupted,
    SemaphoreError,
    ReleaseError,
    NotAttached,
    InvalidJoint
};

class JcTargetAdapter {
public:
    JcTargetAdapter() = default;
    ~JcTargetAdapter();
    JcTargetAdapter(const JcTargetAdapter&) = delete;
    JcTargetAdapter& operator=(const JcTargetAdapter&) = delete;

    bool attach(std::string& error);
    TargetWriteStatus write_csp(std::size_t joint, double position_si) noexcept { return write(joint,position_si,false); }
    TargetWriteStatus write_cst(double raw) noexcept { return write(kJ3,raw,true); }

private:
    TargetWriteStatus write(std::size_t joint,double value,bool cst) noexcept;
    void* target_address_{};
    sem_t* target_semaphore_{SEM_FAILED};
};

}  // namespace jc
