#pragma once
#include "joint_controller/force_processor.hpp"
#include <semaphore.h>
#include <string>
namespace jc {
// Same SysV segment/named-semaphore pattern as the motor feedback adapter.
class ForceShm {
public:
    ~ForceShm();
    bool attach(bool writer, std::string& error, const std::string& key_file="/tmp/j3-testbench/force_handle",
                const std::string& semaphore="/j3_testbench_force_mutex");
    bool publish(const ProcessedForce&) noexcept;
    // read_time_ns is sampled after the snapshot copy, on CLOCK_MONOTONIC.
    ProcessedForce read(std::uint64_t& read_time_ns) noexcept;
private:
    void* address_{}; sem_t* semaphore_{SEM_FAILED}; int owner_fd_{-1}; bool writer_{};
    ProcessedForce latest_{};
    std::uint64_t previous_session_{},previous_sample_{};
};
inline bool same_force_zero(const ProcessedForce& a,const ProcessedForce& b) noexcept {
    return a.session_id!=0 && a.session_id==b.session_id && a.zero_version==b.zero_version &&
           b.valid && b.zeroed && b.zero_state==ZeroState::Complete;
}
}
