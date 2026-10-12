#include "joint_controller/jc_target_adapter.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <sys/ipc.h>
#include <sys/shm.h>

namespace jc {
namespace {
constexpr char kTargetKeyFile[] = "/tmp/j3-testbench/target_handle";
constexpr char kTargetSemaphore[] = "/j3_testbench_target_mutex";
}

JcTargetAdapter::~JcTargetAdapter() {
    if (target_address_ != nullptr) {
        shmdt(target_address_);
    }
    if (target_semaphore_ != SEM_FAILED) {
        sem_close(target_semaphore_);
    }
}

bool JcTargetAdapter::attach(std::string& error) {
    if (target_address_ != nullptr) {
        error = "The target segment is already attached.";
        return false;
    }
    const key_t key = ftok(kTargetKeyFile, 0x01);
    if (key == static_cast<key_t>(-1)) {
        error = std::string("ftok(target): ") + std::strerror(errno);
        return false;
    }
    const int id = shmget(key, sizeof(RawTargetArray), 0);
    if (id == -1) {
        error = std::string("shmget(existing target): ") + std::strerror(errno);
        return false;
    }
    shmid_ds info{};
    if (shmctl(id, IPC_STAT, &info) == -1) {
        error = std::string("shmctl(target metadata): ") + std::strerror(errno);
        return false;
    }
    if (info.shm_segsz != sizeof(RawTargetArray)) {
        error = "Target ABI mismatch: expected exactly 480 bytes (30 x 16).";
        return false;
    }
    void* const mapped = shmat(id, nullptr, 0);
    if (mapped == reinterpret_cast<void*>(-1)) {
        error = std::string("shmat(target read-write): ") + std::strerror(errno);
        return false;
    }
    sem_t* const semaphore = sem_open(kTargetSemaphore, 0);
    if (semaphore == SEM_FAILED) {
        const int saved_error = errno;
        shmdt(mapped);
        error = std::string("sem_open(existing target lock): ") + std::strerror(saved_error);
        return false;
    }
    target_address_ = mapped;
    target_semaphore_ = semaphore;
    error.clear();
    return true;
}

TargetWriteStatus JcTargetAdapter::write(std::size_t joint,double value,bool cst) noexcept {
    if (joint != kJ3 || !std::isfinite(value)||(cst&&std::abs(value)>32767)) {
        return TargetWriteStatus::InvalidJoint;
    }
    if (target_address_ == nullptr || target_semaphore_ == SEM_FAILED) {
        return TargetWriteStatus::NotAttached;
    }
    if (sem_trywait(target_semaphore_) != 0) {
        if (errno == EAGAIN) {
            return TargetWriteStatus::Contended;
        }
        if (errno == EINTR) {
            return TargetWriteStatus::Interrupted;
        }
        return TargetWriteStatus::SemaphoreError;
    }

    auto* const slots = static_cast<RawTarget*>(target_address_);
    RawTarget target{};
    std::memcpy(&target, slots + joint, sizeof(target));
    target = cst?cst_target(target,value):csp_target(target,value);
    std::memcpy(slots + joint, &target, sizeof(target));

    if (sem_post(target_semaphore_) != 0) {
        return TargetWriteStatus::ReleaseError;
    }
    return TargetWriteStatus::Ok;
}

}  // namespace jc
