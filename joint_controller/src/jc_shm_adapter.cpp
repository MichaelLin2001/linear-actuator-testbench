#include "joint_controller/jc_shm_adapter.hpp"

#include <cerrno>
#include <cstring>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <time.h>

namespace jc {
namespace {
constexpr char kFeedbackKeyFile[] = "/home/jc_robot/feedback_handle";
constexpr char kFeedbackSemaphore[] = "sem_feedback_handle_mutex";

std::uint64_t now_ns() noexcept {
    timespec value{};
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0;
    }
    return static_cast<std::uint64_t>(value.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(value.tv_nsec);
}
}  // namespace

JcShmAdapter::~JcShmAdapter() {
    if (feedback_address_ != nullptr) {
        shmdt(feedback_address_);
    }
    if (feedback_semaphore_ != SEM_FAILED) {
        sem_close(feedback_semaphore_);
    }
}

bool JcShmAdapter::attach(std::string& error) {
    if (feedback_address_ != nullptr) {
        error = "The feedback segment is already attached.";
        return false;
    }
    const key_t key = ftok(kFeedbackKeyFile, 0x02);
    if (key == static_cast<key_t>(-1)) {
        error = std::string("ftok(feedback): ") + std::strerror(errno);
        return false;
    }
    const int id = shmget(key, sizeof(RawFeedbackArray), 0);
    if (id == -1) {
        error = std::string("shmget(existing feedback): ") + std::strerror(errno);
        return false;
    }
    shmid_ds info{};
    if (shmctl(id, IPC_STAT, &info) == -1) {
        error = std::string("shmctl(feedback metadata): ") + std::strerror(errno);
        return false;
    }
    if (info.shm_segsz != sizeof(RawFeedbackArray)) {
        error = "Feedback ABI mismatch: expected exactly 480 bytes (30 x 16).";
        return false;
    }
    const void* mapped = shmat(id, nullptr, SHM_RDONLY);
    if (mapped == reinterpret_cast<void*>(-1)) {
        error = std::string("shmat(read-only feedback): ") + std::strerror(errno);
        return false;
    }
    sem_t* semaphore = sem_open(kFeedbackSemaphore, 0);
    if (semaphore == SEM_FAILED) {
        const int saved_error = errno;
        shmdt(mapped);
        error = std::string("sem_open(existing feedback lock): ") + std::strerror(saved_error);
        return false;
    }
    feedback_address_ = mapped;
    feedback_semaphore_ = semaphore;
    error.clear();
    return true;
}

FeedbackSnapshot JcShmAdapter::read_snapshot() noexcept {
    FeedbackSnapshot snapshot{};
    ++stats_.total_reads;
    const auto start = now_ns();
    if (start == 0) {
        snapshot.read_status = ReadStatus::ClockError;
    } else if (feedback_address_ == nullptr || feedback_semaphore_ == SEM_FAILED) {
        snapshot.read_status = ReadStatus::NotAttached;
    } else if (sem_trywait(feedback_semaphore_) != 0) {
        snapshot.error_number = errno;
        if (snapshot.error_number == EAGAIN) {
            snapshot.read_status = ReadStatus::Contended;
            ++stats_.semaphore_contention_count;
        } else if (snapshot.error_number == EINTR) {
            snapshot.read_status = ReadStatus::Interrupted;
        } else {
            snapshot.read_status = ReadStatus::SemaphoreError;
        }
    } else {
        // Keep the shared critical section to one whole-array copy and release.
        std::memcpy(snapshot.raw.data(), feedback_address_, sizeof(RawFeedbackArray));
        const int release_result = sem_post(feedback_semaphore_);
        if (release_result == 0) {
            snapshot.read_status = ReadStatus::Ok;
            snapshot.snapshot_read_ok = true;
        } else {
            snapshot.error_number = errno;
            snapshot.read_status = ReadStatus::ReleaseError;
            snapshot.raw = {};
        }
    }
    const auto end = now_ns();
    if (end == 0 || start == 0 || end < start) {
        snapshot.snapshot_read_ok = false;
        snapshot.read_status = ReadStatus::ClockError;
        snapshot.raw = {};
    } else {
        snapshot.snapshot_read_duration_ns = end - start;
    }
    if (snapshot.snapshot_read_duration_ns > stats_.maximum_read_duration_ns) {
        stats_.maximum_read_duration_ns = snapshot.snapshot_read_duration_ns;
    }
    if (snapshot.snapshot_read_ok) {
        snapshot.snapshot_read_timestamp_ns = end;
        stats_.last_success_timestamp_ns = end;
        ++stats_.successful_reads;
        stats_.consecutive_failed_reads = 0;
    } else {
        ++stats_.failed_reads;
        ++stats_.consecutive_failed_reads;
    }
    convert_feedback(snapshot);
    return snapshot;
}

}  // namespace jc
