#pragma once

#include "joint_controller/spsc_ring.hpp"
#include "joint_controller/types.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace jc {

class DataLogger {
public:
    DataLogger();
    ~DataLogger();
    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    bool start(const std::string& path, const LoopConfig& config, std::string& error);
    CycleRecord* reserve_record() noexcept;
    void publish_record() noexcept { records_->publish(); }
    void finish(const LoopSummary& summary);
    std::uint64_t dropped_records() const noexcept { return dropped_.load(std::memory_order_relaxed); }
    int io_error() const noexcept { return io_error_.load(std::memory_order_relaxed); }

private:
    void worker(std::string path);
    void set_io_error(int error) noexcept;
    std::unique_ptr<SpscRing<CycleRecord, kLogCapacity>> records_;
    std::unique_ptr<char[]> file_buffer_;
    std::atomic<std::uint64_t> dropped_{};
    std::atomic<int> io_error_{};
    std::atomic<bool> finish_requested_{};
    std::thread thread_;
    std::mutex startup_mutex_;
    std::condition_variable startup_cv_;
    bool startup_done_{};
    LoopConfig config_{};
    LoopSummary summary_{};
};

}  // namespace jc
