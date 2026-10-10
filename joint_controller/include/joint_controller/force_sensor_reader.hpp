#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <string>

namespace jc {

struct ForceSample {
    double force_n{};
    std::uint16_t package_no{};
    std::uint64_t timestamp_ns{};
    bool valid{};
};
struct ForceStatistics {
    std::uint64_t frames{}, missing_packages{}, duplicate_packages{}, discontinuities{}, first_timestamp_ns{}, last_timestamp_ns{};
};

class ForceSensorReader {
public:
    ForceSensorReader() noexcept = default;
    ~ForceSensorReader();

    ForceSensorReader(const ForceSensorReader&) = delete;
    ForceSensorReader& operator=(const ForceSensorReader&) = delete;

    bool start(const std::string& device="/dev/ttyUSB0", unsigned baud=115200);
    void stop() noexcept;
    ForceSample get_latest() const noexcept;
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    ForceStatistics statistics() const noexcept;

private:
    bool send_command(const char* command, std::size_t length) noexcept;
    void reader_loop() noexcept;
    void publish(float force_n, std::uint16_t package_no, std::uint64_t timestamp_ns) noexcept;

    mutable std::mutex lifecycle_mutex_;
    int serial_fd_{-1};
    std::thread reader_thread_;
    std::atomic<bool> stop_requested_{};
    std::atomic<bool> running_{};

    std::atomic<std::uint64_t> sample_sequence_{};
    std::atomic<std::uint64_t> force_bits_{};
    std::atomic<std::uint64_t> timestamp_ns_{};
    std::atomic<std::uint32_t> package_no_{};
    std::atomic<bool> sample_valid_{};
    std::atomic<std::uint64_t> frames_{}, missing_{}, duplicates_{}, discontinuities_{}, first_timestamp_{};
};

}  // namespace jc
