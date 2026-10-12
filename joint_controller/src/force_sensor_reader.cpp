#include "joint_controller/force_sensor_reader.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace jc {
namespace {
constexpr char kStartCommand[] = "AT+GSD\r\n";
constexpr char kStopCommand[] = "AT+GSD=STOP\r\n";
constexpr std::size_t kMinimumFrameLength = 10;
constexpr std::size_t kMaximumFrameLength = 4096;

std::uint64_t monotonic_timestamp_ns() noexcept {
    timespec value{};
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0;
    }
    return static_cast<std::uint64_t>(value.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(value.tv_nsec);
}

bool configure_serial(int fd, unsigned baud) noexcept {
    const speed_t speed=baud==115200?B115200:baud==230400?B230400:baud==460800?B460800:baud==921600?B921600:0;
    if(speed==0) return false;
    termios settings{};
    if (tcgetattr(fd, &settings) != 0) {
        return false;
    }
    cfmakeraw(&settings);
    if (cfsetispeed(&settings, speed) != 0 || cfsetospeed(&settings, speed) != 0) {
        return false;
    }
    settings.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
    settings.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
#ifdef CRTSCTS
    settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    settings.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &settings) == 0;
}
}  // namespace

ForceSensorReader::~ForceSensorReader() {
    stop();
}

bool ForceSensorReader::start(const std::string& device, unsigned baud) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load(std::memory_order_acquire)) {
        return true;
    }
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }

    const int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return false;
    }
    if (!configure_serial(fd,baud) || tcflush(fd, TCIOFLUSH) != 0) {
        close(fd);
        return false;
    }

    serial_fd_ = fd;
    stop_requested_.store(false, std::memory_order_release);
    sample_valid_.store(false, std::memory_order_release);
    if (!send_command(kStartCommand, sizeof(kStartCommand) - 1U)) {
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    running_.store(true, std::memory_order_release);
    try {
        reader_thread_ = std::thread(&ForceSensorReader::reader_loop, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    return true;
}

void ForceSensorReader::stop() noexcept {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (serial_fd_ >= 0) {
        (void)send_command(kStopCommand, sizeof(kStopCommand) - 1U);
    }
    stop_requested_.store(true, std::memory_order_release);
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    running_.store(false, std::memory_order_release);
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool ForceSensorReader::send_command(const char* command, std::size_t length) noexcept {
    if (serial_fd_ < 0) {
        return false;
    }
    std::size_t sent = 0;
    while (sent < length) {
        const ssize_t count = write(serial_fd_, command + sent, length - sent);
        if (count > 0) {
            sent += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd descriptor{serial_fd_, POLLOUT, 0};
            if (poll(&descriptor, 1, 100) > 0) {
                continue;
            }
        }
        return false;
    }
    return true;
}

void ForceSensorReader::publish(float force_n, std::uint16_t package_no,
                                std::uint64_t timestamp_ns) noexcept {
    const double value = static_cast<double>(force_n);
    std::uint64_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));

    const auto sequence = sample_sequence_.load();
    sample_sequence_.store(sequence + 1U);
    if(frames_.load()==0)first_timestamp_.store(timestamp_ns);
    else {
        const auto delta=static_cast<std::uint16_t>(package_no-static_cast<std::uint16_t>(package_no_.load()));
        if(delta==0)duplicates_.fetch_add(1);
        else if(delta<32768U)missing_.fetch_add(static_cast<std::uint64_t>(delta-1U));
        else discontinuities_.fetch_add(1);
    }
    frames_.fetch_add(1);
    force_bits_.store(bits);
    package_no_.store(package_no);
    timestamp_ns_.store(timestamp_ns);
    sample_valid_.store(true);
    sample_sequence_.store(sequence + 2U);
}

ForceSample ForceSensorReader::get_latest() const noexcept {
    ForceSample sample{};
    for (unsigned attempt = 0; attempt < 4U; ++attempt) {
        const auto before = sample_sequence_.load();
        if ((before & 1U) != 0U) {
            continue;
        }
        const auto bits = force_bits_.load();
        sample.package_no = static_cast<std::uint16_t>(
            package_no_.load());
        sample.timestamp_ns = timestamp_ns_.load();
        sample.valid = sample_valid_.load();
        std::memcpy(&sample.force_n, &bits, sizeof(bits));
        const auto after = sample_sequence_.load();
        if (before == after && (after & 1U) == 0U) {
            return sample;
        }
    }
    return ForceSample{};
}
ForceStatistics ForceSensorReader::statistics() const noexcept {
    for(unsigned attempt=0;attempt<4;++attempt){
        const auto before=sample_sequence_.load();if(before&1U)continue;
        ForceStatistics s{frames_.load(),missing_.load(),duplicates_.load(),discontinuities_.load(),first_timestamp_.load(),timestamp_ns_.load()};
        if(before==sample_sequence_.load())return s;
    }
    return {};
}

void ForceSensorReader::reader_loop() noexcept {
    std::array<std::uint8_t, kMaximumFrameLength * 2U> buffer{};
    std::size_t buffered = 0;

    while (!stop_requested_.load(std::memory_order_acquire)) {
        pollfd descriptor{serial_fd_, POLLIN, 0};
        const int poll_result = poll(&descriptor, 1, 20);
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            break;
        }
        if (poll_result == 0 || (descriptor.revents & POLLIN) == 0) {
            continue;
        }

        if (buffered == buffer.size()) {
            buffered = 0;
        }
        const ssize_t count = read(serial_fd_, buffer.data() + buffered, buffer.size() - buffered);
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            break;
        }
        if (count == 0) {
            continue;
        }
        buffered += static_cast<std::size_t>(count);

        std::size_t consumed = 0;
        while (buffered - consumed >= 2U) {
            while (buffered - consumed >= 2U &&
                   (buffer[consumed] != 0xAAU || buffer[consumed + 1U] != 0x55U)) {
                ++consumed;
            }
            if (buffered - consumed < 4U) {
                break;
            }
            const auto frame_length = static_cast<std::size_t>(
                (static_cast<std::uint16_t>(buffer[consumed + 2U]) << 8U) |
                static_cast<std::uint16_t>(buffer[consumed + 3U]));
            if (frame_length < kMinimumFrameLength || frame_length > kMaximumFrameLength) {
                ++consumed;
                continue;
            }
            if (buffered - consumed < frame_length) {
                break;
            }

            const auto package_no = static_cast<std::uint16_t>(
                (static_cast<std::uint16_t>(buffer[consumed + 4U]) << 8U) |
                static_cast<std::uint16_t>(buffer[consumed + 5U]));
            const std::uint32_t raw_force = static_cast<std::uint32_t>(buffer[consumed + 6U]) |
                (static_cast<std::uint32_t>(buffer[consumed + 7U]) << 8U) |
                (static_cast<std::uint32_t>(buffer[consumed + 8U]) << 16U) |
                (static_cast<std::uint32_t>(buffer[consumed + 9U]) << 24U);
            float force{};
            std::memcpy(&force, &raw_force, sizeof(force));
            const auto timestamp = monotonic_timestamp_ns();
            if (std::isfinite(force) && timestamp != 0U) {
                publish(force, package_no, timestamp);
            }
            consumed += frame_length;
        }

        if (consumed != 0U) {
            const auto remaining = buffered - consumed;
            std::memmove(buffer.data(), buffer.data() + consumed, remaining);
            buffered = remaining;
        }
    }
    running_.store(false, std::memory_order_release);
}

}  // namespace jc
