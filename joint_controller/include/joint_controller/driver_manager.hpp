#pragma once
#include <string>
#include <atomic>
#include <sys/types.h>
namespace jc {
struct DriverStatus {
    bool process_running{};
    bool command_sent{};
    bool state_confirmed{};
    bool operation_enabled{};
    bool switch_on_disabled{};
    int mode_display{-1};
    double position_m{};
    bool position_known{};
    std::string statusword{"unknown"};
    std::string state_text{"unknown"};
    std::string raw;
};
class DriverManager {
public:
    ~DriverManager();
    void request_stop() noexcept { stop_requested_.store(true); }
    void clear_stop() noexcept { stop_requested_.store(false); }
    bool start(std::string& error);
    DriverStatus query(std::string& error);
    DriverStatus enable(std::string& error);
    DriverStatus stop_application(std::string& error);
    DriverStatus disable(std::string& error);
    bool shutdown(std::string& error);
    bool managed() const noexcept { return session_pid_ > 0; }
private:
    bool send(const std::string& command, std::string& response, std::string& error,
              unsigned timeout_seconds = 15);
    static DriverStatus parse_status(const std::string& text);
    pid_t session_pid_{-1};
    std::atomic<bool> stop_requested_{};
};
}  // namespace jc
