#include "joint_controller/driver_manager.hpp"
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace jc {
namespace {
constexpr const char* kStart = "/home/nvidia/Documents/jc_driver/tools/single_j3/start_driver.sh";
constexpr const char* kSend = "/home/nvidia/Documents/jc_driver/tools/single_j3/send_command.sh";
constexpr const char* kLog = "/tmp/jc_driver_single_j3/session.log";
constexpr const char* kLauncherLog = "/tmp/jc_driver_single_j3/stage2_launcher.log";
std::size_t file_size(const char* path) { struct stat st{}; return stat(path, &st) == 0 ? static_cast<std::size_t>(st.st_size) : 0; }
std::string read_from(const char* path, std::size_t offset) {
    std::ifstream in(path); if (!in) return {}; in.seekg(static_cast<std::streamoff>(offset));
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}
bool run_wait(const char* program, const std::string& arg, std::string& error) {
    const pid_t pid = fork();
    if (pid < 0) { error = std::string("fork: ") + std::strerror(errno); return false; }
    if (pid == 0) { execl(program, program, arg.c_str(), static_cast<char*>(nullptr)); _exit(127); }
    int status{}; while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) { error = std::string(program) + " failed"; return false; }
    return true;
}
int last_integer_after(const std::string& text, const std::string& marker) {
    const auto pos = text.rfind(marker); if (pos == std::string::npos) return -1;
    try { return std::stoi(text.substr(pos + marker.size())); } catch (...) { return -1; }
}
std::string last_statusword(const std::string& text) {
    const std::string marker = "J3 Statusword: "; const auto pos = text.rfind(marker);
    if (pos == std::string::npos) return "unknown";
    const auto begin = pos + marker.size(); const auto end = text.find(' ', begin);
    return text.substr(begin, end == std::string::npos ? text.size()-begin : end-begin);
}
}
DriverManager::~DriverManager() {
    if (session_pid_ > 0) {
        std::string error;
        if(disable(error).state_confirmed) shutdown(error);
        else std::cerr << "STOP NOT CONFIRMED; keep driver available for manual stop: " << error << '\n';
    }
}
bool DriverManager::start(std::string& error) {
    if (session_pid_ > 0) return true;
    const auto offset = file_size(kLog);
    const pid_t pid = fork();
    if (pid < 0) { error = std::string("fork driver: ") + std::strerror(errno); return false; }
    if (pid == 0) {
        const int fd = open(kLauncherLog, O_WRONLY|O_CREAT|O_APPEND, 0644);
        if (fd >= 0) { dup2(fd, STDOUT_FILENO); dup2(fd, STDERR_FILENO); close(fd); }
        execl(kStart, kStart, static_cast<char*>(nullptr)); _exit(127);
    }
    session_pid_ = pid;
    for (unsigned i=0; i<120; ++i) {
        int status{}; if (waitpid(pid, &status, WNOHANG) == pid) {
            session_pid_ = -1; error = "jc_driver launcher exited before command prompt"; return false;
        }
        if (read_from(kLog, offset).find("[Command] >") != std::string::npos) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    error = "timeout waiting for jc_driver command prompt"; return false;
}
bool DriverManager::send(const std::string& command, std::string& response, std::string& error,
                         unsigned timeout_seconds) {
    const bool stop_command=command=="AppStart STOP"||command=="AppM18Enable DIS_DRV"||command=="AppM18Status"||command=="quit";
    if(stop_requested_.load()&&!stop_command){error="command cancelled by emergency stop";return false;}
    if (session_pid_ <= 0 || kill(session_pid_, 0) != 0) { error = "managed jc_driver is not running"; return false; }
    const auto offset = file_size(kLog);
    if (!run_wait(kSend, command, error)) return false;
    const unsigned polls = timeout_seconds * 20;
    for (unsigned i=0; i<polls; ++i) {
        if(stop_requested_.load()&&!stop_command){error="command interrupted by emergency stop";return false;}
        response = read_from(kLog, offset);
        // The command text is appended after a prompt that may already be before
        // offset.  Completion is the following prompt in the newly appended data.
        const auto command_at = response.find(command);
        if (command_at != std::string::npos) {
            const auto next_prompt = response.find("[Command] >", command_at + command.size());
            if (next_prompt != std::string::npos || command == "quit") return true;
        }
        int status{}; if (waitpid(session_pid_, &status, WNOHANG) == session_pid_) {
            session_pid_ = -1;
            if (command == "quit") return true;
            error = "jc_driver exited while processing command"; return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    error = "timeout waiting for jc_driver response to: " + command; return false;
}
DriverStatus DriverManager::parse_status(const std::string& text) {
    DriverStatus s{}; s.raw = text; s.command_sent = true; s.statusword = last_statusword(text);
    s.operation_enabled = text.rfind("(Operation enabled)") != std::string::npos;
    s.switch_on_disabled = text.rfind("(Switch on disabled)") != std::string::npos;
    s.mode_display = last_integer_after(text, "J3 Mode display: ");
    const std::string position_marker = "J3 Position: ";
    const auto position_at = text.rfind(position_marker);
    if (position_at != std::string::npos) {
        try {
            s.position_m = std::stod(text.substr(position_at + position_marker.size())) * 0.001;
            s.position_known = true;
        } catch (...) { s.position_known = false; }
    }
    if (s.operation_enabled) s.state_text = "Operation enabled";
    else if (s.switch_on_disabled) s.state_text = "Switch on disabled";
    s.process_running = true; return s;
}
DriverStatus DriverManager::query(std::string& error) {
    DriverStatus status{};
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        std::string response;
        error.clear();
        if (!send("AppM18Status", response, error)) { return {}; }
        status = parse_status(response);
        status.state_confirmed = status.statusword != "unknown";
        if (status.state_confirmed) { return status; }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    error = "drive status was sent but no valid state was returned";
    return status;
}
DriverStatus DriverManager::enable(std::string& error) {
    std::string response;
    DriverStatus result{};
    if (!send("AppM18Enable RESET CONFIRM", response, error)) return result;
    result.command_sent = true;
    // The verified stage-1 sequence allowed the M18 node five seconds to
    // finish NMT reset before remapping PDOs and starting guarded enable.
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (!send("AppM18Enable EN_DRV CONFIRM", response, error)) return result;
    auto checked = query(error); checked.command_sent = true;
    if (!checked.operation_enabled || checked.mode_display != 8) { error = "drive did not confirm Operation enabled with mode 8"; return checked; }
    if (!send("AppStart POS", response, error)) return checked;
    checked = query(error); checked.command_sent = true;
    checked.state_confirmed = checked.operation_enabled && checked.mode_display == 8;
    if (!checked.state_confirmed) error = "post-AppStart status confirmation failed";
    return checked;
}
DriverStatus DriverManager::stop_application(std::string& error) {
    std::string response; DriverStatus s{};
    if (!send("AppStart STOP", response, error)) return s;
    s = query(error); s.command_sent = true;
    s.state_confirmed = s.statusword != "unknown"; return s;
}
DriverStatus DriverManager::disable(std::string& error) {
    std::string response; DriverStatus s{};
    (void)send("AppStart STOP", response, error);
    if (!send("AppM18Enable DIS_DRV", response, error)) { s.command_sent = true; return s; }
    s = query(error); s.command_sent = true; s.state_confirmed = s.switch_on_disabled;
    if (!s.state_confirmed) error = "drive did not confirm Switch on disabled";
    return s;
}
bool DriverManager::shutdown(std::string& error) {
    if (session_pid_ <= 0) return true;
    std::string response; const pid_t pid = session_pid_;
    if (!send("quit", response, error, 8)) return false;
    for (unsigned i=0; i<80; ++i) {
        int status{}; if (waitpid(pid, &status, WNOHANG) == pid) { session_pid_ = -1; return true; }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    error = "timeout waiting for managed jc_driver shutdown"; return false;
}
}  // namespace jc
