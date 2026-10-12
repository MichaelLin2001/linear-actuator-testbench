#include "jc_log.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <cstdarg>
#include <mutex>
#include <fstream>
#include <vector>
#include <condition_variable>
#include <thread>

#define JC_LOG_USE_COLOR 0

#if JC_LOG_USE_COLOR
#define COLOR_RESET "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_WHITE "\033[37m"
#else
#define COLOR_RESET ""
#define COLOR_RED ""
#define COLOR_GREEN ""
#define COLOR_YELLOW ""
#define COLOR_BLUE ""
#define COLOR_MAGENTA ""
#define COLOR_CYAN ""
#define COLOR_WHITE ""
#endif

static std::ofstream g_logFile;
static std::vector<std::string> g_logBuffer;
static std::mutex g_logMutex;
static std::condition_variable g_logCv;
static bool g_exitFlag = false;

static std::thread logThread;

const size_t LOG_BUFFER_SIZE = 20;
const std::chrono::milliseconds LOG_FLUSH_INTERVAL(500);

void logFlushThread(void)
{
    while (!g_exitFlag) {
        std::unique_lock<std::mutex> lock(g_logMutex);
        g_logCv.wait_for(lock, LOG_FLUSH_INTERVAL, [] { return !g_logBuffer.empty() || g_exitFlag; });

        if (!g_logBuffer.empty()) {
            for (auto& line : g_logBuffer) {
                g_logFile << line << "\n";
            }
            g_logFile.flush();
            g_logBuffer.clear();
        }
    }

    for (auto& line : g_logBuffer) {
        g_logFile << line << "\n";
    }
    g_logFile.flush();
    g_logBuffer.clear();
}

void initLogger(const char* filePath = "/tmp/j3-testbench/app.log")
{
    g_logFile.open(filePath, std::ios::app);
    if (!g_logFile.is_open()) {
        std::cerr << "Failed to open log file: " << filePath << std::endl;
    }

    logThread = std::thread(logFlushThread);
}

void deInitLogger(void)
{
    g_exitFlag = true;
    g_logCv.notify_one();
    logThread.join();
}

static std::string getCurrentTime()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

static std::pair<const char*, const char*> getLogLevelInfo(Level level)
{
    switch (level) {
        case DEBUG:
            return {"D", COLOR_CYAN};
        case INFO:
            return {"I", COLOR_GREEN};
        case WARNING:
            return {"W", COLOR_YELLOW};
        case ERROR:
            return {"E", COLOR_RED};
        case FATAL:
            return {"F", COLOR_MAGENTA};
        default:
            return {"UNKNOWN", COLOR_WHITE};
    }
}

void _JC_LOG(Level level, int line, const char* func, const char* format, ...)
{
    auto [levelStr, color] = getLogLevelInfo(level);
    std::string timeStr = getCurrentTime();

    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    {
        std::string logLine = std::string(color) + timeStr + " [" + levelStr + " " + func + " " + std::to_string(line) + "] " + COLOR_RESET + buffer;
        std::lock_guard<std::mutex> lock(g_logMutex);
        g_logBuffer.push_back(logLine);

        if (g_logBuffer.size() >= LOG_BUFFER_SIZE) {
            g_logCv.notify_one();
        }
    }
}
