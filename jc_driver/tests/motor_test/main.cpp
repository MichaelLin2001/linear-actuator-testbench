#include <cstdlib>
#include "command_session.hpp"
#include <csignal>
#include <readline/readline.h>
#include "app_comm.h"
#include <cstdio>
#include <cerrno>
#include <sys/stat.h>
#include "Console.hpp"
#include "time.h"
#include "application.h"
#include "m18.h"
#include "jc_log.h"
extern "C" {
#include "ethercat.h"
#include "utils.h"
}

using CRRCode = CppReadline::Console::ReturnCode;
std::string g_cmd_info;

unsigned help(const std::vector<std::string>&)
{
    std::cout << g_cmd_info;
    return 0;
}

#define CPU_BOND 1
#define CPU_PRIORITY 90
static int set_realtime_scheduler(int cpu_core, int priority)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("sched_setaffinity failed");
        return -1;
    }

    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed (need root)");
        return -1;
    }

    return 0;
}

static void request_exit(int) { app_exit_requested = 1; }

static CommandSession* session=nullptr;
static CppReadline::Console* console=nullptr;

static int check_exit()
{
    if (!app_exit_requested && session && console) {
        try {
            if(session->poll([](const std::string& command){return console->executeCommand(command);}))
                app_exit_requested=1;
        }catch(const std::exception& error){
            fprintf(stderr,"Driver command channel failed: %s\n",error.what());app_exit_requested=1;
        }
    }
    if (app_exit_requested) {
        rl_replace_line("", 0);
        rl_done = 1;
    }
    return 0;
}

int main()
{
    if (set_realtime_scheduler(CPU_BOND, CPU_PRIORITY) != 0) {
        JC_LOG("Real-time scheduling failed to be configured, Exit!");
        return -1;
    }

    if (mkdir("/tmp/j3-testbench", 0777) != 0 && errno != EEXIST) {
        perror("mkdir /tmp/j3-testbench");
        return 1;
    }
    std::unique_ptr<CommandSession> command_session;
    try {command_session=std::make_unique<CommandSession>("/tmp/j3-testbench");}
    catch(const std::exception& error){fprintf(stderr,"%s\n",error.what());return 1;}
    initLogger("/tmp/j3-testbench/app.log");

    CppReadline::Console cli("[Command] > ");

    cli.registerCommand("help", help);

    // Readline must leave SIGINT to our flag-only handler.
    rl_catch_signals = 0;
    rl_event_hook = check_exit;
    struct sigaction action{};
    action.sa_handler = request_exit;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, nullptr) != 0 ||
        sigaction(SIGTERM, &action, nullptr) != 0) {
        perror("sigaction");
        return 1;
    }

    shared_memory_init();


    JC_LOG_INFO("JC Driver Start...");
    JC_LOG_INFO("EtherCAT Master: SOEM");
    const char* iface = std::getenv("JC_ETHERCAT_IFACE");
    ethercat_init(iface && *iface ? iface : "enp2s0");

    if (ec_slavecount <= 0) {
        JC_LOG_ERROR("Slave not found, program exits!");
        return 1;
    } else {
        JC_LOG_INFO("Slave Count: %d", ec_slavecount);
    }

    ethercat_thread_start();

    app_init(cli, g_cmd_info);
    session=command_session.get();console=&cli;

    while (!app_exit_requested && cli.readLine() != CRRCode::Quit) {}
    JC_LOG("Command Line Exited...");

    app_deinit();
    ethercat_thread_stop();
    JC_LOG("EtherCAT Thread Stop...");

    deInitLogger();

    JC_LOG("JC Driver Stop...");
    return 0;
}
