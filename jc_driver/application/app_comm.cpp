#include "app_comm.h"
#include "jc_log.h"
#include "m18.h"
#include <cstring>
#include <atomic>
#include <cmath>
#include "utils.h"

const char* mode_list[RM_MAX - 1] = {"PP", "FP", "PV", "PT", "R2", "HM", "R3", "CSP", "CSV", "CST"};

volatile sig_atomic_t app_exit_requested = 0;

static std::atomic<AppMode> app_start_running{APP_MODE_STOP};

__attribute__((weak)) void app_set_runing_callback(AppMode mode)
{
    // Weak function, can be overridden
}

void app_set_running(AppMode mode)
{
    app_start_running = mode;
    app_set_runing_callback(mode);
}

AppMode app_get_running(void)
{
    return app_start_running;
}


static void _app_m18_motor_cmd_packet(struct can_frame* frame, uint8_t motor_id, MHandle handle, MotorParam param)
{
    if (param.run_mode == RM_CSP) {
        int32_t _target_pos = MM_2_UNIT(handle.target_pos);
        m18_build_csp_message(frame, motor_id, _target_pos);
    } else if (param.run_mode == RM_CST) {
        if(!std::isfinite(handle.target_torque)||std::abs(handle.target_torque)>32767)return;
        int16_t _target_tor = static_cast<int16_t>(std::round(handle.target_torque));
        m18_build_cst_message(frame, motor_id, _target_tor);
    }
}

void app_motor_cmd_packet(struct can_frame* frame, uint8_t motor_id, MHandle handle, MotorParam param)
{
    if (!frame) {
        JC_LOG_ERROR("frame is NULL.");
        return;
    }

    if (motor_id == 0) {
        JC_LOG_ERROR("motor id is 0.");
        return;
    }

    switch (param.motor_type) {
        case MT_M18:
            _app_m18_motor_cmd_packet(frame, motor_id, handle, param);
            break;
        default:
            JC_LOG_WARN("Unknown Motor Type: %#x", param.motor_type);
            break;
    }
}

unsigned app_set_runmode(const std::vector<std::string>& input)
{
    int index;
    uint8_t run_mode = 0;

    if (input.size() != 3) {
        goto out;
    }

    index = std::stoi(input[1]);
    if (index != 3) goto out;
    for (int i = 0; i < RM_MAX - 1; i++) {
        if (input[2] == mode_list[i]) {
            run_mode = i + 1;
            break;
        }
    }

    if (run_mode > 0 && run_mode < RM_MAX) {
        app_set_motor_runmode(index, run_mode);
        return 0;
    }

out:
    app_cmd_help_printf("AppSetRunMode");

    return 1;
}

unsigned app_start(const std::vector<std::string>& input)
{
    int start = 0;
    switch (input.size() - 1) {
        case 1:
            if (input[1] == "TOR") {
                fprintf(stderr,"Use guarded AppM18LoadRaw; AppStart TOR is disabled\n");return 1;
            } else if (input[1] == "POS") {
                if (!app_lock_current_position()) return 1;
                app_set_running(APP_MODE_POS);
            } else if (input[1] == "USER") {
                app_set_running(APP_MODE_USER);
            } else if (input[1] == "STOP") {
                app_set_running(APP_MODE_STOP);
                ethercat_reset_continue_tx_buffers();
            } else {
                app_cmd_help_printf("AppStart");
                return 1;
            }
            break;
        default:
            app_cmd_help_printf("AppStart");
            return 1;
    }

    JC_LOG_INFO("APP Start: %s", input[1].c_str());
    return 0;
}

const std::vector<struct CMD> app_cmd_list = {
    {"AppStart",      app_start,       "- AppStart <mode>\n  mode: TOR | POS | USER | STOP"                                    },
    {"AppM18LoadRaw", app_m18_load_raw, "- AppM18LoadRaw <initial_raw> <limit_raw> (controller only)"},
    {"AppM18CspHold", app_m18_csp_hold, "- AppM18CspHold (after zero raw command)"},
    {"AppWrite", app_motor_write, "- AppWrite 3 <absolute_mm>"},
    {"AppRead",       app_motor_read,  "- AppRead <index>"                                                                     },
    {"AppSetRunMode", app_set_runmode, "- AppSetRunMode <index> <runMode>\n  runMode: PP | PV | PT | FP | HM | CSP | CSV | CST"},
    {"AppM18Enable",  app_m18_enable,  "- AppM18Enable <CMD>\n  CMD: RESET | EN_DRV | DIS_DRV"                  },
    {"AppM18Status",  app_m18_status,  "- AppM18Status"                                                                         },
    {"AppMoveJ3Relative", app_m18_move_j3_relative, "- AppMoveJ3Relative <delta_mm>"                                    },
};

void app_cmd_help_printf(std::string cmd_name)
{
    for (const auto& cmd : app_cmd_list) {
        if (cmd.name == cmd_name) {
            std::cout << "Command format error::\n" << cmd.help << "\n";
            return;
        }
    }
}

void app_register_cmd(CppReadline::Console& console, std::string& info)
{
    const auto& cmds = app_cmd_list;

    info.append("APP CMD:\n");
    for (const auto& cmd : cmds) {
        console.registerCommand(cmd.name, cmd.func);
        info.append(cmd.help + "\n");
    }
}
