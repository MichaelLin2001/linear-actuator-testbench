#ifndef _APP_COMM_H_
#define _APP_COMM_H_
#include "stdint.h"
#include <csignal>
#include "Console.hpp"
#include "comm_message.h"
#include "transmit.h"

struct CMD {
    std::string name;
    unsigned (*func)(const std::vector<std::string>& input);
    std::string help;
};
extern const std::vector<struct CMD> app_cmd_list;

unsigned app_start(const std::vector<std::string>& input);
unsigned app_motor_write(const std::vector<std::string>& input);
unsigned app_motor_read(const std::vector<std::string>& input);
unsigned app_set_runmode(const std::vector<std::string>& input);
unsigned app_m18_enable(const std::vector<std::string>& input);
unsigned app_m18_load_raw(const std::vector<std::string>& input);
unsigned app_m18_csp_hold(const std::vector<std::string>& input);
unsigned app_m18_status(const std::vector<std::string>& input);
unsigned app_m18_move_j3_relative(const std::vector<std::string>& input);
void app_cmd_help_printf(std::string cmd_name);

/**
 * Initialize motor control application should before user set running mode
 */
void app_motor_control_init(void);
void app_motor_control_deinit(void);

void app_register_cmd(CppReadline::Console& console, std::string& info);
void app_set_running(AppMode mode);
void app_set_runing_callback(AppMode mode);
AppMode app_get_running(void);
void app_motor_cmd_packet(struct can_frame* frame, uint8_t motor_id, MHandle handle, MotorParam param);
void app_set_motor_fdb(uint8_t index, float fdb_speed, float fdb_pos, float fdb_torque);
void app_get_motor_fdb(uint8_t index, MHandle* motorHandle);
void app_set_motor_target(uint8_t index, float speed, float pos, float torque);
void app_get_motor_target(uint8_t index, MHandle* motorHandle);
void app_get_all_motor_target(MHandle* motorHandle);
void app_set_motor_target_handle(uint8_t index, MHandle motorHandle);
void app_get_all_motor_fdb(MHandle* motorHandle);
void app_set_motor_runmode(uint8_t index, uint8_t runmode);
bool app_lock_current_position(void);
extern volatile sig_atomic_t app_exit_requested;

#endif  // _APP_COMM_H_
