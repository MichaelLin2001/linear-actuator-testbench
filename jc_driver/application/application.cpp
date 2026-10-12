#include "application.h"
#include "comm_message.h"
#include "app_comm.h"

void app_init(CppReadline::Console& console, std::string& info)
{
    app_register_cmd(console, info);
    app_motor_control_init();
}

void app_deinit(void)
{
    if (app_m18_enable({"AppM18Enable", "DIS_DRV"}) != 0)
        fprintf(stderr, "Exit: J3 disable could not be verified; check the drive before restarting.\n");
}