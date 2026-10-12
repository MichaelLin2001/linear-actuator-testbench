#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include <stdexcept>
#include <cerrno>
#include <cmath>
#include <thread>
#include <atomic>
#include <time.h>
#include "transmit.h"
#include "queue.h"
#include "SharedMemory.hpp"
#include "jc01.h"
#include "math.h"
#include "stddef.h"
#include "m18.h"
#include "jc_log.h"
#include "comm_message.h"
extern "C" {
#include "utils.h"
}
#include "triple_map.h"

#define DC_VOLT_WARN_LIMIT (42.0f)

#define MOTOR_COUNT_MAX (30)

#define SHM_TARGET_PATH "/tmp/j3-testbench/target_handle"
#define SHM_TARGET_SEM_NAME "/j3_testbench_target"
#define SHM_TARGET_KEY_NUM (0x01)
static SharedMemory* shm_target;

#define SHM_FEEDBACK_PATH "/tmp/j3-testbench/feedback_handle"
#define SHM_FEEDBACK_SEM_NAME "/j3_testbench_feedback"
#define SHM_FEEDBACK_KEY_NUM (0x02)
static SharedMemory* shm_feedback;
static std::atomic<bool> raw_fault{false};
static std::atomic<unsigned> raw_generation{0};
static std::atomic<int> raw_limit{0};
static uint64_t driver_now_ns(){
    timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t)!=0)return 0;
    return static_cast<uint64_t>(t.tv_sec)*1000000000ULL+static_cast<uint64_t>(t.tv_nsec);
}
static bool raw_lease_alive(uint16_t sequence,uint64_t now,unsigned generation){
    static unsigned previous_generation=0;static uint16_t previous_sequence=0;
    static uint64_t last_update=0;static bool expired=true;
    if(generation!=previous_generation){previous_generation=generation;previous_sequence=sequence;last_update=now;expired=false;}
    if(expired||now==0||now<last_update)return false;
    if(now-last_update>=100000000ULL){expired=true;return false;}
    if(sequence!=previous_sequence){previous_sequence=sequence;last_update=now;}
    return true;
}


MotorParam gMotorParam[MOTOR_COUNT_MAX] = {
    {}, {}, {},
    {.motor_type = MT_M18, .run_mode = RM_CSP, .tor_const = 0.0404, .lead_mm = 0.0024, .gear_ratio = 1, .eff_total = 1, .pos_limit = {}}
};
MHandle gMotorHandle[MOTOR_COUNT_MAX] = {0};
static_assert(sizeof(MHandle) == 16, "jc01 shared-memory slot ABI must remain 16 bytes");
static_assert(sizeof(gMotorHandle) == 480, "jc01 shared-memory array ABI must remain 480 bytes");

#define J03_NODE_ID 1
#define SLAVE_EXPECT_COUNT 1
#define M18_SLAVE_ID 0
#define J03_SAFE_MOVE_STEP_MM (0.01f)
#define J03_ENABLE_MAX_SHIFT_MM (0.02f)
/**
 * M18 is always expected to be the last slave
 */
enum {
    J03_TX_PASSGE = 1,
    J03_RX_PASSGE,
    J03_CFG_PASSGE,
} M18_PASSAGE_FUNC;

// Single J3 on EtherCAT slave index 0.
const static int jx_map[SLAVE_EXPECT_COUNT][ECAT_SLAVE_PER_MOTOR_COUNT] = {
    {3,  -1, -1, -1, -1, -1}
};

// Set the communication ID of each passage of the slave
const static uint8_t id_map[SLAVE_EXPECT_COUNT][ECAT_SLAVE_PER_MOTOR_COUNT] = {
    {J03_NODE_ID, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};

/*
 * @brief  : Calculates the linear actuator thrust based on motor phase current.
 * @param  :
 *      motorCurrent_A        - Motor phase current (A).
 *      MotorParam            - Motor paramter.
 *  @return:
 *      float - Calculated thrust(N) or Load torque(Nmm) depend on motor type.
 *
 * Notes:
 *     The calculation is based on:
 *         1. Motor torque = Kt × current
 *         2. Load torque  = motor torque × gear ratio × efficiency
 *         3. Thrust       = (2 × π × load torque) / lead
 */
static float cal_actuator_force(float motorCurrent_A, MotorParam motor_param)
{
    float motorTorque = motor_param.tor_const * motorCurrent_A;

    float loadTorque = motorTorque * motor_param.gear_ratio * motor_param.eff_total;


    float force = (2.0f * (float)(M_PI)*loadTorque) / motor_param.lead_mm * 1000;

    return force;
}

void app_get_motor_fdb(uint8_t index, MHandle* handle)
{
    const uint32_t offset_base = (index) * sizeof(MHandle);
    shm_feedback->read(&handle->feedback_speed, 1, offset_base + offsetof(MHandle, feedback_speed));
    shm_feedback->read(&handle->feedback_pos, 1, offset_base + offsetof(MHandle, feedback_pos));
    shm_feedback->read(&handle->feedback_torque, 1, offset_base + offsetof(MHandle, feedback_torque));
}

void app_get_motor_target(uint8_t index, MHandle* handle)
{
    const uint32_t offset_base = (index) * sizeof(MHandle);
    shm_target->read(&handle->target_speed, 1, offset_base + offsetof(MHandle, target_speed));
    shm_target->read(&handle->target_pos, 1, offset_base + offsetof(MHandle, target_pos));
    shm_target->read(&handle->target_torque, 1, offset_base + offsetof(MHandle, target_torque));
}

void app_get_all_motor_target(MHandle* handle)
{
    shm_target->read(handle, MOTOR_COUNT_MAX, 0);
}

static int read_j3_state(uint16_t*, int8_t*, int32_t*);

bool app_lock_current_position(void)
{
    // Capture targets while cyclic output is stopped. Switching the software
    // packet format to CST before the M18 drive has left CSP makes a zeroed
    // position field look like a real CSP target and causes a position kick.
    app_set_running(APP_MODE_STOP);
    ethercat_reset_continue_tx_buffers();

    uint16_t status = 0;
    int8_t mode = 0;
    int32_t position = 0;
    const int ret = read_j3_state(&status, &mode, &position);
    const uint16_t state = status & 0x006F;
    if (app_exit_requested || ret != 0 || (status & 0x0008) ||
        (state != 0x0040 && state != 0x0021 && state != 0x0023 && state != 0x0027) ||
        (state == 0x0027 && mode != MRUN_CSP)) {
        fprintf(stderr, "AppStart POS rejected: ret=%d status=0x%04X mode=%d; output remains STOP\n",
                ret, status, mode);
        return false;
    }
    const float target = UNIT_2_MM(position);
    const double counts = MM_2_UNIT(target);
    if (!std::isfinite(counts) || counts < INT32_MIN || counts > INT32_MAX) return false;
    app_set_motor_runmode(3, RM_CSP);
    app_set_motor_target(3, 0, target, 0);
    JC_LOG_INFO("J3 POS hold aligned from SDO: %.5f mm", target);
    return true;
}

void app_set_runing_callback(AppMode mode)
{
    if (mode == APP_MODE_TOR) {
        gMotorParam[3].run_mode = RM_CST;
    } else if (mode == APP_MODE_POS) {
        gMotorParam[3].run_mode = RM_CSP;
    }
}

void app_set_motor_fdb(uint8_t index, float fdb_speed, float fdb_pos, float fdb_torque)
{
    const uint32_t offset_base = (index) * sizeof(MHandle);

    shm_feedback->write(&fdb_speed, 1, offset_base + offsetof(MHandle, feedback_speed));
    shm_feedback->write(&fdb_pos, 1, offset_base + offsetof(MHandle, feedback_pos));
    shm_feedback->write(&fdb_torque, 1, offset_base + offsetof(MHandle, feedback_torque));
}

void app_set_all_motor_fdb(MHandle* handle)
{
    shm_feedback->write(handle, MOTOR_COUNT_MAX, 0);
}

void app_set_motor_runmode(uint8_t index, uint8_t mode)
{
    const uint32_t offset_base = (index) * sizeof(MHandle);

    uint8_t cur_mode = 0;
    shm_target->read(&cur_mode, 1, offset_base + offsetof(MHandle, run_mode));

    // Keep position when switching CST -> CSP to avoid a jump caused by stale target_pos.
    if (mode == RM_CSP && cur_mode == RM_CST) {
        MHandle fdb_handle = {0};
        app_get_motor_fdb(index, &fdb_handle);
        JC_LOG_INFO("Motor %d CST->CSP, target=%.4f", index, fdb_handle.feedback_pos);
        app_set_motor_target(index, 0, fdb_handle.feedback_pos, 0);
    }

    shm_target->write(&mode, 1, offset_base + offsetof(MHandle, run_mode));
}

void app_set_motor_target(uint8_t index, float speed, float pos, float torque)
{
    const uint32_t offset_base = (index) * sizeof(MHandle);
    shm_target->write(&speed, 1, offset_base + offsetof(MHandle, target_speed));
    shm_target->write(&pos, 1, offset_base + offsetof(MHandle, target_pos));
    shm_target->write(&torque, 1, offset_base + offsetof(MHandle, target_torque));
}

static void app_shared_memory_clear(SharedMemory* shm)
{
    MHandle handles[MOTOR_COUNT_MAX];
    memset(handles, 0, sizeof(handles));
    shm->write(handles, MOTOR_COUNT_MAX, 0);
    JC_LOG_INFO("shared memory Clear.");
}

void txMessageCallBack(void)
{
    EtherCAT_Msg message[SLAVE_EXPECT_COUNT] = {0};
    MHandle motor_handles[MOTOR_COUNT_MAX] = {0};
    AppMode app_mode;

    uint8_t slave_id = 0, passage = 0;
    uint16_t id = 0;

    app_mode = app_get_running();
    if (app_mode != APP_MODE_STOP) {
        app_get_all_motor_target(motor_handles);

        if(app_mode==APP_MODE_TOR){
            const auto& target=motor_handles[3];
            const auto sequence=static_cast<uint16_t>(target.reserved[0]|(static_cast<unsigned>(target.reserved[1])<<8));
            if(!raw_lease_alive(sequence,driver_now_ns(),raw_generation.load())||!std::isfinite(target.target_torque)||
                std::abs(target.target_torque)>raw_limit.load())raw_fault.store(true);
            if(raw_fault.load()){
                m18_build_cst_message(&message[0].motor[0],J03_NODE_ID,0);
                canopen_build_sync(&message[0].motor[2],SYNC_TYPE_BASIC);
                canopen_build_ds402_state_request(&message[0].motor[1],J03_NODE_ID,DS402_STATE_SWITCH_ON_DISABLED);
                ethercat_send_message_continue(0,message[0]);return;
            }
        }

        if (triple_map_decode(3, &slave_id, &passage, &id) == 0) {
            app_motor_cmd_packet(&message[slave_id].motor[passage - 1], id, motor_handles[3], gMotorParam[3]);
            canopen_build_sync(&message[slave_id].motor[passage + 1], SYNC_TYPE_BASIC);
        }

        for (int slave_id = 0; slave_id < ec_slavecount && slave_id < SLAVE_EXPECT_COUNT; slave_id++) {
            ethercat_send_message_continue(slave_id, message[slave_id]);
        }
    }
}

static uint16_t decode_m18_txpdo1(const uint8_t* data, float* fdb_pos, float* fdb_torque)
{
    int32_t _fdb_pos = (int32_t)data[0] | ((int32_t)data[1] << 8) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 24);

    const uint16_t raw = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    const int32_t raw_6077 = raw <= 0x7FFF ? raw : static_cast<int32_t>(raw) - 65536;
    uint16_t error_code = (uint16_t)(data[6] | (data[7] << 8));

    *fdb_pos = UNIT_2_MM(_fdb_pos);
    *fdb_torque = static_cast<float>(raw_6077); // Preserve raw feedback; no thrust calibration.

    return error_code;
}

static void decode_m18_txpdo2(const uint8_t* data, float* fdb_vel, float* fdb_dc_volt)
{
    int32_t _fdb_vel = (int32_t)data[0] | ((int32_t)data[1] << 8) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 24);
    uint32_t _fdb_dc_volt = (uint32_t)data[4] | ((uint32_t)data[5] << 8) | ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);

    *fdb_vel = UNIT_2_MM(_fdb_vel);
    *fdb_dc_volt = (float)_fdb_dc_volt / 1000.0f;
}

static void decode_m18_response(MHandle* handle, const MotorParam* param, const struct can_frame msg)
{
    uint8_t func_code = CANOPEN_GET_FC(msg.id);
    float dc_link_volt = 0.0f;
    uint16_t error_code = 0;

    switch (func_code) {
        case CANOPEN_FC_TPDO1:
            error_code = decode_m18_txpdo1(msg.data, &handle->feedback_pos, &handle->feedback_torque);
            break;
        case CANOPEN_FC_TPDO2:
            decode_m18_txpdo2(msg.data, &handle->feedback_speed, &dc_link_volt);
            break;
        default:
            break;
    }

    //TODO: handle error_code and dc_link_volt
    if (error_code) {
        JC_LOG_ERROR("M18 %s (error_code=0x%04X)", ((error_code == 0xFF54) || (error_code == 0xE017)) ? "encoder battery undervoltage" : "motor error", error_code);
    }
}

void rxMessageCallBack(void)
{
    static MHandle _fdb_handles[MOTOR_COUNT_MAX] = {0};

    for (int _slave_id = 0; _slave_id < ec_slavecount; _slave_id++) {
        if (rx_message_queue[_slave_id].read_available() == 0) {
            continue;
        }

        EtherCAT_Msg_ptr rx_msg;
        if (!rx_message_queue[_slave_id].pop(rx_msg)) {
            continue;
        }
        for (int _passage = 0; _passage < ECAT_SLAVE_PER_MOTOR_COUNT; _passage++) {
            if (rx_msg->motor[_passage].dlc == 0) {
                continue;
            }
            int32_t jx_index;
            if (_slave_id != M18_SLAVE_ID) {
                jx_index = triple_map_encode(_slave_id, _passage + 1, rx_msg->motor[_passage].id);
            } else {
                jx_index = triple_map_encode(_slave_id, _passage + 1, CANOPEN_GET_NODE_ID(rx_msg->motor[_passage].id));
            }

            if (jx_index >= 0 && gMotorParam[jx_index].decode_frame != nullptr) {
                gMotorParam[jx_index].decode_frame(&_fdb_handles[jx_index], &gMotorParam[jx_index], rx_msg->motor[_passage]);
            }
        }

        ethercat_set_query_response(_slave_id, *rx_msg.get());
        ethercat_notify_response();
    }

    _fdb_handles[3].reserved[0]=2; // Raw-current protocol version, within the existing ABI.
    _fdb_handles[3].reserved[1]=raw_fault.load()?1:0;
    app_set_all_motor_fdb(_fdb_handles);
}

static void m18_driver_init(void)
{
    EtherCAT_Msg message = {0};
    uint32_t tpdo1_map[] = {0x60640020, 0x60770010, 0x603F0010};
    uint32_t tpdo2_map[] = {0x606C0020, 0x60790020};
    uint8_t tpdo1_map_num = sizeof(tpdo1_map) / sizeof(tpdo1_map[0]);
    uint8_t tpdo2_map_num = sizeof(tpdo2_map) / sizeof(tpdo2_map[0]);
    uint32_t rpdo1_map[] = {0x607A0020, 0x60710010};
    uint8_t rpdo1_map_num = sizeof(rpdo1_map) / sizeof(rpdo1_map[0]);

    m18_reset_node(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID);
    usleep(2000 * 1000);

    JC_LOG_INFO("Set M18 PDO Map");
    m18_set_pdo_map(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID, CANOPEN_FC_RPDO1, rpdo1_map, rpdo1_map_num);
    m18_set_pdo_map(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID, CANOPEN_FC_TPDO1, tpdo1_map, tpdo1_map_num);
    m18_set_pdo_map(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID, CANOPEN_FC_TPDO2, tpdo2_map, tpdo2_map_num);

    usleep(50 * 1000);


    /**
     * TODO: Set the target position to the current position value
     */
    m18_start_node(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID);

    usleep(2000 * 1000);
    m18_set_mode(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID, gMotorParam[3].run_mode);
}


void shared_memory_init(void)
{
    shm_target = new SharedMemory(SHM_TARGET_PATH, sizeof(MHandle) * MOTOR_COUNT_MAX, SHM_TARGET_KEY_NUM, LOCK_TYPE_MUTEX, SHM_TARGET_SEM_NAME);
    shm_feedback = new SharedMemory(SHM_FEEDBACK_PATH, sizeof(MHandle) * MOTOR_COUNT_MAX, SHM_FEEDBACK_KEY_NUM, LOCK_TYPE_MUTEX, SHM_FEEDBACK_SEM_NAME);

    if (!(shm_target->sharedMemValid() && shm_feedback->sharedMemValid())) {
        JC_LOG_ERROR("shared memory is invalid, motor contorl thread start fail!");
        return;
    }

    JC_LOG_INFO("Start Connect ShareMemory");
    shm_target->connect();
    shm_feedback->connect();
    app_shared_memory_clear(shm_target);
    app_shared_memory_clear(shm_feedback);
}

void app_motor_control_init(void)
{
    if (ec_slavecount < SLAVE_EXPECT_COUNT) {
        JC_LOG_WARN("slave count is not the expected value, ec_slavecount(%d)!=SLAVE_EXPECT_COUNT(%d)", ec_slavecount, SLAVE_EXPECT_COUNT);
        // return;
    }

    triple_map_init(jx_map, id_map, SLAVE_EXPECT_COUNT);
    gMotorParam[3].decode_frame = decode_m18_response;


    // Startup stays silent until an explicit command reads a valid position.


    app_set_running(APP_MODE_STOP);
    ethercat_reset_continue_tx_buffers();
    JC_LOG_WARN("J3 is not initialized or enabled at startup. Inspect AppM18Status before guarded enable.");
    JC_LOG_INFO("JC-01 Motor Control Thread Start");
}

static void app_set_pos_step(uint8_t index, float actual, float target, float step)
{
    float _tor = 0, _speed = 0;
    const long period_ns = ECAT_PDO_PERIOD_MS * 1000 * 1000;
    struct timespec next, now, actualPeriodTime, actualTime;
    struct timespec cycleTime = {0, period_ns};
    int ret;

    long overtime_ns;
    cycleTime.tv_sec = period_ns / 1000000000;
    cycleTime.tv_nsec = period_ns % 1000000000;

    clock_gettime(CLOCK_MONOTONIC, &next);

    app_set_motor_target(index, _speed, actual, _tor);

    if (actual > target) {
        step = -step;
    }

    float _step_target = actual;
    while (abs(_step_target - target) > 0.01) {
        if (app_exit_requested) return;
        _step_target += step;
        app_set_motor_target(index, _speed, _step_target, _tor);
        struct timespec ideal_next = timespec_add(&next, &cycleTime);
        clock_gettime(CLOCK_MONOTONIC, &now);
        overtime_ns = TIMESPEC2NS(timespec_minus(&now, &ideal_next));

        if (overtime_ns > 0) {
            // JC_LOG_WARN("PDO Cycle Overtime: %8lld ns, Resynchronizing...", overtime_ns);
            next = timespec_add(&now, &cycleTime);
        } else {
            next = ideal_next;
        }

        do {
            ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
            if (ret == EINTR) {
                JC_LOG_WARN("Sleep interrupted by signal, continuing...");
            } else if (ret != 0) {
                JC_LOG_ERROR("clock_nanosleep error: %d", ret);
                break;
            }
        } while (ret == EINTR);
    }

    if (!app_exit_requested) app_set_motor_target(index, _speed, target, _tor);
}

static const char* m18_ds402_state_name(uint16_t status_word)
{
    // Bit 5 is not part of the DS402 fault/fault-reaction state masks.
    if ((status_word & 0x004F) == 0x000F) return "Fault reaction active";
    if ((status_word & 0x004F) == 0x0008) return "Fault";
    switch (status_word & 0x006F) {
        case 0x0000:
            return "Not ready to switch on";
        case 0x0040:
            return "Switch on disabled";
        case 0x0021:
            return "Ready to switch on";
        case 0x0023:
            return "Switched on";
        case 0x0027:
            return "Operation enabled";
        case 0x0007:
            return "Quick stop active";
        default:
            return "Unknown";
    }
}

static int read_j3_state(uint16_t* status_word, int8_t* operation_mode, int32_t* position_count)
{
    uint32_t value = 0;
    uint8_t value_size = 0;

    int ret = m18_read_sdo(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID, 0x6041, 0, &value, &value_size);
    if (ret != 0 || value_size != 2) {
        return (ret != 0) ? ret : -1;
    }
    *status_word = static_cast<uint16_t>(value);

    ret = m18_read_sdo(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID, 0x6061, 0, &value, &value_size);
    if (ret != 0 || value_size != 1) {
        return (ret != 0) ? ret : -2;
    }
    *operation_mode = static_cast<int8_t>(value & 0xFF);

    ret = m18_read_sdo(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID, 0x6064, 0, &value, &value_size);
    if (ret != 0 || value_size != 4) {
        return (ret != 0) ? ret : -3;
    }
    *position_count = static_cast<int32_t>(value);

    return 0;
}

unsigned app_m18_status(const std::vector<std::string>& input)
{
    if (input.size() != 1) {
        app_cmd_help_printf("AppM18Status");
        return 1;
    }

    uint16_t status_word = 0;
    int8_t operation_mode = 0;
    int32_t position_count = 0;
    const int ret = read_j3_state(&status_word, &operation_mode, &position_count);
    if (ret != 0) {
        JC_LOG_ERROR("J3 status read failed, ret=%d", ret);
        return 1;
    }

    printf("J3 Statusword: 0x%04X (%s)\n", status_word, m18_ds402_state_name(status_word));
    printf("J3 Mode display: %d%s\n", operation_mode, operation_mode == MRUN_CSP ? " (CSP)" : "");
    printf("J3 Position: %.5f mm (%d counts)\n", UNIT_2_MM(position_count), position_count);
    return 0;
}

static unsigned app_move_j3(double value_mm, bool relative)
{
    if (app_get_running() != APP_MODE_POS || gMotorParam[3].run_mode != RM_CSP) {
        JC_LOG_ERROR("J3 relative move requires AppStart POS and CSP mode");
        return 1;
    }

    uint16_t status_word = 0;
    int8_t operation_mode = 0;
    int32_t position_count = 0;
    const int ret = read_j3_state(&status_word, &operation_mode, &position_count);
    if (ret != 0) {
        JC_LOG_ERROR("J3 pre-move status read failed, ret=%d", ret);
        return 1;
    }
    if ((status_word & 0x006F) != 0x0027) {
        JC_LOG_ERROR("J3 is not Operation enabled: statusword=0x%04X (%s)",
                     status_word, m18_ds402_state_name(status_word));
        return 1;
    }
    if (operation_mode != MRUN_CSP) {
        JC_LOG_ERROR("J3 mode display is %d, expected CSP(%d)", operation_mode, MRUN_CSP);
        return 1;
    }

    const float sdo_position_mm = UNIT_2_MM(position_count);
    MHandle feedback = {0};
    app_get_motor_fdb(3, &feedback);
    if (!std::isfinite(feedback.feedback_pos) || std::fabs(feedback.feedback_pos - sdo_position_mm) > 0.25f) {
        JC_LOG_ERROR("J3 TPDO/SDO position mismatch: TPDO=%.5f mm SDO=%.5f mm",
                     feedback.feedback_pos, sdo_position_mm);
        return 1;
    }

    const double target_mm = relative ? sdo_position_mm + value_mm : value_mm;
    const double delta_mm = target_mm - sdo_position_mm;
    const double counts = target_mm * static_cast<double>(POS_UNIT);
    if (!std::isfinite(counts) || counts < INT32_MIN || counts > INT32_MAX) {
        JC_LOG_ERROR("J3 target exceeds signed 32-bit position range");
        return 1;
    }
    const float target = static_cast<float>(target_mm);
    // Check the float conversion used by the cyclic packet before its int32 cast.
    const double encoded_counts = MM_2_UNIT(target);
    if (encoded_counts < INT32_MIN || encoded_counts > INT32_MAX) {
        JC_LOG_ERROR("J3 target rounds outside signed 32-bit position range");
        return 1;
    }
    if (delta_mm == 0) return 0;
    JC_LOG_WARN("J3 move accepted: current=%.5f mm delta=%.5f mm target=%.5f mm",
                sdo_position_mm, delta_mm, target_mm);
    app_set_pos_step(3, sdo_position_mm, target, J03_SAFE_MOVE_STEP_MM);
    return 0;
}

static unsigned app_move_input(const std::vector<std::string>& input, bool relative)
{
    if (input.size() != (relative ? 2u : 3u) || (!relative && input[1] != "3")) {
        app_cmd_help_printf(relative ? "AppMoveJ3Relative" : "AppWrite");
        return 1;
    }
    try {
        const auto& text = input[relative ? 1 : 2];
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used != text.size() || !std::isfinite(value)) return 1;
        return app_move_j3(value, relative);
    } catch (const std::exception& exception) {
        JC_LOG_ERROR("invalid J3 position: %s", exception.what());
        return 1;
    }
}

unsigned app_m18_move_j3_relative(const std::vector<std::string>& input) { return app_move_input(input, true); }
unsigned app_motor_write(const std::vector<std::string>& input) { return app_move_input(input, false); }

unsigned app_motor_read(const std::vector<std::string>& input)
{
    int index;
    switch (input.size() - 1) {
        case 1:
            index = std::stoi(input[1]);
            break;
        default:
            app_cmd_help_printf("AppRead");
            return 1;
    }

    if (index != 3) { JC_LOG_ERROR("only J3 index 3 is available"); return 1; }

    MHandle handle_fdb = {0};
    MHandle handle_tgt = {0};
    app_get_motor_fdb(index, &handle_fdb);
    app_get_motor_target(index, &handle_tgt);
    printf("┌───────────────────────────────────────┐\n");
    printf("│%*sMotor J%-2d%*s│\n", 15, "", index, 15, "");
    printf("├──────┬────────────┬────────────┬──────┤\n");
    printf("│ Type │   Actual   │  Target    │ Unit │\n");
    printf("├──────┼────────────┼────────────┼──────┤\n");
    printf("│ VEL  │ %10.5f │ %10.5f │ mm/s │\n", handle_fdb.feedback_speed, handle_tgt.target_speed);

    printf("│ POS  │ %10.5f │ %10.5f │  mm  │\n", handle_fdb.feedback_pos, handle_tgt.target_pos);

    printf("│ Iraw │ %10.0f │          - │ raw  │\n", handle_fdb.feedback_torque);
    printf("│ Icmd │          - │ %10.0f │ raw  │\n", handle_tgt.target_torque);
    printf("Feedback: signed raw 0x6077; amperes scale unverified\n");
    printf("└──────┴────────────┴────────────┴──────┘\n");
    return 0;
}

static unsigned mode_failure(const char* reason){
    app_set_running(APP_MODE_STOP);ethercat_reset_continue_tx_buffers();
    raw_fault.store(true);m18_disable_driver(M18_SLAVE_ID,J03_TX_PASSGE,J03_NODE_ID);
    fprintf(stderr,"J3 mode handover failed: %s; disable requested\n",reason);return 1;
}
static bool verify_object(uint16_t index,uint32_t expected,uint8_t width){
    uint32_t value=0;uint8_t size=0;
    return m18_read_sdo(M18_SLAVE_ID,J03_CFG_PASSGE,J03_NODE_ID,index,0,&value,&size)==0&&size==width&&
        (width==2?(value&0xFFFF)==(expected&0xFFFF):value==expected);
}
unsigned app_m18_load_raw(const std::vector<std::string>& input){
    int seed=0,limit=0;
    try{size_t a=0,b=0;if(input.size()!=3)return 1;
        seed=std::stoi(input[1],&a);limit=std::stoi(input[2],&b);
        if(a!=input[1].size()||b!=input[2].size()||limit<=0||limit>32767||std::abs(static_cast<long long>(seed))>limit)return 1;
    }catch(...){return 1;}
    uint16_t status=0;int8_t mode=0;int32_t position=0;
    if(app_exit_requested||read_j3_state(&status,&mode,&position)!=0||(status&0x006F)!=0x0027||mode!=MRUN_CSP)return mode_failure("CSP not enabled");
    const int32_t initial_position=position;
    app_set_running(APP_MODE_STOP);ethercat_reset_continue_tx_buffers();
    const auto bits=static_cast<uint16_t>(static_cast<int16_t>(seed));
    if(m18_write_sdo(M18_SLAVE_ID,J03_CFG_PASSGE,J03_NODE_ID,0x6071,0,bits,SDO_WRITE_2B)!=0||!verify_object(0x6071,bits,2))return mode_failure("raw target verification");
    if(m18_write_sdo(M18_SLAVE_ID,J03_CFG_PASSGE,J03_NODE_ID,0x6060,0,MRUN_CST,SDO_WRITE_1B)!=0)return mode_failure("CST request");
    for(unsigned i=0;i<20;++i){
        if(app_exit_requested||read_j3_state(&status,&mode,&position)!=0||(status&0x006F)!=0x0027||
            std::abs(static_cast<double>(position)-initial_position)>MM_2_UNIT(J03_ENABLE_MAX_SHIFT_MM))return mode_failure("CST state or displacement");
        if(mode==MRUN_CST){
            app_set_motor_runmode(3,RM_CST);app_set_motor_target(3,0,UNIT_2_MM(position),static_cast<float>(seed));
            raw_limit.store(limit);raw_fault.store(false);raw_generation.fetch_add(1);
            app_set_running(APP_MODE_TOR);printf("RAW_CST_READY %d\n",seed);return 0;
        }
        usleep(10000);
    }
    return mode_failure("CST mode display timeout");
}
unsigned app_m18_csp_hold(const std::vector<std::string>& input){
    if(input.size()!=1)return 1;
    uint16_t status=0;int8_t mode=0;int32_t position=0;
    MHandle target{};app_get_motor_target(3,&target);
    if(app_exit_requested||raw_fault.load()||target.target_torque!=0||read_j3_state(&status,&mode,&position)!=0||
        (status&0x006F)!=0x0027||mode!=MRUN_CST)return mode_failure("CST zero handover not ready");
    const int32_t initial_position=position;
    app_set_running(APP_MODE_STOP);ethercat_reset_continue_tx_buffers();
    if(m18_write_sdo(M18_SLAVE_ID,J03_CFG_PASSGE,J03_NODE_ID,0x6071,0,0,SDO_WRITE_2B)!=0||!verify_object(0x6071,0,2)||
        m18_write_sdo(M18_SLAVE_ID,J03_CFG_PASSGE,J03_NODE_ID,0x607A,0,static_cast<uint32_t>(position),SDO_WRITE_4B)!=0||
        !verify_object(0x607A,static_cast<uint32_t>(position),4))return mode_failure("CSP target verification");
    if(m18_write_sdo(M18_SLAVE_ID,J03_CFG_PASSGE,J03_NODE_ID,0x6060,0,MRUN_CSP,SDO_WRITE_1B)!=0)return mode_failure("CSP request");
    for(unsigned i=0;i<20;++i){
        if(app_exit_requested||read_j3_state(&status,&mode,&position)!=0||(status&0x006F)!=0x0027||
            std::abs(static_cast<double>(position)-initial_position)>MM_2_UNIT(J03_ENABLE_MAX_SHIFT_MM))return mode_failure("CSP state or displacement");
        if(mode==MRUN_CSP){
            app_set_motor_runmode(3,RM_CSP);app_set_motor_target(3,0,UNIT_2_MM(position),0);
            app_set_running(APP_MODE_POS);printf("RAW_CSP_READY\n");return 0;
        }
        usleep(10000);
    }
    return mode_failure("CSP mode display timeout");
}

unsigned app_m18_enable(const std::vector<std::string>& input)
{
    if (input.size() == 2 && input[1] == "DIS_DRV") {
        app_set_running(APP_MODE_STOP);
        ethercat_reset_continue_tx_buffers();
        m18_disable_driver(M18_SLAVE_ID, J03_TX_PASSGE, J03_NODE_ID);
        usleep(250 * 1000);

        uint16_t status_word = 0;
        int8_t operation_mode = 0;
        int32_t position_count = 0;
        const int ret = read_j3_state(&status_word, &operation_mode, &position_count);
        if (ret != 0) {
            JC_LOG_ERROR("J3 disable status read failed, ret=%d", ret);
            return 1;
        }
        if ((status_word & 0x006F) == 0x0027) {
            JC_LOG_ERROR("J3 disable failed: statusword=0x%04X", status_word);
            return 1;
        }
        JC_LOG_WARN("J3 disabled: statusword=0x%04X (%s)",
                    status_word, m18_ds402_state_name(status_word));
        return 0;
    }

    if (input.size() != 2) {
        app_cmd_help_printf("AppM18Enable");
        return 1;
    }

    if (input[1] == "RESET") {
        app_set_running(APP_MODE_STOP);
        ethercat_reset_continue_tx_buffers();
        m18_reset_node(M18_SLAVE_ID, J03_TX_PASSGE, J03_NODE_ID);
    } else if (input[1] == "EN_DRV") {
        if (app_exit_requested) return 1;
        app_set_running(APP_MODE_STOP);
        ethercat_reset_continue_tx_buffers();
        gMotorParam[3].run_mode = RM_CSP;raw_fault.store(false);

        JC_LOG_WARN("Initializing J3 in disabled state before guarded enable.");
        m18_driver_init();

        uint16_t status_word = 0;
        int8_t operation_mode = 0;
        int32_t position_count = 0;
        int ret = read_j3_state(&status_word, &operation_mode, &position_count);
        if (app_exit_requested || ret != 0) {
            JC_LOG_ERROR("J3 pre-enable state read failed, ret=%d", ret);
            return 1;
        }
        if (status_word & 0x0008) {
            JC_LOG_ERROR("J3 cannot be enabled from statusword=0x%04X (%s)",
                         status_word, m18_ds402_state_name(status_word));
            return 1;
        }

        ret = m18_write_sdo(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID,
                             0x6060, 0, MRUN_CSP, SDO_WRITE_1B);
        if (ret != 0) {
            JC_LOG_ERROR("J3 CSP mode write failed, ret=%d", ret);
            return 1;
        }

        uint32_t requested_mode_value = 0;
        uint8_t requested_mode_size = 0;
        ret = m18_read_sdo(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID,
                           0x6060, 0, &requested_mode_value, &requested_mode_size);
        if (ret != 0 || requested_mode_size != 1 ||
            static_cast<int8_t>(requested_mode_value & 0xFF) != MRUN_CSP) {
            JC_LOG_ERROR("J3 CSP mode request verification failed: value=%d size=%u ret=%d",
                         static_cast<int8_t>(requested_mode_value & 0xFF), requested_mode_size, ret);
            return 1;
        }

        ret = m18_write_sdo(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID,
                             0x607A, 0, static_cast<uint32_t>(position_count), SDO_WRITE_4B);
        if (ret != 0) {
            JC_LOG_ERROR("J3 hold target write failed, ret=%d", ret);
            return 1;
        }

        uint32_t target_value = 0;
        uint8_t target_size = 0;
        ret = m18_read_sdo(M18_SLAVE_ID, J03_CFG_PASSGE, J03_NODE_ID,
                           0x607A, 0, &target_value, &target_size);
        if (ret != 0 || target_size != 4 || static_cast<int32_t>(target_value) != position_count) {
            JC_LOG_ERROR("J3 hold target verification failed: actual=%d readback=%d size=%u ret=%d",
                         position_count, static_cast<int32_t>(target_value), target_size, ret);
            return 1;
        }

        const float pre_enable_position_mm = UNIT_2_MM(position_count);
        gMotorParam[3].run_mode = RM_CSP;
        app_set_motor_target(3, 0, pre_enable_position_mm, 0);

        // This M18 activates CSP only when synchronous targets are present
        // throughout the DS402 enable transition. The target was verified
        // above and remains fixed at the measured current position.
        app_set_running(APP_MODE_POS);
        usleep(10 * 1000);
        if (!app_exit_requested) m18_enable_driver(M18_SLAVE_ID, J03_TX_PASSGE, J03_NODE_ID);

        // The mode display can lag the controlword transition. Keep sending
        // the verified hold target for up to two seconds while monitoring the
        // DS402 state and actual position every 100 ms.
        bool csp_active = false;
        float post_enable_position_mm = pre_enable_position_mm;
        for (int check = 0; check < 20; check++) {
            usleep(100 * 1000);
            ret = read_j3_state(&status_word, &operation_mode, &position_count);
            post_enable_position_mm = UNIT_2_MM(position_count);
            if (app_exit_requested || ret != 0 ||
                std::fabs(post_enable_position_mm - pre_enable_position_mm) > J03_ENABLE_MAX_SHIFT_MM) {
                JC_LOG_ERROR("J3 moved while waiting for CSP: check=%d ret=%d shift=%.5f mm",
                             check + 1, ret, post_enable_position_mm - pre_enable_position_mm);
                app_set_running(APP_MODE_STOP);
                ethercat_reset_continue_tx_buffers();
                m18_disable_driver(M18_SLAVE_ID, J03_TX_PASSGE, J03_NODE_ID);
                return 1;
            }
            if ((status_word & 0x006F) == 0x0027 && operation_mode == MRUN_CSP) {
                csp_active = true;
                break;
            }
        }

        if (!csp_active) {
            app_set_running(APP_MODE_STOP);
            ethercat_reset_continue_tx_buffers();
            JC_LOG_ERROR("J3 guarded enable failed: ret=%d status=0x%04X mode=%d shift=%.5f mm",
                         ret, status_word, operation_mode,
                         post_enable_position_mm - pre_enable_position_mm);
            m18_disable_driver(M18_SLAVE_ID, J03_TX_PASSGE, J03_NODE_ID);
            return 1;
        }

        JC_LOG_WARN("J3 guarded enable complete: hold=%.5f mm shift=%.5f mm",
                    pre_enable_position_mm, post_enable_position_mm - pre_enable_position_mm);
    } else {
        JC_LOG_ERROR("Unknown CMD");
        return 1;
    }

    return 0;
}
