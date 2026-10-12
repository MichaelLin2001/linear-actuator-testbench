extern "C" {
#include "ethercat.h"
#include "utils.h"
}
#include "transmit.h"
#include "queue.h"
#include "jc_log.h"
#include "ethercat_msg_pool.hpp"
#include "ethercat_canfd_msg_pool.hpp"
#include "ethercat_config.h"
#include <sys/time.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>

spsc_queue<EtherCAT_Msg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>> tx_message_queue[ECAT_SLAVE_COUNT_MAX];
spsc_queue<EtherCAT_Msg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>> rx_message_queue[ECAT_SLAVE_COUNT_MAX];
EtherCAT_Msg tx_message_continued[ECAT_SLAVE_COUNT_MAX];
EtherCAT_Msg rx_message_query[ECAT_SLAVE_COUNT_MAX];

spsc_queue<EtherCAT_CanfdMsg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>> canfd_tx_message_queue[ECAT_SLAVE_COUNT_MAX];
spsc_queue<EtherCAT_CanfdMsg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>> canfd_rx_message_queue[ECAT_SLAVE_COUNT_MAX];

EtherCAT_CanfdPdo canfd_tx_pdo_cache[ECAT_SLAVE_COUNT_MAX] = {};
EtherCAT_CanfdPdo canfd_rx_pdo_cache[ECAT_SLAVE_COUNT_MAX] = {};

uint16_t canfd_tx_pdo_index[ECAT_SLAVE_COUNT_MAX][ECAT_CANFD_CHANNELS] = {{0}};
uint16_t canfd_rx_pdo_index_last[ECAT_SLAVE_COUNT_MAX][ECAT_CANFD_CHANNELS] = {{0}};

EtherCAT_CanfdMsg canfd_rx_message_query[ECAT_SLAVE_COUNT_MAX][ECAT_CANFD_CHANNELS];
bool canfd_rx_message_query_valid[ECAT_SLAVE_COUNT_MAX][ECAT_CANFD_CHANNELS] = {{false}};

std::thread ethercatPDOThread;
std::atomic<bool> g_ethercat_thread_running(false);

std::mutex response_mutex;
std::condition_variable response_cv;

std::mutex sdo_response_mutex;
std::condition_variable sdo_response_cv;
EtherCAT_Msg sdo_response_query[ECAT_SLAVE_COUNT_MAX] = {};
uint64_t sdo_response_sequence[ECAT_SLAVE_COUNT_MAX] = {};

char IOmap[8192];
OSAL_THREAD_HANDLE checkThread;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

static bool g_continue_mode_en = false;
void ethercat_notify_response(void)
{
    for (int i = 0; i < ECAT_SLAVE_COUNT_MAX; i++) {
        /**
         * NOTE: There should be only one queue waiting at any given time
         */
        response_cv.notify_one();
    }
}

void ethercat_enable_continue(bool enable)
{
    g_continue_mode_en = enable;
}

void ethercat_set_query_response(uint8_t slaveId, EtherCAT_Msg rx_message)
{
    memcpy(&rx_message_query[slaveId], &rx_message, sizeof(EtherCAT_Msg));

    bool has_sdo_response = false;
    for (int i = 0; i < ECAT_SLAVE_PER_MOTOR_COUNT; i++) {
        const struct can_frame& frame = rx_message.motor[i];
        if (frame.dlc >= 4 && frame.id >= 0x580u && frame.id <= 0x5FFu) {
            has_sdo_response = true;
            break;
        }
    }

    if (has_sdo_response) {
        {
            std::lock_guard<std::mutex> lock(sdo_response_mutex);
            memcpy(&sdo_response_query[slaveId], &rx_message, sizeof(EtherCAT_Msg));
            sdo_response_sequence[slaveId]++;
        }
        sdo_response_cv.notify_all();
    }
}

void ethercat_set_canfd_query_response(uint8_t slaveId, EtherCAT_CanfdMsg rx_message)
{
    memcpy(&canfd_rx_message_query[slaveId][rx_message.channel], &rx_message, sizeof(EtherCAT_CanfdMsg));
    canfd_rx_message_query_valid[slaveId][rx_message.channel] = true;
}

void ethercat_push_rxmessage_queue(spsc_queue<EtherCAT_Msg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>>& queue, const EtherCAT_Msg_ptr& msg)
{
    if (queue.write_available()) {
        queue.push(msg);
    } else {
        do {
            JC_LOG_WARN("Queue Fulled, Waiting For RX_Message Handling");
            sleep(1);
        } while (!queue.push(msg));
    }
}

void ethercat_push_txmessage_queue(spsc_queue<EtherCAT_Msg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>>& queue, const EtherCAT_Msg_ptr& msg)
{
    if (queue.write_available()) {
        queue.push(msg);
    } else {
        do {
            JC_LOG_WARN("Queue Fulled, Waiting For TX_Message Handling");
            sleep(1);
        } while (!queue.push(msg));
    }
}

void ethercat_push_canfd_txmessage_queue(spsc_queue<EtherCAT_CanfdMsg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>>& queue, const EtherCAT_CanfdMsg_ptr& msg)
{
    if (queue.write_available()) {
        queue.push(msg);
    } else {
        do {
            JC_LOG_WARN("Queue Fulled, Waiting For TX_Message Handling");
            sleep(1);
        } while (!queue.push(msg));
    }
}

void ethercat_push_canfd_rxmessage_queue(spsc_queue<EtherCAT_CanfdMsg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>>& queue, const EtherCAT_CanfdMsg_ptr& msg)
{
    if (queue.write_available()) {
        queue.push(msg);
    } else {
        do {
            JC_LOG_WARN("Queue Fulled, Waiting For RX_Message Handling");
            sleep(1);
        } while (!queue.push(msg));
    }
}

static inline bool is_canfd_pdo_layout(int slave_index)
{
    return (ec_slave[slave_index].Obytes >= (int)sizeof(EtherCAT_CanfdPdo)) && (ec_slave[slave_index].Ibytes >= (int)sizeof(EtherCAT_CanfdPdo));
}

static int ethercat_loop_processdata(void)
{
    int _wkc = 0;
    EtherCAT_Msg_ptr _tx_msg = {0};
    uint8_t msg_len = sizeof(EtherCAT_Msg);

    for (int index = 0; index < ec_slavecount; index++) {
        int slave_index = index + 1;
        if (ec_slave[slave_index].outputs == NULL) {
            continue;
        }

        if (is_canfd_pdo_layout(slave_index)) {
            // ========================
            // CANFD PDO mode (ESI-fixed)
            // ========================
            if (canfd_tx_message_queue[index].read_available()) {
                // Same as legacy CAN: consume one frame per cycle for FIFO order
                EtherCAT_CanfdMsg_ptr _tx_msg;
                if (canfd_tx_message_queue[index].pop(_tx_msg) && _tx_msg) {
                    if (_tx_msg->channel < ECAT_CANFD_CHANNELS) {
                        struct canfd_frame& _slot = canfd_tx_pdo_cache[index].channel[_tx_msg->channel];
                        _slot.index = ++canfd_tx_pdo_index[index][_tx_msg->channel];
                        _slot.id = _tx_msg->id;
                        _slot.len = (uint16_t)((_tx_msg->len > ECAT_CANFD_DATA_BYTES) ? ECAT_CANFD_DATA_BYTES : _tx_msg->len);
                        memset(_slot.data, 0, sizeof(_slot.data));
                        if (_slot.len > 0) {
                            memcpy(_slot.data, _tx_msg->data, _slot.len);
                        }
                        memset(_slot.reserved_30, 0, sizeof(_slot.reserved_30));
                        _slot.reserved_4 = 0;
                    }
                }
            }
            // continued mode (use last sent frame)
            memcpy(ec_slave[slave_index].outputs, &canfd_tx_pdo_cache[index], sizeof(EtherCAT_CanfdPdo));
        } else {
            // ========================
            // legacy 8-byte CAN frame mode
            // ========================
            if (tx_message_queue[index].read_available()) {
                // queue mode
                if (tx_message_queue[index].pop(_tx_msg)) {
                    memcpy(ec_slave[slave_index].outputs, _tx_msg.get(), msg_len);
                }
            } else {
                // continued mode
                memcpy(ec_slave[slave_index].outputs, &tx_message_continued[index], msg_len);
            }
        }
    }

    ec_send_processdata();
    _wkc = ec_receive_processdata(EC_TIMEOUTRET);

    if (_wkc < expectedWKC) {
        JC_LOG_ERROR("[EtherCAT Error] Dropped packet (Bad WKC!)");
        wkc_err_count++;
    } else {
        for (int index = 0; index < ec_slavecount; index++) {
            int slave_index = index + 1;
            if (ec_slave[slave_index].inputs == NULL) {
                continue;
            }

            if (is_canfd_pdo_layout(slave_index)) {
                const EtherCAT_CanfdPdo* _pdo_in = (const EtherCAT_CanfdPdo*)ec_slave[slave_index].inputs;
                memcpy(&canfd_rx_pdo_cache[index], _pdo_in, sizeof(EtherCAT_CanfdPdo));

                for (uint8_t ch = 0; ch < ECAT_CANFD_CHANNELS; ch++) {
                    const struct canfd_frame& _slot = _pdo_in->channel[ch];
                    const uint16_t _idx = _slot.index;
                    if (_idx == canfd_rx_pdo_index_last[index][ch]) {
                        continue;
                    }
                    canfd_rx_pdo_index_last[index][ch] = _idx;

                    EtherCAT_CanfdMsg _rx_msg{};
                    _rx_msg.channel = ch;
                    _rx_msg.id = _slot.id;
                    _rx_msg.len = (uint16_t)((_slot.len > ECAT_CANFD_DATA_BYTES) ? ECAT_CANFD_DATA_BYTES : _slot.len);
                    memset(_rx_msg.data, 0, sizeof(_rx_msg.data));
                    if (_rx_msg.len > 0) {
                        memcpy(_rx_msg.data, _slot.data, _rx_msg.len);
                    }

                    // Filter out CANopen NMT Heartbeat (0x700+node_id) to avoid filling rx queue
                    const bool is_heartbeat = (_rx_msg.id >= 0x700u && _rx_msg.id <= 0x7FFu);
                    if (!is_heartbeat) {
                        EtherCAT_CanfdMsg_ptr rx_ptr = EtherCAT_CanfdMsgPool::instance().acquire();
                        *rx_ptr = _rx_msg;
                        ethercat_push_canfd_rxmessage_queue(canfd_rx_message_queue[index], rx_ptr);
                    }

                    // Provide responses as query responses for synchronous requests.
                    // - SDO response: 0x580..0x5FF
                    // - SeenPin motor status response: 0x180..0x1FF
                    if ((_rx_msg.id >= 0x580u && _rx_msg.id <= 0x5FFu) || (_rx_msg.id >= 0x180u && _rx_msg.id <= 0x1FFu)) {
                        ethercat_set_canfd_query_response((uint8_t)index, _rx_msg);
                        ethercat_notify_response();
                    }
                }
            } else {
                EtherCAT_Msg* _rx_msg = (EtherCAT_Msg*)ec_slave[slave_index].inputs;
                for (int i = 0; i < ECAT_SLAVE_PER_MOTOR_COUNT; i++) {
                    if (_rx_msg->motor[i].dlc == 0) {
                        continue;
                    }

                    EtherCAT_Msg_ptr rx_ptr = EtherCAT_MsgPool::instance().acquire();
                    *rx_ptr = *_rx_msg;
                    ethercat_push_rxmessage_queue(rx_message_queue[index], rx_ptr);
                    break;
                }
            }
        }
    }

    wkc_err_iteration_count++;
    return _wkc;
}

static void degraded_handler()
{
    JC_LOG_ERROR("[EtherCAT Error] Logging error...");
    time_t current_time = time(NULL);
    char* time_str = ctime(&current_time);
    JC_LOG_ERROR("ESTOP. EtherCAT became degraded at %s.", time_str);
    JC_LOG_ERROR("[EtherCAT Error] Stopping RT process.");
}

static int run_ethercat(const char* ifname)
{
    int i;
    int oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname)) {
        JC_LOG_INFO("[EtherCAT Init] Initialization on device %s succeeded.", ifname);
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0) {
            JC_LOG_INFO("[EtherCAT Init] %d slaves found and configured.", ec_slavecount);
            if (ec_slavecount < ECAT_SLAVE_COUNT_MAX) {
                JC_LOG_WARN("[RT EtherCAT] Warning: Expected %d slaves, found %d.", ECAT_SLAVE_COUNT_MAX, ec_slavecount);
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++) {
                ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;
            }

            ec_config_map(&IOmap);
            ec_configdc();

            JC_LOG_INFO("[EtherCAT Init] Mapped slaves.");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * ECAT_SLAVE_COUNT_MAX);

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++) {
                JC_LOG_INFO("[SLAVE %d]", slave_idx);
                JC_LOG_INFO("  IN  %d bytes, %d bits", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                JC_LOG_INFO("  OUT %d bytes, %d bits", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
            }

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) {
                oloop = 1;
            }

            if (oloop > 8) {
                oloop = 8;
            }

            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) {
                iloop = 1;
            }

            if (iloop > 8) {
                iloop = 8;
            }

            JC_LOG_INFO("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            JC_LOG_INFO("[EtherCAT Init] Requesting operational state for all slaves...\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            JC_LOG_INFO("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                JC_LOG_INFO("[EtherCAT Init] Operational state reached for all slaves.\n");
                inOP = TRUE;
                return 1;
            } else {
                JC_LOG_ERROR("[EtherCAT Error] Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        JC_LOG_ERROR("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                                     ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        } else {
            JC_LOG_ERROR("[EtherCAT Error] No slaves found!\n");
        }
    } else {
        JC_LOG_ERROR("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n", ifname);
    }
    return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;

static OSAL_THREAD_FUNC ecatcheck(void* ptr)
{
    (void)ptr;
    int slave = 0;
    while (1) {
        // count errors
        if (err_iteration_count > ECAT_ERR_PERIOD) {
            err_iteration_count = 0;
            err_count = 0;
        }

        if (err_count > ECAT_ERR_MAX) {
            // possibly shut down
            JC_LOG_ERROR("[EtherCAT Error] EtherCAT connection degraded.\n");
            JC_LOG_ERROR("[Simulink-Linux] Shutting down....\n");
            degraded_handler();
            break;
        }
        err_iteration_count++;

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = FALSE;
                JC_LOG_ERROR("...");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        JC_LOG_ERROR("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        JC_LOG_ERROR("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                    } else if (ec_slave[slave].state > 0) {
                        if (ec_reconfig_slave(slave, ECAT_TIMEOUTMON_US)) {
                            ec_slave[slave].islost = FALSE;
                            JC_LOG_INFO("[EtherCAT Status] Slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state) {
                            ec_slave[slave].islost = TRUE;
                            JC_LOG_ERROR("[EtherCAT Error] Slave %d lost\n", slave);
                            err_count++;
                        }
                    }
                }

                if (ec_slave[slave].islost) {
                    if (!ec_slave[slave].state) {
                        if (ec_recover_slave(slave, ECAT_TIMEOUTMON_US)) {
                            ec_slave[slave].islost = FALSE;
                            JC_LOG_INFO("[EtherCAT Status] Slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE;
                        JC_LOG_INFO("[EtherCAT Status] Slave %d found\n", slave);
                    }
                }
            }

            if (!ec_group[currentgroup].docheckstate) {
                JC_LOG_INFO("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
            }
        }
        osal_usleep(50000);
    }
}

void ethercat_reset_continue_tx_buffers(void)
{
    memset(tx_message_continued, 0, sizeof(tx_message_continued));
    memset(canfd_tx_pdo_cache, 0, sizeof(canfd_tx_pdo_cache));
    memset(canfd_tx_pdo_index, 0, sizeof(canfd_tx_pdo_index));
    JC_LOG_INFO("[EtherCAT] Continue TX buffers reset.");
}

void ethercat_init(const char* ifname)
{
    int i;
    int rc;
    JC_LOG_INFO("[EtherCAT] Initializing EtherCAT\n");
    osal_thread_create((void*)&checkThread, 128000, (void*)&ecatcheck, (void*)&ctime);
    for (i = 1; i < 10; i++) {
        JC_LOG_WARN("[EtherCAT] Attempting to start EtherCAT, try %d of 10.", i);
        rc = run_ethercat(ifname);
        if (rc) {
            break;
        }
        osal_usleep(1000000);
    }

    if (rc) {
        JC_LOG_INFO("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);

    } else {
        JC_LOG_ERROR("[EtherCAT Error] Failed to initialize EtherCAT after 100 tries. \n");
    }
}

void ethercat_run()
{
    if (wkc_err_iteration_count > ECAT_ERR_PERIOD) {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }

    if (wkc_err_count > ECAT_ERR_MAX) {
        JC_LOG_ERROR("[EtherCAT Error] Error count too high!");
        degraded_handler();
    }

    wkc = ethercat_loop_processdata();
}

int ethercat_send_message_queue(int slaveId, const EtherCAT_Msg tx_message, EtherCAT_Msg* rx_message, int timeout_ms)
{
    int retry = 3, ret = 0;
    uint8_t msg_len = sizeof(EtherCAT_Msg);

    EtherCAT_Msg_ptr msg(new EtherCAT_Msg);
    memcpy(msg.get(), &tx_message, msg_len);

    if (slaveId >= ec_slavecount) {
        JC_LOG_ERROR("slaveId(%d) is outof ec_slavecount(%d), transmit data failed", slaveId, ec_slavecount);
        return -1;
    }

re_send:
    ethercat_push_txmessage_queue(tx_message_queue[slaveId], msg);

    if (rx_message == NULL) {
        return 0;
    }

    std::unique_lock<std::mutex> lock(response_mutex);
    auto wait_result = response_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    if (wait_result == std::cv_status::no_timeout) {
        for (int i = 0; i < ECAT_SLAVE_PER_MOTOR_COUNT; i++) {
            if (rx_message_query[slaveId].motor[i].dlc != 0) {
                memcpy(rx_message, &rx_message_query[slaveId], msg_len);
                memset(&rx_message_query[slaveId], 0, msg_len);
                goto out;
            }
        }
    } else {
        JC_LOG_ERROR("wait slave-%d response time-out..", slaveId);
        ret = -110;
    }

    if (retry--) {
        JC_LOG_ERROR("retry %d...", retry);
        goto re_send;
    }

out:
    return ret;
}

static const struct can_frame* find_canopen_sdo_request(const EtherCAT_Msg& message)
{
    for (int i = 0; i < ECAT_SLAVE_PER_MOTOR_COUNT; i++) {
        const struct can_frame& frame = message.motor[i];
        if (frame.dlc == 8 && frame.id >= 0x600u && frame.id <= 0x67Fu) {
            const uint8_t command = frame.data[0];
            if (command == 0x40u || (command & 0xE0u) == 0x20u) {
                return &frame;
            }
        }
    }
    return nullptr;
}

static bool has_matching_canopen_sdo_response(const EtherCAT_Msg& message, const struct can_frame& request)
{
    const uint32_t expected_response_id = request.id - 0x80u;
    const bool is_read_request = request.data[0] == 0x40u;

    for (int i = 0; i < ECAT_SLAVE_PER_MOTOR_COUNT; i++) {
        const struct can_frame& frame = message.motor[i];
        if (frame.dlc != 8 || frame.id != expected_response_id) {
            continue;
        }
        if (frame.data[1] != request.data[1] || frame.data[2] != request.data[2] || frame.data[3] != request.data[3]) {
            continue;
        }

        const uint8_t command = frame.data[0];
        if (command == 0x80u) {
            return true;
        }
        if (is_read_request && (command & 0xE0u) == 0x40u) {
            return true;
        }
        if (!is_read_request && command == 0x60u) {
            return true;
        }
    }

    return false;
}

int ethercat_send_canopen_sdo_request(int slaveId, const EtherCAT_Msg tx_message, EtherCAT_Msg* rx_message, int timeout_ms)
{
    if (slaveId < 0 || slaveId >= ec_slavecount) {
        JC_LOG_ERROR("slaveId(%d) is outof ec_slavecount(%d), transmit data failed", slaveId, ec_slavecount);
        return -1;
    }
    if (rx_message == nullptr || timeout_ms <= 0) {
        JC_LOG_ERROR("invalid SDO response buffer or timeout");
        return -2;
    }

    const struct can_frame* request = find_canopen_sdo_request(tx_message);
    if (request == nullptr) {
        JC_LOG_ERROR("CANopen SDO request not found in EtherCAT message");
        return -3;
    }

    EtherCAT_Msg_ptr message(new EtherCAT_Msg);
    memcpy(message.get(), &tx_message, sizeof(EtherCAT_Msg));

    constexpr int max_attempts = 4;
    for (int attempt = 0; attempt < max_attempts; attempt++) {
        uint64_t start_sequence;
        {
            std::lock_guard<std::mutex> lock(sdo_response_mutex);
            start_sequence = sdo_response_sequence[slaveId];
        }

        ethercat_push_txmessage_queue(tx_message_queue[slaveId], message);

        std::unique_lock<std::mutex> lock(sdo_response_mutex);
        const bool received = sdo_response_cv.wait_for(
            lock,
            std::chrono::milliseconds(timeout_ms),
            [&]() {
                return sdo_response_sequence[slaveId] != start_sequence &&
                       has_matching_canopen_sdo_response(sdo_response_query[slaveId], *request);
            });

        if (received) {
            memcpy(rx_message, &sdo_response_query[slaveId], sizeof(EtherCAT_Msg));
            return 0;
        }

        JC_LOG_ERROR("wait slave-%d matching SDO response time-out (attempt %d/%d)", slaveId, attempt + 1, max_attempts);
    }

    return -110;
}

int ethercat_send_message_continue(int slaveId, const EtherCAT_Msg tx_message)
{
    int ec_slave_index = slaveId + 1;

    if (ec_slave_index > ec_slavecount) {
        JC_LOG_ERROR("ec_slave_index(%d) is outof ec_slavecount(%d), transmit data failed", ec_slave_index, ec_slavecount);
        return -1;
    }

    if (!ec_slave[ec_slave_index].outputs) {
        JC_LOG_ERROR("ec_slave[%d].outputs is NULL, transmit data failed.\n", ec_slave_index);
        return -2;
    }

    memcpy(&(tx_message_continued[slaveId]), &tx_message, sizeof(EtherCAT_Msg));
    return 0;
}

int ethercat_send_canfd_message_queue(int slaveId, const EtherCAT_CanfdMsg tx_message, EtherCAT_CanfdMsg* rx_message, int timeout_ms)
{
    int retry = 3, ret = 0;
    uint8_t msg_len = sizeof(EtherCAT_CanfdMsg);

    EtherCAT_CanfdMsg_ptr msg(new EtherCAT_CanfdMsg);
    memcpy(msg.get(), &tx_message, msg_len);

    if (slaveId >= ec_slavecount) {
        JC_LOG_ERROR("slaveId(%d) is outof ec_slavecount(%d), transmit data failed", slaveId, ec_slavecount);
        return -1;
    }
    /* channel check: CANFD*/
    if (tx_message.channel >= ECAT_CANFD_CHANNELS) {
        JC_LOG_ERROR("tx_message.channel(%d) out of range [0..%d)", (int)tx_message.channel, ECAT_CANFD_CHANNELS);
        return -2;
    }

re_send_canfd:
    canfd_rx_message_query_valid[slaveId][tx_message.channel] = false;

    ethercat_push_canfd_txmessage_queue(canfd_tx_message_queue[slaveId], msg);

    if (rx_message == NULL) {
        return 0;
    }

    /* PDO layout check when waiting response */
    if (!is_canfd_pdo_layout(slaveId + 1)) {
        JC_LOG_ERROR("slave-%d does not match CANFD PDO layout (needs %zuB I/O)", slaveId, sizeof(EtherCAT_CanfdPdo));
        return -3;
    }

    std::unique_lock<std::mutex> lock(response_mutex);
    auto wait_result = response_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms));
    if (wait_result == std::cv_status::no_timeout) {
        if (canfd_rx_message_query_valid[slaveId][tx_message.channel]) {
            memcpy(rx_message, &canfd_rx_message_query[slaveId][tx_message.channel], msg_len);
            canfd_rx_message_query_valid[slaveId][tx_message.channel] = false;
            goto out_canfd;
        }
    } else {
        JC_LOG_ERROR("wait slave-%d response time-out..", slaveId);
        ret = -110;
    }

    if (retry--) {
        JC_LOG_ERROR("retry %d...", retry);
        goto re_send_canfd;
    }

out_canfd:
    return ret;
}

int ethercat_send_canfd_message_continue(int slaveId, const EtherCAT_CanfdMsg tx_message)
{
    int ec_slave_index = slaveId + 1;
    if (ec_slave_index > ec_slavecount) {
        JC_LOG_ERROR("ec_slave_index(%d) is outof ec_slavecount(%d), transmit data failed", ec_slave_index, ec_slavecount);
        return -1;
    }
    if (!ec_slave[ec_slave_index].outputs) {
        JC_LOG_ERROR("ec_slave[%d].outputs is NULL, transmit data failed.\n", ec_slave_index);
        return -2;
    }

    /* channel check: CANFD */
    if (tx_message.channel >= ECAT_CANFD_CHANNELS) {
        JC_LOG_ERROR("tx_message.channel(%d) out of range [0..%d)", (int)tx_message.channel, ECAT_CANFD_CHANNELS);
        return -4;
    }
    if (!is_canfd_pdo_layout(ec_slave_index)) {
        JC_LOG_ERROR("slave-%d does not match CANFD PDO layout (needs %zuB I/O)", slaveId, sizeof(EtherCAT_CanfdPdo));
        return -3;
    }

    uint8_t ch = tx_message.channel;
    struct canfd_frame& slot = canfd_tx_pdo_cache[slaveId].channel[ch];
    slot.index = ++canfd_tx_pdo_index[slaveId][ch];
    slot.id = tx_message.id;
    slot.len = (uint16_t)((tx_message.len > ECAT_CANFD_DATA_BYTES) ? ECAT_CANFD_DATA_BYTES : tx_message.len);
    memset(slot.data, 0, sizeof(slot.data));
    if (slot.len > 0) {
        memcpy(slot.data, tx_message.data, slot.len);
    }
    memset(slot.reserved_30, 0, sizeof(slot.reserved_30));
    slot.reserved_4 = 0;
    return 0;
}

__attribute__((weak)) void txMessageCallBack(void)
{
    // Weak function, can be overridden
}

__attribute__((weak)) void rxMessageCallBack(void)
{
    // Weak function, can be overridden
}

static void ethercatPDOThreadFunc(void)
{
    const long period_ns = ECAT_PDO_PERIOD_MS * 1000 * 1000;
    struct timespec next, now, actualPeriodTime, actualTime;
    struct timespec cycleTime = {0, period_ns};
    int ret;

    long overtime_ns;
    cycleTime.tv_sec = period_ns / 1000000000;
    cycleTime.tv_nsec = period_ns % 1000000000;

    clock_gettime(CLOCK_MONOTONIC, &next);

    g_ethercat_thread_running = true;
    while (g_ethercat_thread_running) {
        txMessageCallBack();

        ethercat_run();

        rxMessageCallBack();

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
}

void ethercat_thread_start(void)
{
    ethercatPDOThread = std::thread(ethercatPDOThreadFunc);
}

void ethercat_thread_stop(void)
{
    g_ethercat_thread_running = false;
    // wait for the thread to finish
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ethercatPDOThread.join();
}
