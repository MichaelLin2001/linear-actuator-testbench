#include "m18.h"
#include "transmit.h"
#include <unistd.h>
#include "jc_log.h"
extern "C" {
#include "utils.h"
}

void m18_enable(uint8_t slave_id, uint8_t passage, uint32_t node_id)
{
    m18_start_node(slave_id, passage, node_id);
    usleep(1000 * 100);
    m18_enable_driver(slave_id, passage, node_id);
}

static inline int send_nmt_message(uint8_t slave_id, uint8_t passage, uint32_t node_id, NMT_CMD cmd)
{
    EtherCAT_Msg tx_message = {0}, rx_message = {0};
    int ret = 0, retry = 3;

    canopen_build_nmt_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, cmd);
resend:
    ret = ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    if (ret < 0) {
        JC_LOG_ERROR("send message fail, ret = %d.", ret);
        return 0;
    }

#if 1
    return 0;
#else
    if (rx_message.motor[passage_resp].dlc == 2 && rx_message.motor[passage_resp].id == id_resp &&
        (rx_message.motor[passage_resp].data[0] == NMT_CMD_ENTER_PRE_OPERATIONAL && rx_message.motor[passage_resp].data[1] == node_id)) {
        return 0;
    }

    if (retry > 0) {
        retry--;
        goto resend;
    }

    JC_LOG_ERROR("response validation failed...");
    JC_LOG_DEBUG("dlc=%d, passage_resp=%d, id=%d.", rx_message.motor[passage_resp].dlc, passage_resp, rx_message.motor[passage_resp].id);
    dump_buffer(rx_message.motor[passage_resp].data, rx_message.motor[passage_resp].dlc);

    return 0;
#endif
}

/**
 * Important: driver cannot transition directly from Stop to Pre-op state. A Reset command is required.
 */
int m18_enter_pre_op(uint8_t slave_id, uint8_t passage, uint32_t node_id)
{
    int ret = send_nmt_message(slave_id, passage, node_id, NMT_CMD_ENTER_PRE_OPERATIONAL);

    if (ret == 0) {
        JC_LOG_INFO("Set Motor Enter Pre-OP Sucessfully.");
    } else {
        JC_LOG_ERROR("Set Motor Enter Pre-OP Failed.");
    }

    return 0;
}

int m18_start_node(uint8_t slave_id, uint8_t passage, uint32_t node_id)
{
    int ret = send_nmt_message(slave_id, passage, node_id, NMT_CMD_START_REMOTE_NODE);

    if (ret == 0) {
        JC_LOG_INFO("Start Node Sucessfully.");
    } else {
        JC_LOG_ERROR("Start Node Failed.");
    }

    return 0;
}

int m18_stop_node(uint8_t slave_id, uint8_t passage, uint32_t node_id)
{
    int ret = send_nmt_message(slave_id, passage, node_id, NMT_CMD_STOP_REMOTE_NODE);

    if (ret == 0) {
        JC_LOG_INFO("Stop Node Sucessfully.");
    } else {
        JC_LOG_ERROR("Stop Node Failed.");
    }

    return 0;
}

int m18_reset_node(uint8_t slave_id, uint8_t passage, uint32_t node_id)
{
    int ret = send_nmt_message(slave_id, passage, node_id, NMT_CMD_RESET_NODE);

    if (ret == 0) {
        JC_LOG_ERROR("Reset Node Sucessfully.");
    } else {
        JC_LOG_ERROR("Reset Node Failed.");
    }

    return 0;
}

int m18_reset_comm(uint8_t slave_id, uint8_t passage, uint32_t node_id)
{
    int ret = send_nmt_message(slave_id, passage, node_id, NMT_CMD_RESET_COMMUNICATION);

    if (ret == 0) {
        JC_LOG_INFO("Reset Communication Sucessfully.");
    } else {
        JC_LOG_ERROR("Reset Communication Failed.");
    }

    return 0;
}

void m18_enable_driver(uint8_t slave_id, uint8_t passage, uint32_t node_id)
{
    EtherCAT_Msg tx_message = {0}, rx_message = {0};
    int ret = 0, retry = 3;

    canopen_build_ds402_state_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, DS402_STATE_READY_TO_SWITCH_ON);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    usleep(1000 * 200);
    canopen_build_ds402_state_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, DS402_STATE_SWITCHED_ON);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    usleep(1000 * 200);
    canopen_build_ds402_state_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, DS402_STATE_OPERATION_ENABLED);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    usleep(1000 * 200);

    JC_LOG_INFO("Enable Driver");
}

void m18_disable_driver(uint8_t slave_id, uint8_t passage, uint32_t node_id)
{
    EtherCAT_Msg tx_message = {0}, rx_message = {0};
    int ret = 0, retry = 3;

    canopen_build_ds402_state_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, DS402_STATE_SWITCH_ON_DISABLED);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);

    JC_LOG_INFO("Disable Driver");
}

void m18_set_position(uint8_t slave_id, uint8_t passage, uint32_t node_id, int32_t pos_count)
{
    EtherCAT_Msg tx_message = {0}, rx_message = {0};
    int ret = 0, retry = 3;

    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x607A, 0x00, pos_count, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
}

void m18_set_position_pp(uint8_t slave_id, uint8_t passage, uint32_t node_id, int32_t pos_count)
{
    EtherCAT_Msg tx_message = {0}, rx_message = {0};
    int ret = 0, retry = 3;

    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x607A, 0x00, pos_count, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    canopen_build_ds402_state_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, DS402_STATE_OPERATION_ENABLED);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    // trigger update position
    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x6040, 0x00, 0x1F, SDO_WRITE_2B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
}

void m18_set_mode(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint8_t mode)
{
    EtherCAT_Msg tx_message = {0}, rx_message = {0};
    int ret = 0, retry = 3;

    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x6060, 0x00, mode, SDO_WRITE_1B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
}

static const struct can_frame* m18_find_sdo_response(const EtherCAT_Msg& message,
                                                     uint32_t node_id,
                                                     uint16_t index,
                                                     uint8_t subindex,
                                                     bool read_response)
{
    const uint32_t response_id = CANOPEN_COBID_SDO_TX(node_id);
    for (int i = 0; i < ECAT_SLAVE_PER_MOTOR_COUNT; ++i) {
        const struct can_frame& frame = message.motor[i];
        if (frame.dlc == 8 && frame.id == response_id &&
            frame.data[1] == (index & 0xFF) &&
            frame.data[2] == ((index >> 8) & 0xFF) &&
            frame.data[3] == subindex) {
            const SDO_CMD command = static_cast<SDO_CMD>(frame.data[0]);
            if (command == SDO_ERR_FDB ||
                (read_response && SDO_IS_READ_RESP(command) && sdo_get_data_size(command) > 0) ||
                (!read_response && command == SDO_FDB)) {
                return &frame;
            }
        }
    }
    return nullptr;
}

static uint32_t m18_sdo_abort_code(const struct can_frame& frame)
{
    return static_cast<uint32_t>(frame.data[4]) |
           (static_cast<uint32_t>(frame.data[5]) << 8) |
           (static_cast<uint32_t>(frame.data[6]) << 16) |
           (static_cast<uint32_t>(frame.data[7]) << 24);
}

int m18_read_sdo(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint16_t index,
                 uint8_t subindex, uint32_t* value, uint8_t* value_size)
{
    if (passage == 0 || passage > ECAT_SLAVE_PER_MOTOR_COUNT ||
        node_id == 0 || node_id > 0x7F || value == nullptr || value_size == nullptr) {
        JC_LOG_ERROR("invalid M18 SDO read argument");
        return -1;
    }

    EtherCAT_Msg tx_message = {0};
    EtherCAT_Msg rx_message = {0};
    canopen_build_sdo_read_request(&tx_message.motor[passage - 1], node_id, index, subindex);
    const int ret = ethercat_send_canopen_sdo_request(slave_id, tx_message, &rx_message, 100);
    if (ret != 0) {
        JC_LOG_ERROR("M18 SDO read transport failed: ret=%d node=%u index=0x%04X sub=%u",
                     ret, node_id, index, subindex);
        return ret;
    }

    const struct can_frame* response = m18_find_sdo_response(rx_message, node_id, index, subindex, true);
    if (response == nullptr) {
        JC_LOG_ERROR("M18 SDO read response not found");
        return -3;
    }
    const SDO_CMD command = static_cast<SDO_CMD>(response->data[0]);
    if (command == SDO_ERR_FDB) {
        JC_LOG_ERROR("M18 SDO read abort: node=%u index=0x%04X sub=%u code=0x%08X",
                     node_id, index, subindex, m18_sdo_abort_code(*response));
        return -2;
    }
    const uint8_t size = sdo_get_data_size(command);
    if (!SDO_IS_READ_RESP(command) || size == 0) {
        JC_LOG_ERROR("invalid M18 SDO read response command=0x%02X", command);
        return -4;
    }

    uint32_t result = 0;
    for (uint8_t i = 0; i < size; ++i) {
        result |= static_cast<uint32_t>(response->data[4 + i]) << (8 * i);
    }
    *value = result;
    *value_size = size;
    return 0;
}

int m18_write_sdo(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint16_t index,
                  uint8_t subindex, uint32_t value, SDO_CMD command)
{
    if (passage == 0 || passage > ECAT_SLAVE_PER_MOTOR_COUNT ||
        node_id == 0 || node_id > 0x7F || !SDO_IS_WRITE_CMD(command) ||
        sdo_get_data_size(command) == 0) {
        JC_LOG_ERROR("invalid M18 SDO write argument");
        return -1;
    }

    EtherCAT_Msg tx_message = {0};
    EtherCAT_Msg rx_message = {0};
    canopen_build_sdo_write_request(&tx_message.motor[passage - 1], node_id,
                                    index, subindex, static_cast<int32_t>(value), command);
    const int ret = ethercat_send_canopen_sdo_request(slave_id, tx_message, &rx_message, 100);
    if (ret != 0) {
        JC_LOG_ERROR("M18 SDO write transport failed: ret=%d node=%u index=0x%04X sub=%u",
                     ret, node_id, index, subindex);
        return ret;
    }

    const struct can_frame* response = m18_find_sdo_response(rx_message, node_id, index, subindex, false);
    if (response == nullptr) {
        JC_LOG_ERROR("M18 SDO write response not found");
        return -3;
    }
    if (response->data[0] == SDO_ERR_FDB) {
        JC_LOG_ERROR("M18 SDO write abort: node=%u index=0x%04X sub=%u code=0x%08X",
                     node_id, index, subindex, m18_sdo_abort_code(*response));
        return -2;
    }
    if (response->data[0] != SDO_FDB) {
        JC_LOG_ERROR("invalid M18 SDO write response command=0x%02X", response->data[0]);
        return -4;
    }
    return 0;
}

void m18_set_PP_plan_param(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint32_t speed, uint32_t acce, uint32_t dece, uint32_t speed_max, uint32_t acce_dece_max)
{
    EtherCAT_Msg tx_message = {0};

    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x6081, 0x00, speed, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    usleep(1000 * 200);
    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x6083, 0x00, acce, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    usleep(1000 * 200);
    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x6084, 0x00, dece, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    usleep(1000 * 200);
    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x607F, 0x00, speed_max, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    usleep(1000 * 200);
    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x60C5, 0x00, acce_dece_max, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    usleep(1000 * 200);

    // m18_set_elec_gear_ration(slave_id,passage, node_id, (1<<17), 240);
}

void m18_set_elec_gear_ration(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint32_t Numerator, uint32_t denominator)
{
    EtherCAT_Msg tx_message = {0};

    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x6093, 0x01, Numerator, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);

    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, 0x6093, 0x02, denominator, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
}

void m18_trigger_sync(uint8_t slave_id, uint8_t passage)
{
    EtherCAT_Msg tx_message = {0};

    canopen_build_sync((struct can_frame*)&tx_message.motor[passage - 1], SYNC_TYPE_BASIC);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
}

/**
 * ATTENTION: PDO data construction is highly dependent on PDO map
 */
static void m18_bulid_csp_rpdo_data(struct can_frame* frame, uint32_t node_id, int32_t pos_count)
{
    frame->id = 0x200 + node_id;
    frame->rtr = 0;
    frame->dlc = 6;
    frame->data[0] = pos_count & 0xFF;
    frame->data[1] = (pos_count >> 8) & 0xFF;
    frame->data[2] = (pos_count >> 16) & 0xFF;
    frame->data[3] = (pos_count >> 24) & 0xFF;
}

/**
 * ATTENTION: PDO data construction is highly dependent on PDO map
 */
static void m18_bulid_cst_rpdo_data(struct can_frame* frame, uint32_t node_id, int16_t tor)
{
    frame->id = 0x200 + node_id;
    frame->rtr = 0;
    frame->dlc = 6;
    frame->data[4] = tor & 0xFF;
    frame->data[5] = (tor >> 8) & 0xFF;
}

void m18_build_csp_message(struct can_frame* frame, uint32_t node_id, int32_t pos_count)
{
    m18_bulid_csp_rpdo_data(frame, node_id, pos_count);
}

void m18_build_cst_message(struct can_frame* frame, uint32_t node_id, int16_t tor)
{
    m18_bulid_cst_rpdo_data(frame, node_id, tor);
}

/**
 * @note TPDO2 not supported EventTimer Trigger
 */
void m18_set_pdo_map(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint8_t pdo_num, uint32_t* obj_list, uint8_t obj_count)
{
    EtherCAT_Msg tx_message = {0};

    uint16_t comm_index = 0;
    uint16_t map_index = 0;
    uint32_t data = CANOPEN_COBID_PDO(pdo_num, node_id);

    uint8_t is_tpdo_num = pdo_num % 2;

    if (pdo_num < CANOPEN_FC_TPDO1 || pdo_num > CANOPEN_FC_RPDO4) {
        JC_LOG_ERROR("PDO type is invalid, pdo_num=%d", pdo_num);
        return;
    }

    if (is_tpdo_num == 0) {
        comm_index = CANOPEN_RPDO_COMM_BASE + (pdo_num - CANOPEN_FC_RPDO1) / 2;
        map_index = CANOPEN_RPDO_MAP_BASE + (pdo_num - CANOPEN_FC_RPDO1) / 2;
    } else {
        comm_index = CANOPEN_TPDO_COMM_BASE + (pdo_num - CANOPEN_FC_TPDO1) / 2;
        map_index = CANOPEN_TPDO_MAP_BASE + (pdo_num - CANOPEN_FC_TPDO1) / 2;
    }

    // disable RPDO_n
    data |= 1UL << 31;
    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, comm_index, 0x01, data, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);

    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, comm_index, 0x02, 0x01, SDO_WRITE_1B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);

    // set PDO
    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, map_index, 0x00, 0x00, SDO_WRITE_1B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);

    for (uint8_t i = 0; i < obj_count; i++) {
        uint32_t pdo_obj = obj_list[i];
        canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, map_index, i + 1, pdo_obj, SDO_WRITE_4B);
        ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
    }

    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, map_index, 0x00, obj_count, SDO_WRITE_1B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);

    data &= ~(1UL << 31);
    canopen_build_sdo_write_request((struct can_frame*)&tx_message.motor[passage - 1], node_id, comm_index, 0x01, data, SDO_WRITE_4B);
    ethercat_send_message_queue(slave_id, tx_message, NULL, 100);
}
