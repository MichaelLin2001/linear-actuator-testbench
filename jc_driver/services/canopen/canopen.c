#include "canopen.h"
#include "utils.h"

void canopen_build_sdo_write_request(struct can_frame* frame, uint8_t node_id, uint16_t index, uint8_t subindex, int32_t data, SDO_CMD sdo_cmd)
{
    frame->id = CANOPEN_COBID_SDO_RX(node_id);
    frame->rtr = 0;

    uint8_t data_size;
    switch (sdo_cmd) {
        case SDO_WRITE_1B:
            data_size = 1;
            break;
        case SDO_WRITE_2B:
            data_size = 2;
            break;
        case SDO_WRITE_3B:
            data_size = 3;
            break;
        case SDO_WRITE_4B:
            data_size = 4;
            break;
        default:
            data_size = 4;
            break;
    }

    frame->data[0] = sdo_cmd;
    frame->data[1] = index & 0xFF;
    frame->data[2] = (index >> 8) & 0xFF;
    frame->data[3] = subindex;

    // Copy data (little-endian)
    for (uint8_t i = 0; i < data_size; i++) {
        frame->data[4 + i] = (data >> (8 * i)) & 0xFF;
    }

    frame->dlc = 8;

    for (uint8_t i = 4 + data_size; i < 8; i++) {
        frame->data[i] = 0x00;
    }
}

void canopen_build_sdo_read_request(struct can_frame* frame, uint8_t node_id, uint16_t index, uint8_t subindex)
{
    frame->id = CANOPEN_COBID_SDO_RX(node_id);
    frame->rtr = 0;

    frame->data[0] = SDO_READ_0B;
    frame->data[1] = index & 0xFF;
    frame->data[2] = (index >> 8) & 0xFF;
    frame->data[3] = subindex;

    for (uint8_t i = 4; i < 8; i++) {
        frame->data[i] = 0x00;
    }

    frame->dlc = 8;
}

void canopen_build_nmt_request(struct can_frame* frame, uint8_t node_id, NMT_CMD cmd)
{
    frame->id = 0x00;
    frame->rtr = 0;

    frame->data[0] = cmd;
    frame->data[1] = node_id;

    frame->dlc = 2;
    for (uint8_t i = 2; i < 8; i++) {
        frame->data[i] = 0x00;
    }
}

void canopen_build_ds402_state_request(struct can_frame* frame, uint8_t node_id, DS402_STATE state)
{
    if (!frame) {
        JC_LOG("frame id NULL.");
        return;
    }

    switch (state) {
        case DS402_STATE_NOT_READY_TO_SWITCH_ON:
            break;
        case DS402_STATE_SWITCH_ON_DISABLED:
            canopen_build_sdo_write_request(frame, node_id, 0x6040, 0, 0x00, SDO_WRITE_2B);
            break;
        case DS402_STATE_READY_TO_SWITCH_ON:
            canopen_build_sdo_write_request(frame, node_id, 0x6040, 0, 0x06, SDO_WRITE_2B);
            break;
        case DS402_STATE_SWITCHED_ON:
            canopen_build_sdo_write_request(frame, node_id, 0x6040, 0, 0x07, SDO_WRITE_2B);
            break;
        case DS402_STATE_OPERATION_ENABLED:
            canopen_build_sdo_write_request(frame, node_id, 0x6040, 0, 0x0F, SDO_WRITE_2B);
            break;
        case DS402_STATE_QUICK_STOP_ACTIVE:
            canopen_build_sdo_write_request(frame, node_id, 0x6040, 0, 0x02, SDO_WRITE_2B);
            break;
        case DS402_STATE_FAULT_REACTION_ACTIVE:
            canopen_build_sdo_write_request(frame, node_id, 0x6040, 0, 0x80, SDO_WRITE_2B);
            break;
        case DS402_STATE_FAULT:
            break;
        default:
            JC_LOG();
            break;
    }
}

void canopen_build_ds402_state_get(struct can_frame* frame, uint8_t node_id)
{
    if (!frame) {
        JC_LOG("frame id NULL.");
        return;
    }

    canopen_build_sdo_read_request(frame, node_id, 0x6041, 0);
}

void canopen_build_sync(struct can_frame* frame, SYNC_TYPE type)
{
    if (!frame) {
        JC_LOG("frame id NULL.");
        return;
    }

    if (type == SYNC_TYPE_BASIC) {
        frame->id = 0x80;
        frame->rtr = 0;
        /**
         * The data length here should be 0,
         * but the bottom layer will filter out 0-length frames when forwarding,so we set it 1.
         */
        frame->dlc = 1;
        for (int i = 0; i < 8; i++) {
            frame->data[i] = 0;
        }
    } else if (type == SYNC_TYPE_WITH_COUNTER) {
        JC_LOG("TODO...");
    }
}