#ifndef _CAN_OPEN_H_
#define _CAN_OPEN_H_
#include <stdint.h>
#include "message_define.h"

#define CANOPEN_COBID(fc, node_id)  (((uint16_t)(fc) << 7) | ((node_id) & 0x7F))
#define CANOPEN_GET_FC(cob_id)      ((uint8_t)((cob_id) >> 7))
#define CANOPEN_GET_NODE_ID(cob_id) ((uint8_t)((cob_id) & 0x7F))

#define CANOPEN_RPDO_COMM_BASE  0x1400
#define CANOPEN_RPDO_MAP_BASE   0x1600
#define CANOPEN_TPDO_COMM_BASE  0x1800
#define CANOPEN_TPDO_MAP_BASE   0x1A00

/* Standard Function Code Definitions (Upper 4 bits) */
typedef enum {
    CANOPEN_FC_NMT       = 0x00, /**< Network Management */
    CANOPEN_FC_SYNC      = 0x01, /**< Synchronization */
    CANOPEN_FC_EMERGENCY = 0x01, /**< Emergency (diff in node_id) */
    CANOPEN_FC_TIME      = 0x02, /**< Time Stamp */
    CANOPEN_FC_TPDO1     = 0x03, /**< Transmit PDO 1 */
    CANOPEN_FC_RPDO1     = 0x04, /**< Receive PDO 1 */
    CANOPEN_FC_TPDO2     = 0x05, /**< Transmit PDO 2 */
    CANOPEN_FC_RPDO2     = 0x06, /**< Receive PDO 2 */
    CANOPEN_FC_TPDO3     = 0x07, /**< Transmit PDO 3 */
    CANOPEN_FC_RPDO3     = 0x08, /**< Receive PDO 3 */
    CANOPEN_FC_TPDO4     = 0x09, /**< Transmit PDO 4 */
    CANOPEN_FC_RPDO4     = 0x0A, /**< Receive PDO 4 */
    CANOPEN_FC_SDO_TX    = 0x0B, /**< SDO Transmit (slave->master) */
    CANOPEN_FC_SDO_RX    = 0x0C, /**< SDO Receive (master->slave) */
    CANOPEN_FC_HEARTBEAT = 0x0E, /**< Heartbeat */
    CANOPEN_FC_LSS       = 0x0F  /**< Layer Setting Service */
} CANOPEN_FC;

/* Predefined Broadcast COB-IDs (Node ID part is 0) */
#define CANOPEN_COBID_NMT         CANOPEN_COBID(CANOPEN_FC_NMT, 0)
#define CANOPEN_COBID_SYNC        CANOPEN_COBID(CANOPEN_FC_SYNC, 0)
// Note: SYNC/TIME typically use Node ID 0 for broadcasting
#define CANOPEN_COBID_PDO(pdo_num, node_id)    CANOPEN_COBID(pdo_num, (node_id))

#define CANOPEN_COBID_EMERGENCY(node_id)   CANOPEN_COBID(CANOPEN_FC_EMERGENCY, (node_id))
#define CANOPEN_COBID_SDO_TX(node_id)      CANOPEN_COBID(CANOPEN_FC_SDO_TX, (node_id))
#define CANOPEN_COBID_SDO_RX(node_id)      CANOPEN_COBID(CANOPEN_FC_SDO_RX, (node_id))
#define CANOPEN_COBID_HEARTBEAT(node_id)   CANOPEN_COBID(CANOPEN_FC_HEARTBEAT, (node_id))

/**
  * @brief  SDO Command Specifier Codes
  * @note   These codes define the type of SDO transfer operation
  */
typedef enum {
    SDO_WRITE_1B = 0x2F, /**< Initiate Write (expedited), size indicator = 1 byte */
    SDO_WRITE_2B = 0x2B, /**< Initiate Write (expedited), size indicator = 2 bytes */
    SDO_WRITE_3B = 0x27, /**< Initiate Write (expedited), size indicator = 3 bytes */
    SDO_WRITE_4B = 0x23, /**< Initiate Write (expedited), size indicator = 4 bytes */

    SDO_FDB = 0x60,      /**< Success Response (expedited write/read confirmation) */

    SDO_READ_0B = 0x40,  /**< Initiate Read request (no size indication) */
    SDO_READ_1B = 0x4F,  /**< Read Response (expedited), size indicator = 1 byte */
    SDO_READ_2B = 0x4B,  /**< Read Response (expedited), size indicator = 2 bytes */
    SDO_READ_3B = 0x47,  /**< Read Response (expedited), size indicator = 3 bytes */
    SDO_READ_4B = 0x43,  /**< Read Response (expedited), size indicator = 4 bytes */

    SDO_ERR_FDB = 0x80,  /**< Error Response (abort SDO transfer) */
} SDO_CMD;

typedef enum {
    NMT_CMD_START_REMOTE_NODE      = 0x01, /**< Start remote node command */
    NMT_CMD_STOP_REMOTE_NODE       = 0x02, /**< Stop remote node command */
    NMT_CMD_ENTER_PRE_OPERATIONAL  = 0x80, /**< Enter pre-operational state command */
    NMT_CMD_RESET_NODE             = 0x81, /**< Reset node command */
    NMT_CMD_RESET_COMMUNICATION    = 0x82  /**< Reset communication command */
} NMT_CMD;

// CiA402 (DS402) State Machine states
typedef enum
{
    DS402_STATE_NOT_READY_TO_SWITCH_ON = 0,   // 0: Initial state after power on
    DS402_STATE_SWITCH_ON_DISABLED,           // 1
    DS402_STATE_READY_TO_SWITCH_ON,           // 2
    DS402_STATE_SWITCHED_ON,                  // 3
    DS402_STATE_OPERATION_ENABLED,            // 4
    DS402_STATE_QUICK_STOP_ACTIVE,            // 5
    DS402_STATE_FAULT_REACTION_ACTIVE,        // 6
    DS402_STATE_FAULT                         // 7
} DS402_STATE;

typedef enum {
    SYNC_TYPE_BASIC = 0,
    SYNC_TYPE_WITH_COUNTER
} SYNC_TYPE;

typedef enum {
    RXPDO
} PDO_TYPE;

#define SDO_GET_CMD(data) ((SDO_CMD)((data)[0]))
#define SDO_IS_WRITE_CMD(cmd) (((cmd) & 0xE0) == 0x20)
#define SDO_IS_READ_RESP(cmd) (((cmd) & 0xE0) == 0x40)
#define SDO_IS_ERROR(cmd) (((cmd) & 0xE0) == 0x80)

static inline uint8_t sdo_get_data_size(SDO_CMD cmd)
{
    switch (cmd) {
        case SDO_WRITE_1B:
        case SDO_READ_1B:
            return 1;
        case SDO_WRITE_2B:
        case SDO_READ_2B:
            return 2;
        case SDO_WRITE_3B:
        case SDO_READ_3B:
            return 3;
        case SDO_WRITE_4B:
        case SDO_READ_4B:
            return 4;
        case SDO_READ_0B:
        case SDO_FDB:
        default:
            return 0;
    }
}

void canopen_build_sdo_write_request(struct can_frame* frame, uint8_t node_id,
    uint16_t index, uint8_t subindex, int32_t data, SDO_CMD sdo_cmd);
void canopen_build_sdo_read_request(struct can_frame* frame, uint8_t node_id, uint16_t index, uint8_t subindex);
void canopen_build_nmt_request(struct can_frame* frame, uint8_t node_id, NMT_CMD cmd);

void canopen_build_ds402_state_request(struct can_frame* frame, uint8_t node_id, DS402_STATE state);
void canopen_build_ds402_state_get(struct can_frame* frame, uint8_t node_id);
void canopen_build_sync(struct can_frame* frame, SYNC_TYPE type);
#endif  // _CAN_OPEN_H_