#pragma once

#include <inttypes.h>
#include <string.h>
#include "ethercat_config.h"

//********************************************//
//***********EtherCAT Message*****************//
//********************************************//
enum MessageType {
    ONESHOT,
    CONTINUED
};

#pragma pack(push, 1)
struct can_frame {
    uint32_t id;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
};

typedef struct {
    uint8_t motor_num;
    uint8_t can_ide;
    struct can_frame motor[ECAT_SLAVE_PER_MOTOR_COUNT];
} EtherCAT_Msg;
#pragma pack(pop)

//********************************************//
//*********** EtherCAT CANFD Message *********//
//********************************************//
#pragma pack(push, 1)
struct canfd_frame {
    uint16_t index;                              // 0x700x.1 (RxPdo) / 0x600x.1 (TxPdo) - TxIndex/RxIndex
    uint16_t id;                                 // 0x700x.2 (RxPdo) / 0x600x.2 (TxPdo) - TxID/RxID (16-bit in ESI)
    uint16_t len;                                // 0x700x.3 (RxPdo) / 0x600x.3 (TxPdo) - TxLen/RxLen
    uint8_t data[ECAT_CANFD_DATA_BYTES];         // 0x700x.4 (RxPdo) / 0x600x.4 (TxPdo) - TxData/RxData (240 bits = 30 bytes)
    uint8_t reserved_30[ECAT_CANFD_DATA_BYTES];  // Reserved area (240 bits = 30 bytes)
    uint32_t reserved_4;                         // Reserved area (32 bits = 4 bytes)
};

typedef struct {
    struct canfd_frame channel[ECAT_CANFD_CHANNELS];
} EtherCAT_CanfdPdo;

// Internal structure for queued CANFD frame data (TX/RX share the same layout)
typedef struct {
    uint8_t channel;                      // Channel number (0..7)
    uint16_t id;                          // CAN ID (16-bit)
    uint16_t len;                         // Data length (0..30 bytes)
    uint8_t data[ECAT_CANFD_DATA_BYTES];  // Data payload (30 bytes)
} EtherCAT_CanfdMsg;
#pragma pack(pop)