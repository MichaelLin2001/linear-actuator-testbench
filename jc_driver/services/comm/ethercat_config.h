#ifndef ETHERCAT_CONFIG_H
#define ETHERCAT_CONFIG_H
#ifdef __cplusplus
extern "C" {
#endif

#define ECAT_SLAVE_COUNT_MAX 20
#define ECAT_SLAVE_PER_MOTOR_COUNT 6

#define ECAT_MESSAGE_COUNT_MAX 100

#define ECAT_PDO_PERIOD_MS 1
#define ECAT_TIMEOUTMON_US 500

#define ECAT_MSG_POOL_SIZE 128

#define ECAT_CAN1_PASSAGE_BASE 1
#define ECAT_CAN2_PASSAGE_BASE 4

/**@brief Number of CANFD channels per slave (bidirectional, mapped in ESI as 8 channels) */
#define ECAT_CANFD_CHANNELS 8
/**@brief CANFD data payload size in bytes (mapped as 240 bits = 30 bytes in ESI) */
#define ECAT_CANFD_DATA_BYTES 30

/**@brief EtherCAT errors are measured over this period of loop iterations */
#define ECAT_ERR_PERIOD 100
/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define ECAT_ERR_MAX 20

#ifdef __cplusplus
};
#endif

#endif  // ETHERCAT_CONFIG_H