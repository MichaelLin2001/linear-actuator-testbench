#ifndef TRANSMIT_H
#define TRANSMIT_H
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <condition_variable>
#include "sys/time.h"
#include "ethercat.h"
extern "C" {
#include "message_define.h"
}

#ifdef __cplusplus
extern "C" {
#endif

void ethercat_reset_continue_tx_buffers(void);
void ethercat_init(const char* ifname);
void ethercat_thread_start(void);
void ethercat_thread_stop(void);
int ethercat_send_message_queue(int slaveId, const EtherCAT_Msg tx_message, EtherCAT_Msg* rx_message, int timeout_ms);
int ethercat_send_canopen_sdo_request(int slaveId, const EtherCAT_Msg tx_message,
                                     EtherCAT_Msg* rx_message, int timeout_ms);
int ethercat_send_message_continue(int slaveId, const EtherCAT_Msg tx_message);

int ethercat_send_canfd_message_queue(int slaveId, const EtherCAT_CanfdMsg tx_message, EtherCAT_CanfdMsg* rx_message, int timeout_ms);
int ethercat_send_canfd_message_continue(int slaveId, const EtherCAT_CanfdMsg tx_message);
#ifdef __cplusplus
};
#endif

void txMessageCallBack(void);
void rxMessageCallBack(void);
void ethercat_notify_response(void);
void ethercat_set_query_response(uint8_t slaveId, EtherCAT_Msg rx_message);
void ethercat_set_canfd_query_response(uint8_t slaveId, EtherCAT_CanfdMsg rx_message);
#endif  // PROJECT_RT_ETHERCAT_H
