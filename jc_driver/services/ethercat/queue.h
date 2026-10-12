#ifndef SOEM_QUEUE_H
#define SOEM_QUEUE_H
#include <boost/lockfree/spsc_queue.hpp>
#include <atomic>
#include <memory>
#include <thread>
extern "C" {
#include "message_define.h"
};

using boost::lockfree::spsc_queue, boost::lockfree::capacity;
typedef std::shared_ptr<EtherCAT_Msg> EtherCAT_Msg_ptr;
typedef std::shared_ptr<EtherCAT_CanfdMsg> EtherCAT_CanfdMsg_ptr;

extern spsc_queue<EtherCAT_Msg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>> rx_message_queue[];
extern spsc_queue<EtherCAT_CanfdMsg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>> canfd_rx_message_queue[];

void ethercat_push_txmessage_queue(spsc_queue<EtherCAT_Msg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>>& queue, const EtherCAT_Msg_ptr& msg);
void ethercat_push_rxmessage_queue(spsc_queue<EtherCAT_Msg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>>& queue, const EtherCAT_Msg_ptr& msg);
void ethercat_push_canfd_txmessage_queue(spsc_queue<EtherCAT_CanfdMsg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>>& queue, const EtherCAT_CanfdMsg_ptr& msg);
void ethercat_push_canfd_rxmessage_queue(spsc_queue<EtherCAT_CanfdMsg_ptr, capacity<ECAT_MESSAGE_COUNT_MAX>>& queue, const EtherCAT_CanfdMsg_ptr& msg);
#endif  //SOEM_QUEUE_H
