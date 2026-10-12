#include "ethercat_canfd_msg_pool.hpp"
#include "ethercat_config.h"
#include "jc_log.h"

size_t EtherCAT_CanfdMsgPool::initial_pool_size_ = ECAT_MSG_POOL_SIZE;

EtherCAT_CanfdMsgPool::EtherCAT_CanfdMsgPool(size_t pool_size)
{
    for (size_t i = 0; i < pool_size; ++i) {
        pool_.push(new EtherCAT_CanfdMsg());
    }
}

EtherCAT_CanfdMsgPool& EtherCAT_CanfdMsgPool::instance()
{
    static EtherCAT_CanfdMsgPool inst(initial_pool_size_);
    return inst;
}

EtherCAT_CanfdMsg_ptr EtherCAT_CanfdMsgPool::acquire()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (pool_.empty()) {
        JC_LOG_WARN("CANFD Message Pool EMPTY");
        return EtherCAT_CanfdMsg_ptr(new EtherCAT_CanfdMsg(), [](EtherCAT_CanfdMsg* p) { delete p; });
    }

    EtherCAT_CanfdMsg* obj = pool_.front();
    pool_.pop();
    return EtherCAT_CanfdMsg_ptr(obj, [this](EtherCAT_CanfdMsg* p) {
        std::lock_guard<std::mutex> lock(mtx_);
        pool_.push(p);
    });
}

void EtherCAT_CanfdMsgPool::set_pool_size(size_t size)
{
    initial_pool_size_ = size;
}
