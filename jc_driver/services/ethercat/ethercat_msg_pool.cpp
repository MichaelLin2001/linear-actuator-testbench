#include "ethercat_msg_pool.hpp"
#include "ethercat_config.h"
#include "jc_log.h"

size_t EtherCAT_MsgPool::initial_pool_size_ = ECAT_MSG_POOL_SIZE;

EtherCAT_MsgPool::EtherCAT_MsgPool(size_t pool_size)
{
    for (size_t i = 0; i < pool_size; ++i) {
        pool_.push(new EtherCAT_Msg());
    }
}

EtherCAT_MsgPool& EtherCAT_MsgPool::instance()
{
    static EtherCAT_MsgPool inst(initial_pool_size_);
    return inst;
}

EtherCAT_Msg_ptr EtherCAT_MsgPool::acquire()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (pool_.empty()) {
        JC_LOG_WARN("Message Pool EMPTY");
        return EtherCAT_Msg_ptr(new EtherCAT_Msg(), [](EtherCAT_Msg* p) { delete p; });
    } else {
        EtherCAT_Msg* obj = pool_.front();
        pool_.pop();
        return EtherCAT_Msg_ptr(obj, [this](EtherCAT_Msg* p) {
            std::lock_guard<std::mutex> lock(mtx_);
            pool_.push(p);
        });
    }
}

void EtherCAT_MsgPool::set_pool_size(size_t size)
{
    initial_pool_size_ = size;
}
