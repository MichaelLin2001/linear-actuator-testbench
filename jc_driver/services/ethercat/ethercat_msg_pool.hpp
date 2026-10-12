#pragma once

#include <memory>
#include <queue>
#include <mutex>
#include "queue.h"

class EtherCAT_MsgPool {
  public:
    static EtherCAT_MsgPool& instance();

    EtherCAT_Msg_ptr acquire();

    static void set_pool_size(size_t size);

  private:
    explicit EtherCAT_MsgPool(size_t pool_size);

    EtherCAT_MsgPool(const EtherCAT_MsgPool&) = delete;
    EtherCAT_MsgPool& operator=(const EtherCAT_MsgPool&) = delete;

  private:
    std::queue<EtherCAT_Msg*> pool_;
    std::mutex mtx_;

    static size_t initial_pool_size_;
};
