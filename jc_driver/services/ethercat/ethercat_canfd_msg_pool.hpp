#pragma once

#include <memory>
#include <queue>
#include <mutex>
#include "queue.h"

class EtherCAT_CanfdMsgPool {
  public:
    static EtherCAT_CanfdMsgPool& instance();

    EtherCAT_CanfdMsg_ptr acquire();

    static void set_pool_size(size_t size);

  private:
    explicit EtherCAT_CanfdMsgPool(size_t pool_size);

    EtherCAT_CanfdMsgPool(const EtherCAT_CanfdMsgPool&) = delete;
    EtherCAT_CanfdMsgPool& operator=(const EtherCAT_CanfdMsgPool&) = delete;

  private:
    std::queue<EtherCAT_CanfdMsg*> pool_;
    std::mutex mtx_;

    static size_t initial_pool_size_;
};
