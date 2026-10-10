#pragma once

#include "joint_controller/types.hpp"

#include <semaphore.h>
#include <string>

namespace jc {

class JcShmAdapter {
public:
    JcShmAdapter() = default;
    ~JcShmAdapter();
    JcShmAdapter(const JcShmAdapter&) = delete;
    JcShmAdapter& operator=(const JcShmAdapter&) = delete;

    bool attach(std::string& error);
    FeedbackSnapshot read_snapshot() noexcept;
    const ReadStatistics& statistics() const noexcept { return stats_; }

private:
    const void* feedback_address_{};
    sem_t* feedback_semaphore_{SEM_FAILED};
    ReadStatistics stats_{};
};

}  // namespace jc
