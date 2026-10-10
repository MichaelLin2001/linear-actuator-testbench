#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace jc {

// Exactly one producer and one consumer. A reserved slot is invisible until publish().
template <typename T, std::size_t Capacity>
class SpscRing {
    static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0);
    static_assert(std::is_trivially_copyable<T>::value);
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
public:
    T* reserve() noexcept {
        const auto head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) == Capacity) {
            return nullptr;
        }
        return &slots_[static_cast<std::size_t>(head) & (Capacity - 1)];
    }
    void publish() noexcept {
        head_.store(head_.load(std::memory_order_relaxed) + 1, std::memory_order_release);
    }
    bool try_push(const T& value) noexcept {
        auto* slot = reserve();
        if (slot == nullptr) {
            return false;
        }
        *slot = value;
        publish();
        return true;
    }
    bool try_pop(T& value) noexcept {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        value = slots_[static_cast<std::size_t>(tail) & (Capacity - 1)];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }
private:
    alignas(64) std::atomic<std::uint64_t> head_{};
    alignas(64) std::atomic<std::uint64_t> tail_{};
    alignas(64) std::array<T, Capacity> slots_{};
};

}  // namespace jc
