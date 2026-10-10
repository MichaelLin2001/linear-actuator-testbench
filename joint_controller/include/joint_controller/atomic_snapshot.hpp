#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>
namespace jc {
// One writer, multiple bounded readers. All data words are atomic; seq_cst
// also orders the two sequence checks around the payload on ARM.
template<class T> class AtomicSnapshot {
    static_assert(std::is_trivially_copyable<T>::value);
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
    static constexpr std::size_t count=(sizeof(T)+7)/8;
public:
    void store(const T& value) noexcept {
        std::array<std::uint64_t,count> data{};
        std::memcpy(data.data(),&value,sizeof(value));
        seq_.fetch_add(1);
        for(std::size_t i=0;i<count;++i)words_[i].store(data[i]);
        seq_.fetch_add(1);
    }
    bool load(T& value) const noexcept {
        for(unsigned attempt=0;attempt<4;++attempt){
            const auto before=seq_.load();if(before&1U)continue;
            std::array<std::uint64_t,count> data{};
            for(std::size_t i=0;i<count;++i)data[i]=words_[i].load();
            if(before==seq_.load()){std::memcpy(static_cast<void*>(&value),data.data(),sizeof(value));return true;}
        }
        return false;
    }
private:
    std::atomic<std::uint64_t> seq_{};
    std::array<std::atomic<std::uint64_t>,count> words_{};
};
}
