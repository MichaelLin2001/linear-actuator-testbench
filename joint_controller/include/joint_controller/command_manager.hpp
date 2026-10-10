#pragma once
#include "joint_controller/spsc_ring.hpp"
#include "joint_controller/types.hpp"
namespace jc {
class CommandManager {
public:
    bool submit(Command command, std::uint64_t now_ns) noexcept {
        command.sequence = ++sequence_;
        command.received_ns = now_ns;
        return commands_.try_push(command);
    }
    bool submit(CommandType type, std::uint64_t now_ns) noexcept {
        Command command{}; command.type = type; return submit(command, now_ns);
    }
    bool poll(Command& command) noexcept { return commands_.try_pop(command); }
    std::uint64_t last_submitted_sequence() const noexcept { return sequence_; }
private:
    SpscRing<Command, 32> commands_{};
    std::uint64_t sequence_{};
};
}  // namespace jc
