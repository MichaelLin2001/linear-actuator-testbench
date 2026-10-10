#pragma once
#include "joint_controller/command_manager.hpp"
#include "joint_controller/data_logger.hpp"
#include "joint_controller/jc_shm_adapter.hpp"
#include "joint_controller/output_manager.hpp"
#include "joint_controller/state_machine.hpp"
#include "joint_controller/trajectory_generator.hpp"
#include "joint_controller/atomic_snapshot.hpp"
#include <atomic>
#include <thread>
namespace jc {
std::uint64_t monotonic_now_ns() noexcept;
struct ControlStatus {
    State state{State::Init}; Fault fault{Fault::None}; ExperimentStage stage{ExperimentStage::Idle};
    ProcessedForce force{}; std::uint64_t acknowledged_sequence{},zero_sequence{},experiment_id{};
    CommandResult command_result{CommandResult::None};
    double actual_position{},actual_velocity{},effort{},reference_position{};
    bool stationary{},configured{},feedback_valid{};
};
class ControlLoop {
public:
    ControlLoop(JcShmAdapter&,CommandManager&,DataLogger&,LoopConfig,OutputManager* =nullptr,ForceSensorReader* =nullptr) noexcept;
    ~ControlLoop();
    bool start(std::string& error);
    void request_stop() noexcept { stop_.store(true); }
    void emergency_stop() noexcept { emergency_.store(true); }
    void heartbeat() noexcept { heartbeat_ns_.store(monotonic_now_ns()); }
    bool running() const noexcept { return running_.load(); }
    bool status(ControlStatus& s) const noexcept { return public_.load(s); }
    void join(); const LoopSummary& summary() const noexcept { return summary_; }
private:
    void worker() noexcept;
    JcShmAdapter& adapter_; CommandManager& commands_; DataLogger& logger_;
    LoopConfig config_; OutputManager* output_; ForceSensorReader* sensor_;
    ForceProcessor force_{}; DryRunControllerState reset_{}; StateMachine state_;
    TrajectoryGenerator trajectory_{}; ExperimentConfig experiment_{};
    AtomicSnapshot<ControlStatus> public_{}; LoopSummary summary_{};
    std::atomic<bool> stop_{},emergency_{},running_{}; std::atomic<std::uint64_t> heartbeat_ns_{};
    std::thread thread_;
};
}
