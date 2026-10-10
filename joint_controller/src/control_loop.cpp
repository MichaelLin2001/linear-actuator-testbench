#include "joint_controller/control_loop.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <time.h>
namespace jc {
std::uint64_t monotonic_now_ns() noexcept {
    timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t)!=0)return 0;
    return static_cast<std::uint64_t>(t.tv_sec)*1'000'000'000ULL+static_cast<std::uint64_t>(t.tv_nsec);
}
ControlLoop::ControlLoop(JcShmAdapter& a,CommandManager& c,DataLogger& l,LoopConfig config,OutputManager* o,ForceSensorReader* s) noexcept
    :adapter_(a),commands_(c),logger_(l),config_(config),output_(o),sensor_(s),state_(config.state,reset_) {}
ControlLoop::~ControlLoop(){request_stop();join();}
bool ControlLoop::start(std::string& error){
    if(config_.output_enabled&&!output_){error="output manager missing";return false;}
    running_.store(true);heartbeat();thread_=std::thread(&ControlLoop::worker,this);return true;
}
void ControlLoop::join(){if(thread_.joinable())thread_.join();}
void ControlLoop::worker() noexcept {
    state_.initialize();ControlStatus status{};bool configured=false,have_reference=false,have_contact=false;
    const auto start=monotonic_now_ns();auto scheduled=start+kPeriodNs,previous=start;
    std::uint64_t preload_start=0,contact_since=0,experiment_start=0,quiet_since=0;
    double origin=0,contact_position=0,reference=0;ExperimentStage stage=ExperimentStage::Idle;
    while(!stop_.load()) {
        timespec wake{static_cast<time_t>(scheduled/1'000'000'000ULL),static_cast<long>(scheduled%1'000'000'000ULL)};
        int rc=0;do{rc=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&wake,nullptr);}while(rc==EINTR&&!stop_.load());
        const auto now=monotonic_now_ns();if(now==0||rc!=0){state_.latch_fault(Fault::Clock);break;}
        if(now-start>=config_.duration_ns)break;
        CycleRecord r{};r.cycle_index=summary_.cycles;r.actual_wakeup_ns=now;r.scheduled_wakeup_ns=scheduled;
        r.wakeup_jitter_ns=static_cast<std::int64_t>(now)-static_cast<std::int64_t>(scheduled);
        r.actual_dt_ns=now-previous;r.actual_dt_valid=true;previous=now;
        r.control_mode=config_.output_enabled?ControlMode::CspBuiltin:ControlMode::ReadOnlyDryRun;
        r.feedback=adapter_.read_snapshot();r.reads=adapter_.statistics();
        r.feedback.feedback_valid=r.feedback.snapshot_read_ok&&(r.feedback.finite_mask&config_.state.active_mask)==config_.state.active_mask;
        const auto last=r.reads.last_success_timestamp_ns?r.reads.last_success_timestamp_ns:start;
        r.snapshot_age_ns=now>=last?now-last:0;r.snapshot_read_timeout=r.snapshot_age_ns>=config_.state.snapshot_timeout_ns;
        r.force=force_.update(sensor_?sensor_->get_latest():ForceSample{},now);const auto& j=r.feedback.joints[kJ3];
        if(r.feedback.feedback_valid&&!have_reference){reference=j.position;have_reference=true;}
        const bool quiet=r.feedback.feedback_valid&&std::abs(j.speed)<=0.0002&&!trajectory_.active()&&stage!=ExperimentStage::Preload;
        if(!quiet)quiet_since=0;else if(!quiet_since)quiet_since=now;
        status.stationary=quiet_since&&now-quiet_since>=200'000'000ULL;
        const bool urgent=emergency_.exchange(false);
        if(urgent)state_.latch_fault(Fault::EmergencyStop);
        if(state_.state()==State::Run) {
            const auto hb=heartbeat_ns_.load();
            if(!configured||now<hb||static_cast<double>(now-hb)*1e-9>experiment_.heartbeat_timeout_s)state_.latch_fault(Fault::Heartbeat);
            const double age=r.force.timestamp_ns&&now>=r.force.timestamp_ns?static_cast<double>(now-r.force.timestamp_ns)*1e-9:1e9;
            if(r.feedback.feedback_valid&&(!safe_measurement(experiment_,j.position,j.speed,r.force.zeroed?r.force.external:r.force.raw,age,r.force.valid)||
                (r.force.zeroed&&std::abs(r.force.external)>=experiment_.force_limit_n)))state_.latch_fault(Fault::ExperimentSafety);
            if(r.actual_dt_ns>20'000'000ULL)state_.latch_fault(Fault::ExperimentSafety);
            if(experiment_start&&static_cast<double>(now-experiment_start)*1e-9>experiment_.experiment_timeout_s&&stage!=ExperimentStage::Complete)state_.latch_fault(Fault::ExperimentSafety);
            if(have_contact&&std::abs(j.position-contact_position)>experiment_.contact_displacement&&stage!=ExperimentStage::Retract&&stage!=ExperimentStage::Complete)state_.latch_fault(Fault::ExperimentSafety);
        }
        Command c{};commands_.poll(c);r.command=c.type;
        Command checked_command=c;
        if(urgent&&c.type==CommandType::Reset)checked_command.type=CommandType::None;
        r.command_result=state_.step(r.feedback,r.reads,r.snapshot_read_timeout,checked_command);
        if(urgent&&c.type==CommandType::Reset)r.command_result=CommandResult::RejectedState;
        if(c.type==CommandType::Configure&&r.command_result==CommandResult::Accepted){
            if(state_.state()!=State::Idle||!valid_experiment(c.experiment))r.command_result=CommandResult::RejectedArgument;
            else{experiment_=c.experiment;configured=true;stage=ExperimentStage::Idle;experiment_start=0;have_contact=false;}
        }
        if(c.type==CommandType::ForceZero&&r.command_result==CommandResult::Accepted){
            if(!status.stationary||!r.force.valid||(stage!=ExperimentStage::Idle&&stage!=ExperimentStage::Complete)||
                (r.force.zeroed&&configured&&std::abs(r.force.external)>=experiment_.contact_force_n))r.command_result=CommandResult::RejectedFeedback;
            else{force_.begin_zero(now);status.zero_sequence=c.sequence;}
        }
        if(c.type==CommandType::Run&&r.command_result==CommandResult::Accepted)reference=j.position;
        if(c.type==CommandType::Preload&&r.command_result==CommandResult::Accepted){
            if(!configured||state_.state()!=State::Run||!status.stationary||!r.force.valid||!r.force.zeroed||
                (stage!=ExperimentStage::Idle&&stage!=ExperimentStage::Complete))r.command_result=CommandResult::RejectedState;
            else{stage=ExperimentStage::Preload;preload_start=experiment_start=now;contact_since=0;have_contact=false;origin=j.position;reference=j.position;status.experiment_id++;}
        }
        if(c.type==CommandType::Unload&&r.command_result==CommandResult::Accepted){
            // Current output is locked; only the existing CSP contact is released here.
            if(!configured||state_.state()!=State::Run||(stage!=ExperimentStage::Ready&&stage!=ExperimentStage::Preload))r.command_result=CommandResult::RejectedState;
            else{const double target=j.position-static_cast<double>(experiment_.direction)*experiment_.retract_distance;
                if(target<experiment_.position_min||target>experiment_.position_max||!trajectory_.start(now,j.position,target,experiment_.retract_speed,false,c.sequence))r.command_result=CommandResult::RejectedArgument;
                else stage=ExperimentStage::Retract;}
        }
        if(c.type==CommandType::Move)r.command_result=CommandResult::RejectedArgument;
        if(c.type==CommandType::Stop){trajectory_.stop(j.position,c.sequence);force_.cancel_zero();}
        if(c.type==CommandType::Reset&&r.command_result==CommandResult::Accepted){stage=ExperimentStage::Idle;have_contact=false;experiment_start=0;reference=j.position;}
        if(state_.state()==State::Run&&stage==ExperimentStage::Preload&&r.feedback.feedback_valid){
            if(std::abs(j.position-origin)>=experiment_.max_travel||static_cast<double>(now-preload_start)*1e-9>=experiment_.preload_timeout_s)state_.latch_fault(Fault::ExperimentSafety);
            else if(std::abs(r.force.external)>=experiment_.contact_force_n){
                if(!contact_since){contact_since=now;reference=j.position;contact_position=j.position;have_contact=true;}
                if(static_cast<double>(now-contact_since)*1e-9>=experiment_.contact_confirm_s)stage=ExperimentStage::Ready;
            }else{contact_since=0;reference+=static_cast<double>(experiment_.direction)*experiment_.preload_speed*static_cast<double>(r.actual_dt_ns)*1e-9;
                if(std::abs(reference-origin)>experiment_.max_travel)state_.latch_fault(Fault::ExperimentSafety);}
        }
        if(state_.state()==State::Run&&stage==ExperimentStage::Retract){
            const auto sample=trajectory_.sample(now);reference=sample.position;r.motion_phase=sample.phase;
            if(sample.complete){if(std::abs(r.force.external)>=experiment_.contact_force_n)state_.latch_fault(Fault::ExperimentSafety);
                else{stage=ExperimentStage::Complete;have_contact=false;}}
        }
        if(config_.output_enabled&&state_.state()==State::Run&&r.feedback.feedback_valid){
            r.tracking_error=j.position-reference;
            if(reference<experiment_.position_min||reference>experiment_.position_max||std::abs(r.tracking_error)>config_.maximum_tracking_error_m)state_.latch_fault(Fault::TrackingError);
            else{const auto ws=output_->write_csp(reference);r.target_write_ok=ws==TargetWriteStatus::Ok;
                if(output_->consecutive_failures()>20||ws==TargetWriteStatus::SemaphoreError||ws==TargetWriteStatus::ReleaseError)state_.latch_fault(Fault::TargetWrite);}
        }
        if(logger_.io_error())state_.latch_fault(Fault::Logger);
        if(state_.state()==State::Fault){stage=ExperimentStage::Fault;trajectory_.stop(reference,c.sequence);force_.cancel_zero();}
        r.force=force_.value();r.state=state_.state();r.fault=state_.fault();r.reference_position=reference;
        r.experiment_stage=stage;r.experiment_id=status.experiment_id;r.experiment_direction=configured?experiment_.direction:0;
        r.last_command_sequence=c.type==CommandType::None?status.acknowledged_sequence:c.sequence;
        r.target_write_failures=output_?output_->failures():0;r.submitted_csp_position=output_?output_->last_submitted():0;
        r.cycle_valid=r.feedback.feedback_valid&&r.state!=State::Fault;r.dropped_record_count=logger_.dropped_records();
        status.state=r.state;status.fault=r.fault;status.stage=stage;status.force=r.force;status.configured=configured;
        status.feedback_valid=r.feedback.feedback_valid;
        status.actual_position=std::isfinite(j.position)?j.position:0;status.actual_velocity=std::isfinite(j.speed)?j.speed:0;
        status.effort=std::isfinite(j.effort)?j.effort:0;status.reference_position=reference;
        if(c.type!=CommandType::None){status.acknowledged_sequence=c.sequence;status.command_result=r.command_result;}
        public_.store(status);
        const auto end=monotonic_now_ns();const auto next=advance_schedule(scheduled,end);
        r.cycle_execution_ns=end>=now?end-now:0;r.deadline_miss=end>=scheduled+kPeriodNs;r.skipped_after_cycle=next.skipped;
        if(auto* slot=logger_.reserve_record()){*slot=r;logger_.publish_record();}
        ++summary_.cycles;summary_.valid_cycles+=r.cycle_valid?1U:0U;summary_.deadline_misses+=r.deadline_miss?1U:0U;
        summary_.skipped_periods+=next.skipped;++summary_.dt_samples;summary_.sum_actual_dt_ns+=r.actual_dt_ns;
        summary_.min_actual_dt_ns=std::min(summary_.min_actual_dt_ns,r.actual_dt_ns);summary_.max_actual_dt_ns=std::max(summary_.max_actual_dt_ns,r.actual_dt_ns);
        summary_.max_cycle_execution_ns=std::max(summary_.max_cycle_execution_ns,r.cycle_execution_ns);
        summary_.sum_cycle_execution_ns+=r.cycle_execution_ns;summary_.sum_wakeup_jitter_ns+=r.wakeup_jitter_ns;
        if(r.wakeup_jitter_ns>0)summary_.max_wakeup_jitter_ns=std::max(summary_.max_wakeup_jitter_ns,static_cast<std::uint64_t>(r.wakeup_jitter_ns));
        scheduled=next.next_ns;
    }
    state_.stop();summary_.reads=adapter_.statistics();summary_.final_state=state_.state();summary_.final_fault=state_.fault();running_.store(false);
}
}
