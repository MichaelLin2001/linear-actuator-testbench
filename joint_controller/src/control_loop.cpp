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
ControlLoop::ControlLoop(JcShmAdapter& a,CommandManager& c,DataLogger& l,LoopConfig config,OutputManager* o,ForceShm* s) noexcept
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
    ProcessedForce baseline{};CurrentTrajectory current_profile;
    bool raw_output=false,raw_stop_sent=false;double current_command=0,load_initial=0,ready_sum=0;unsigned ready_samples=0;
    std::uint64_t load_start=0;unsigned level=0;
    double origin=0,contact_position=0,reference=0;ExperimentStage stage=ExperimentStage::Idle;
    while(!stop_.load()) {
        timespec wake{static_cast<time_t>(scheduled/1'000'000'000ULL),static_cast<long>(scheduled%1'000'000'000ULL)};
        int rc=0;do{rc=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&wake,nullptr);}while(rc==EINTR&&!stop_.load());
        const auto now=monotonic_now_ns();if(now==0||rc!=0){state_.latch_fault(Fault::Clock);break;}
        if(now-start>=config_.duration_ns)break;
        CycleRecord r{};r.cycle_index=summary_.cycles;r.actual_wakeup_ns=now;r.scheduled_wakeup_ns=scheduled;
        r.wakeup_jitter_ns=static_cast<std::int64_t>(now)-static_cast<std::int64_t>(scheduled);
        r.actual_dt_ns=now-previous;r.actual_dt_valid=true;previous=now;
        r.control_mode=!config_.output_enabled?ControlMode::ReadOnlyDryRun:raw_output?ControlMode::CstRaw:ControlMode::CspBuiltin;
        r.feedback=adapter_.read_snapshot();r.reads=adapter_.statistics();
        r.feedback.feedback_valid=r.feedback.snapshot_read_ok&&(r.feedback.finite_mask&config_.state.active_mask)==config_.state.active_mask;
        const auto last=r.reads.last_success_timestamp_ns?r.reads.last_success_timestamp_ns:start;
        r.snapshot_age_ns=now>=last?now-last:0;r.snapshot_read_timeout=r.snapshot_age_ns>=config_.state.snapshot_timeout_ns;
        r.force=sensor_?sensor_->read(r.force_read_timestamp_ns):ProcessedForce{};const auto& j=r.feedback.joints[kJ3];
        if(r.feedback.feedback_valid&&!have_reference){reference=position_coordinate(experiment_,origin,j.position);have_reference=true;}
        const bool quiet=r.feedback.feedback_valid&&std::abs(j.speed)<=0.0002&&!trajectory_.active()&&stage!=ExperimentStage::Preload;
        if(!quiet)quiet_since=0;else if(!quiet_since)quiet_since=now;
        status.stationary=quiet_since&&now-quiet_since>=200'000'000ULL;
        const bool urgent=emergency_.exchange(false);
        if(urgent)state_.latch_fault(Fault::EmergencyStop);
        if(state_.state()==State::Run) {
            if(raw_output&&r.feedback.raw[kJ3].reserved[1]!=0)state_.latch_fault(Fault::TargetWrite);
            if(!same_force_zero(baseline,r.force))state_.latch_fault(Fault::ForceSensor);
            r.heartbeat_timestamp_ns=heartbeat_ns_.load();
            r.heartbeat_check_timestamp_ns=monotonic_now_ns();
            if(!configured||!heartbeat_fresh(r.heartbeat_timestamp_ns,r.heartbeat_check_timestamp_ns,experiment_.heartbeat_timeout_s))state_.latch_fault(Fault::Heartbeat);
            const double age=r.force.timestamp_ns&&r.force_read_timestamp_ns>=r.force.timestamp_ns?static_cast<double>(r.force_read_timestamp_ns-r.force.timestamp_ns)*1e-9:1e9;
            if(r.feedback.feedback_valid&&(!safe_measurement(experiment_,j.position,j.speed,r.force.zeroed?r.force.external:r.force.raw,age,r.force.valid)||
                (r.force.zeroed&&std::abs(r.force.external)>=experiment_.force_limit_n)))state_.latch_fault(Fault::ExperimentSafety);
            if(r.actual_dt_ns>20'000'000ULL)state_.latch_fault(Fault::ExperimentSafety);
            if(experiment_start&&static_cast<double>(now-experiment_start)*1e-9>experiment_.experiment_timeout_s&&stage!=ExperimentStage::Complete)state_.latch_fault(Fault::ExperimentSafety);
            if(have_contact&&std::abs(j.position-contact_position)>experiment_.contact_displacement&&stage!=ExperimentStage::Retract&&stage!=ExperimentStage::Complete)state_.latch_fault(Fault::ExperimentSafety);
        }
        if(stage==ExperimentStage::Ready&&r.feedback.feedback_valid&&status.stationary){
            ready_sum+=j.effort;++ready_samples;
        }else{ready_sum=0;ready_samples=0;}
        Command c{};commands_.poll(c);r.command=c.type;
        Command checked_command=c;
        if(urgent&&c.type==CommandType::Reset)checked_command.type=CommandType::None;
        r.command_result=state_.step(r.feedback,r.reads,r.snapshot_read_timeout,checked_command);
        if(urgent&&c.type==CommandType::Reset)r.command_result=CommandResult::RejectedState;
        if(c.type==CommandType::Configure&&r.command_result==CommandResult::Accepted){
            if(state_.state()!=State::Idle||!valid_experiment(c.experiment))r.command_result=CommandResult::RejectedArgument;
            else{experiment_=c.experiment;reference=position_coordinate(experiment_,origin,j.position);configured=true;stage=ExperimentStage::Idle;experiment_start=0;have_contact=false;}
        }
        if(c.type==CommandType::ForceZero)r.command_result=CommandResult::RejectedArgument;
        if(c.type==CommandType::Run&&r.command_result==CommandResult::Accepted){baseline=r.force;reference=position_coordinate(experiment_,origin,j.position);}
        if(c.type==CommandType::Preload&&r.command_result==CommandResult::Accepted){
            if(!configured||state_.state()!=State::Run||!status.stationary||!r.force.valid||!r.force.zeroed||
                (stage!=ExperimentStage::Idle&&stage!=ExperimentStage::Complete))r.command_result=CommandResult::RejectedState;
            else{stage=ExperimentStage::Preload;preload_start=experiment_start=now;contact_since=0;have_contact=false;origin=j.position;reference=position_coordinate(experiment_,origin,j.position);status.experiment_id++;}
        }
        if(c.type==CommandType::BeginLoad&&r.command_result==CommandResult::Accepted){
            const double seed=ready_samples?std::round(experiment_.direction*ready_sum/static_cast<double>(ready_samples)):0;
            if(!config_.output_enabled||state_.state()!=State::Run||stage!=ExperimentStage::Ready||!status.stationary||ready_samples<100||
                r.feedback.raw[kJ3].reserved[0]!=2||seed<0||seed>=experiment_.max_current_raw||!current_profile.start(experiment_,std::abs(seed)))r.command_result=CommandResult::RejectedState;
            else{current_command=seed*experiment_.direction;load_initial=static_cast<double>(experiment_.direction);raw_output=true;raw_stop_sent=false;stage=ExperimentStage::SwitchingCst;
                contact_position=j.position;have_contact=true;}
        }
        if(c.type==CommandType::CommitLoad&&r.command_result==CommandResult::Accepted){
            if(state_.state()!=State::Run||stage!=ExperimentStage::SwitchingCst)r.command_result=CommandResult::RejectedState;
            else{load_start=now;stage=ExperimentStage::Rise;}
        }
        if(c.type==CommandType::Unload&&r.command_result==CommandResult::Accepted){
            if(!configured||state_.state()!=State::Run||stage==ExperimentStage::Retract||stage==ExperimentStage::Complete||stage==ExperimentStage::SwitchingCst||stage==ExperimentStage::SwitchingCsp)
                r.command_result=CommandResult::RejectedState;
            else if(raw_output){current_profile.cancel();stage=ExperimentStage::Unload;}
            else if(stage!=ExperimentStage::Ready&&stage!=ExperimentStage::Preload)r.command_result=CommandResult::RejectedState;
        }
        if(c.type==CommandType::ResumeCsp&&r.command_result==CommandResult::Accepted){
            if(state_.state()!=State::Run||stage!=ExperimentStage::SwitchingCsp||current_command!=0)r.command_result=CommandResult::RejectedState;
            else{raw_output=false;reference=position_coordinate(experiment_,origin,j.position);}
        }
        if(r.command_result==CommandResult::Accepted&&
            ((c.type==CommandType::Unload&&!raw_output)||(c.type==CommandType::ResumeCsp))){
            const double target_absolute=j.position-static_cast<double>(experiment_.direction)*experiment_.retract_distance;
            const double target=position_coordinate(experiment_,origin,target_absolute);
            if(target_absolute<experiment_.position_min||target_absolute>experiment_.position_max||
                !trajectory_.start(now,position_coordinate(experiment_,origin,j.position),target,experiment_.retract_speed,false,c.sequence)){
                r.command_result=CommandResult::RejectedArgument;
                if(c.type==CommandType::ResumeCsp)state_.latch_fault(Fault::ExperimentSafety);
            }else stage=ExperimentStage::Retract;
        }
        if(c.type==CommandType::Move)r.command_result=CommandResult::RejectedArgument;
        if(c.type==CommandType::Stop){trajectory_.stop(position_coordinate(experiment_,origin,j.position),c.sequence);}
        if(c.type==CommandType::Reset&&r.command_result==CommandResult::Accepted){stage=ExperimentStage::Idle;raw_output=false;current_command=0;current_profile.cancel();have_contact=false;experiment_start=0;reference=position_coordinate(experiment_,origin,j.position);}
        if(state_.state()==State::Run&&stage==ExperimentStage::Preload&&r.feedback.feedback_valid){
            if(std::abs(j.position-origin)>=experiment_.max_travel||static_cast<double>(now-preload_start)*1e-9>=experiment_.preload_timeout_s)state_.latch_fault(Fault::ExperimentSafety);
            else {
                if(std::abs(r.force.external)>=experiment_.contact_force_n&&!contact_since){contact_position=j.position;have_contact=true;}
                if(advance_preload(experiment_,now,r.actual_dt_ns,r.force.external,position_coordinate(experiment_,origin,j.position),contact_since,reference))stage=ExperimentStage::Ready;
                if(std::abs(absolute_position(experiment_,origin,reference)-origin)>experiment_.max_travel)state_.latch_fault(Fault::ExperimentSafety);
            }
        }
        if(state_.state()==State::Run&&raw_output){
            if(stage!=ExperimentStage::SwitchingCst&&stage!=ExperimentStage::SwitchingCsp&&stage!=ExperimentStage::Unload){
                const auto point=current_profile.sample(static_cast<double>(now-load_start)*1e-9);
                current_command=std::copysign(point.magnitude_raw,load_initial);level=point.level;
                stage=point.done?ExperimentStage::Unload:point.stage;
            }
            if(stage==ExperimentStage::Unload){
                const double step=experiment_.unload_slew_raw_s*static_cast<double>(r.actual_dt_ns)*1e-9;
                current_command=std::copysign(std::max(0.0,std::abs(current_command)-step),current_command);
                if(current_command==0)stage=ExperimentStage::SwitchingCsp;
            }
            if(std::abs(current_command)>experiment_.current_limit_raw||std::abs(j.effort)>experiment_.current_limit_raw)
                state_.latch_fault(Fault::ExperimentSafety);
        }
        if(state_.state()==State::Run&&stage==ExperimentStage::Retract){
            const auto sample=trajectory_.sample(now);reference=sample.position;r.motion_phase=sample.phase;
            if(sample.complete){if(std::abs(r.force.external)>=experiment_.contact_force_n)state_.latch_fault(Fault::ExperimentSafety);
                else{stage=ExperimentStage::Complete;have_contact=false;}}
        }
        const double absolute_reference=absolute_position(experiment_,origin,reference);
        if(config_.output_enabled&&state_.state()==State::Run&&r.feedback.feedback_valid){
            r.tracking_error=raw_output?0:j.position-absolute_reference;
            if(!raw_output&&(absolute_reference<experiment_.position_min||absolute_reference>experiment_.position_max||std::abs(r.tracking_error)>config_.maximum_tracking_error_m))state_.latch_fault(Fault::TrackingError);
            else{const auto ws=raw_output?output_->write_cst(current_command):output_->write_csp(absolute_reference);r.target_write_ok=ws==TargetWriteStatus::Ok;
                if(output_->consecutive_failures()>20||ws==TargetWriteStatus::SemaphoreError||ws==TargetWriteStatus::ReleaseError)state_.latch_fault(Fault::TargetWrite);}
        }
        if(logger_.io_error())state_.latch_fault(Fault::Logger);
        if(state_.state()==State::Fault){stage=ExperimentStage::Fault;trajectory_.stop(reference,c.sequence);current_profile.cancel();
            if(raw_output&&output_&&!raw_stop_sent){current_command=0;raw_stop_sent=output_->write_cst(0)==TargetWriteStatus::Ok;}}
        r.commanded_current_raw=raw_output?std::round(current_command):0;
        r.actual_current_raw=j.effort;r.actual_current_valid=r.feedback.feedback_valid;r.experiment_level=level;
        r.state=state_.state();r.fault=state_.fault();r.reference_position=absolute_reference;
        r.experiment_stage=stage;r.experiment_id=status.experiment_id;r.experiment_direction=configured?experiment_.direction:0;
        r.last_command_sequence=c.type==CommandType::None?status.acknowledged_sequence:c.sequence;
        r.target_write_failures=output_?output_->failures():0;r.submitted_csp_position=output_?output_->last_submitted():0;
        r.cycle_valid=r.feedback.feedback_valid&&r.state!=State::Fault;r.dropped_record_count=logger_.dropped_records();
        status.state=r.state;status.fault=r.fault;status.stage=stage;status.force=r.force;status.configured=configured;
        status.feedback_valid=r.feedback.feedback_valid;
        status.actual_position=std::isfinite(j.position)?j.position:0;status.actual_velocity=std::isfinite(j.speed)?j.speed:0;
        status.effort=std::isfinite(j.effort)?j.effort:0;status.reference_position=absolute_reference;
        status.commanded_current_raw=r.commanded_current_raw;status.level=level;
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
    if(raw_output&&output_)output_->write_cst(0);
    state_.stop();summary_.reads=adapter_.statistics();summary_.final_state=state_.state();summary_.final_fault=state_.fault();running_.store(false);
}
}
