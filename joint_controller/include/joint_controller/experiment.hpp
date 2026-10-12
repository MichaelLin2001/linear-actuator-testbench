#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace jc {
enum class ExperimentStage : unsigned { Idle, Preload, Ready, Rise, Settle, Record, Peak, Fall, Unload, Retract, Complete, Fault, SwitchingCst, SwitchingCsp };
inline const char* stage_name(ExperimentStage s) noexcept {
    constexpr const char* names[]={"IDLE","PRELOAD","READY","RISE","SETTLE","RECORD","PEAK","FALL","UNLOAD","RETRACT","COMPLETE","FAULT","SWITCHING_CST","SWITCHING_CSP"};
    return names[static_cast<unsigned>(s)];
}
struct ExperimentConfig {
    int direction{}; // Signed position direction, verified on this installation.
    bool linear{};
    bool relative_position{true};
    unsigned levels{5};
    double max_current_raw{}, transition_s{1}, settle_s{1}, record_s{3};
    double rise_s{10}, peak_hold_s{3}, fall_s{10};
    double preload_speed{}, contact_force_n{}, contact_confirm_s{0.3}, max_travel{}, preload_timeout_s{};
    double unload_slew_raw_s{}, retract_distance{}, retract_speed{};
    double current_limit_raw{}, force_limit_n{}, position_min{}, position_max{}, velocity_limit{}, contact_displacement{};
    double force_timeout_s{0.1}, experiment_timeout_s{}, heartbeat_timeout_s{1};
};
inline double position_coordinate(const ExperimentConfig& c,double origin,double absolute) noexcept {
    return c.relative_position?absolute-origin:absolute;
}
inline double absolute_position(const ExperimentConfig& c,double origin,double coordinate) noexcept {
    return c.relative_position?origin+coordinate:coordinate;
}
inline bool valid_experiment(const ExperimentConfig& c) noexcept {
    const double positive[]={c.max_current_raw,c.transition_s,c.record_s,c.rise_s,c.peak_hold_s,c.fall_s,
        c.preload_speed,c.contact_force_n,c.contact_confirm_s,c.max_travel,c.preload_timeout_s,
        c.unload_slew_raw_s,c.retract_distance,c.retract_speed,c.current_limit_raw,c.force_limit_n,
        c.velocity_limit,c.contact_displacement,c.force_timeout_s,c.experiment_timeout_s,c.heartbeat_timeout_s};
    for(double v:positive) if(!std::isfinite(v)||v<=0) return false;
    return (c.direction==1||c.direction==-1)&&c.levels>0&&c.levels<=1000&&
        std::isfinite(c.settle_s)&&c.settle_s>=0&&std::isfinite(c.position_min)&&std::isfinite(c.position_max)&&
        c.position_min<c.position_max&&c.max_current_raw<=c.current_limit_raw&&c.current_limit_raw<=32767&&
        std::floor(c.max_current_raw)==c.max_current_raw&&std::floor(c.current_limit_raw)==c.current_limit_raw&&c.contact_force_n<c.force_limit_n&&
        c.preload_speed<=c.velocity_limit&&c.retract_speed<=c.velocity_limit&&
        c.contact_displacement<=c.max_travel&&c.force_timeout_s<=1&&c.heartbeat_timeout_s<=5;
}
// Continue advancing during confirmation; hold the measured position only once confirmed.
inline bool advance_preload(const ExperimentConfig& c,std::uint64_t now,std::uint64_t dt_ns,
                            double force,double position,std::uint64_t& contact_since,double& reference) noexcept {
    if(std::abs(force)>=c.contact_force_n){
        if(!contact_since)contact_since=now;
        if(static_cast<double>(now-contact_since)*1e-9>=c.contact_confirm_s){reference=position;return true;}
    }else contact_since=0;
    reference+=static_cast<double>(c.direction)*c.preload_speed*static_cast<double>(dt_ns)*1e-9;
    return false;
}
inline bool heartbeat_fresh(std::uint64_t heartbeat,std::uint64_t checked_at,double timeout_s) noexcept {
    return heartbeat!=0&&checked_at>=heartbeat&&static_cast<double>(checked_at-heartbeat)*1e-9<=timeout_s;
}
struct CurrentPoint { double magnitude_raw{}; ExperimentStage stage{ExperimentStage::Idle}; unsigned level{}; bool done{}; };
// A pure clock-based profile: controller owns execution; no UI streaming of setpoints.
class CurrentTrajectory {
public:
    bool start(const ExperimentConfig& c,double preload_raw) noexcept {
        if(!valid_experiment(c)||!std::isfinite(preload_raw)||preload_raw<0||preload_raw>=c.max_current_raw) return false;
        c_=c;initial_=preload_raw;active_=true;return true;
    }
    void cancel() noexcept { active_=false; }
    CurrentPoint sample(double t) const noexcept {
        if(!active_||!std::isfinite(t)||t<0) return {0,ExperimentStage::Fault,0,true};
        const double span=c_.max_current_raw-initial_;
        if(c_.linear) {
            if(t<c_.rise_s) return {initial_+span*t/c_.rise_s,ExperimentStage::Rise,0,false};
            t-=c_.rise_s;
            if(t<c_.peak_hold_s) return {c_.max_current_raw,ExperimentStage::Peak,0,false};
            t-=c_.peak_hold_s;
            if(t<c_.fall_s) return {c_.max_current_raw-span*t/c_.fall_s,ExperimentStage::Fall,0,false};
        } else {
            const double block=c_.transition_s+c_.settle_s+c_.record_s;
            const double total=2*static_cast<double>(c_.levels)*block;
            if(t<total) {
                const auto k=static_cast<unsigned>(t/block);
                const bool up=k<c_.levels;
                const unsigned from=up?k:2*c_.levels-k;
                const unsigned to=up?from+1:from-1;
                const double a=initial_+span*static_cast<double>(from)/c_.levels;
                const double b=initial_+span*static_cast<double>(to)/c_.levels;
                t-=static_cast<double>(k)*block;
                if(t<c_.transition_s) return {a+(b-a)*t/c_.transition_s,up?ExperimentStage::Rise:ExperimentStage::Fall,to,false};
                t-=c_.transition_s;
                return {b,t<c_.settle_s?ExperimentStage::Settle:ExperimentStage::Record,to,false};
            }
        }
        return {initial_,ExperimentStage::Complete,0,true};
    }
    double duration_s() const noexcept {
        return c_.linear?c_.rise_s+c_.peak_hold_s+c_.fall_s:
            2*static_cast<double>(c_.levels)*(c_.transition_s+c_.settle_s+c_.record_s);
    }
private:
    ExperimentConfig c_{}; double initial_{}; bool active_{};
};
inline bool safe_measurement(const ExperimentConfig& c,double position,double velocity,double force,
                             double force_age_s,bool force_valid) noexcept {
    return std::isfinite(position)&&std::isfinite(velocity)&&std::isfinite(force)&&std::isfinite(force_age_s)&&
        position>=c.position_min&&position<=c.position_max&&std::abs(velocity)<=c.velocity_limit&&
        std::abs(force)<c.force_limit_n&&force_valid&&force_age_s>=0&&force_age_s<c.force_timeout_s;
}
} // namespace jc
