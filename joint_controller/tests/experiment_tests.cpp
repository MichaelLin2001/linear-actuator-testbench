#include "joint_controller/experiment.hpp"
#include "joint_controller/trajectory_generator.hpp"
#if defined(__linux__)
#include "joint_controller/jc_target_adapter.hpp"
#endif
#include "joint_controller/force_processor.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#define CHECK(x) do { if(!(x)){std::fprintf(stderr,"failed at line %d: %s\n",__LINE__,#x);std::exit(1);} } while(false)
jc::ExperimentConfig config(){
    jc::ExperimentConfig c{};c.direction=1;c.max_current_raw=1;c.current_limit_raw=2;
    c.preload_speed=.0001;c.contact_force_n=5;c.max_travel=.002;c.preload_timeout_s=30;
    c.unload_slew_raw_s=.1;c.retract_distance=.001;c.retract_speed=.0001;
    c.force_limit_n=100;c.position_min=0;c.position_max=.05;c.velocity_limit=.001;
    c.contact_displacement=.001;c.experiment_timeout_s=300;return c;
}
int main(){
    {
        const std::uint64_t wake=1'000'000'000ULL,hb=wake+17;
        CHECK(jc::heartbeat_fresh(hb,hb+100,1));
        CHECK(jc::heartbeat_fresh(hb,hb+1'000'000'000ULL,1));
        CHECK(!jc::heartbeat_fresh(hb,hb+1'000'000'001ULL,1));
        CHECK(!jc::heartbeat_fresh(0,hb,1));CHECK(!jc::heartbeat_fresh(hb,wake,1));
        for(int direction:{-1,1}){
            auto p=config();p.direction=direction;p.contact_confirm_s=.2;
            std::uint64_t since=0;double reference=.02;
            for(unsigned i=0;i<200;++i){
                const double previous=reference;
                CHECK(!jc::advance_preload(p,wake+i*1'000'000ULL,1'000'000,5,.02,since,reference));
                CHECK((reference-previous)*direction>0);
            }
            CHECK(jc::advance_preload(p,wake+200'000'000ULL,1'000'000,5,.021,since,reference));
            CHECK(reference==.021);
            // A dip restarts confirmation but never pauses the advancing reference.
            since=0;reference=.02;
            CHECK(!jc::advance_preload(p,wake,1'000'000,-6,.02,since,reference));
            const double before_dip=reference;
            CHECK(!jc::advance_preload(p,wake+190'000'000ULL,1'000'000,4,.02,since,reference));
            CHECK(since==0&&(reference-before_dip)*direction>0);
            CHECK(!jc::advance_preload(p,wake+200'000'000ULL,1'000'000,-6,.02,since,reference));
            CHECK(!jc::advance_preload(p,wake+399'000'000ULL,1'000'000,-6,.02,since,reference));
            CHECK(jc::advance_preload(p,wake+400'000'000ULL,1'000'000,-6,.022,since,reference));
            CHECK(reference==.022);
        }
    }

    for(int direction:{-1,1}) {
        auto relative=config(),absolute=relative;absolute.relative_position=false;
        const double origin=.023,contact=origin+static_cast<double>(direction)*.0002;
        double qr=jc::position_coordinate(relative,origin,origin),qa=origin;
        for(unsigned i=0;i<10;++i){
            qr+=static_cast<double>(direction)*.00001;qa+=static_cast<double>(direction)*.00001;
            CHECK(std::abs(jc::absolute_position(relative,origin,qr)-qa)<1e-12);
        }
        qr=jc::position_coordinate(relative,origin,contact);
        for(unsigned i=0;i<10;++i)CHECK(std::abs(jc::absolute_position(relative,origin,qr)-contact)<1e-12);
        const double target=contact-static_cast<double>(direction)*.001;
        jc::TrajectoryGenerator tr,ta;
        CHECK(tr.start(1,jc::position_coordinate(relative,origin,contact),jc::position_coordinate(relative,origin,target),.0001,false,1));
        CHECK(ta.start(1,contact,target,.0001,false,1));
        for(std::uint64_t ns=1;ns<=20'000'000'001ULL;ns+=100'000'000ULL){
            const auto sr=tr.sample(ns),sa=ta.sample(ns);
            CHECK(std::abs(jc::absolute_position(relative,origin,sr.position)-sa.position)<1e-12);
            CHECK(sr.phase==sa.phase&&sr.complete==sa.complete);
        }
        CHECK(jc::absolute_position(absolute,origin,contact)==contact);
    }
    auto c=config();jc::CurrentTrajectory t;CHECK(t.start(c,.2));
    const double block=c.transition_s+c.settle_s+c.record_s;
    CHECK(std::abs(t.sample(0).magnitude_raw-.2)<1e-12);
    unsigned peaks=0;
    for(unsigned i=0;i<2*c.levels;++i){
        const auto a=t.sample(static_cast<double>(i)*block+c.transition_s+c.settle_s+.1);
        CHECK(a.stage==jc::ExperimentStage::Record);CHECK(a.magnitude_raw>=.2&&a.magnitude_raw<=1);
        if(std::abs(a.magnitude_raw-1)<1e-12)++peaks;
    }
    CHECK(peaks==1);CHECK(t.sample(t.duration_s()).done);CHECK(t.sample(t.duration_s()).magnitude_raw==.2);
    CHECK(t.sample(block*c.levels+.5).stage==jc::ExperimentStage::Fall);
    c.linear=true;CHECK(t.start(c,.2));CHECK(t.sample(10).magnitude_raw==1);
    CHECK(t.sample(13).stage==jc::ExperimentStage::Fall);CHECK(t.sample(23).done);
    for(double time=0;time<=23;time+=.013){auto a=t.sample(time);CHECK(a.magnitude_raw>=.2-1e-12&&a.magnitude_raw<=1+1e-12);}
    t.cancel();CHECK(t.sample(1).magnitude_raw==0&&t.sample(1).stage==jc::ExperimentStage::Fault);
    CHECK(!t.start(c,1));c.max_current_raw=3;CHECK(!t.start(c,0));c=config();
    CHECK(jc::safe_measurement(c,.02,0,10,.01,true));
    CHECK(!jc::safe_measurement(c,.02,0,100,.01,true));
    CHECK(!jc::safe_measurement(c,.02,0,10,.1,true));
    CHECK(!jc::safe_measurement(c,.02,0,10,.01,false));
    CHECK(!jc::safe_measurement(c,.06,0,10,.01,true));
    CHECK(!jc::safe_measurement(c,.02,.01,10,.01,true));
    CHECK(!jc::safe_measurement(c,.02,0,std::numeric_limits<double>::quiet_NaN(),.01,true));
    c.max_current_raw=.5;CHECK(!jc::valid_experiment(c));c=config();
    c.current_limit_raw=32768;CHECK(!jc::valid_experiment(c));c=config();
    c.direction=-1;CHECK(jc::valid_experiment(c));c.levels=0;CHECK(!jc::valid_experiment(c));
#if defined(__linux__)
    jc::RawTargetArray slots{};slots.fill(jc::RawTarget{0x5a,0x5a,{0x5a,0x5a},90.0F,90.0F,90.0F});const auto before=slots;
    slots[jc::kJ3]=jc::csp_target(slots[jc::kJ3],.012);
    CHECK(slots[3].target_position==12);CHECK(slots[3].run_mode==8);CHECK(slots[3].target_effort==0);
    CHECK(slots[3].motor_type==before[3].motor_type&&std::memcmp(slots[3].reserved,before[3].reserved,2)==0);
    for(std::size_t i=0;i<slots.size();++i)if(i!=3)CHECK(std::memcmp(&slots[i],&before[i],sizeof(slots[i]))==0);
    auto raw_target=jc::cst_target(slots[3],-7.6);
    CHECK(raw_target.target_effort==-8&&raw_target.run_mode==10&&raw_target.target_position==12);
    CHECK(raw_target.motor_type==slots[3].motor_type);
    raw_target.reserved[0]=255;raw_target.reserved[1]=255;
    raw_target=jc::cst_target(raw_target,0);CHECK(raw_target.reserved[0]==0&&raw_target.reserved[1]==0);
#endif
    jc::ForceProcessor f;const std::uint64_t start=1'000'000'000ULL;f.begin_zero(start);
    for(unsigned i=0;i<=100;++i)f.update({12,static_cast<std::uint16_t>(i),start+static_cast<std::uint64_t>(i)*10'000'000ULL,true},start+static_cast<std::uint64_t>(i)*10'000'000ULL);
    CHECK(f.value().zeroed&&f.value().zero==12);
    auto sample=jc::ForceSample{17,101,start+1'010'000'000ULL,true};auto a=f.update(sample,sample.timestamp_ns);
    CHECK(a.external==5&&a.new_sample);CHECK(!f.update(sample,sample.timestamp_ns+1).new_sample);
    CHECK(!f.update(sample,sample.timestamp_ns+100'000'000ULL).valid);
    std::puts("experiment tests passed");
}
