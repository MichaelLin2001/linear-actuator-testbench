#include "joint_controller/force_processor.hpp"
#include <algorithm>
#include <limits>
namespace jc {
void ForceProcessor::begin_zero(std::uint64_t now) noexcept {
    zero_started_=now;first_zero_sample_=0;sum_=0;
    minimum_=std::numeric_limits<double>::infinity();maximum_=-minimum_;
    value_.zero_samples=0;value_.zero_state=ZeroState::Collecting;
    value_.zeroed=false;value_.external=0;
}
void ForceProcessor::cancel_zero() noexcept {
    if(value_.zero_state==ZeroState::Collecting)value_.zero_state=ZeroState::Failed;
}
ProcessedForce ForceProcessor::update(const ForceSample& s,std::uint64_t now) noexcept {
    value_.new_sample=false;
    if(s.valid&&std::isfinite(s.force_n)&&s.timestamp_ns!=0&&s.timestamp_ns<=now&&
       s.timestamp_ns>value_.timestamp_ns&&
       (value_.timestamp_ns==0||s.package_no!=value_.package_no)){
        value_.new_sample=true;value_.raw=s.force_n;value_.package_no=s.package_no;value_.timestamp_ns=s.timestamp_ns;
        if(value_.zero_state==ZeroState::Collecting&&s.timestamp_ns>=zero_started_){
            if(first_zero_sample_==0)first_zero_sample_=s.timestamp_ns;
            sum_+=s.force_n;++value_.zero_samples;
            minimum_=std::min(minimum_,s.force_n);maximum_=std::max(maximum_,s.force_n);
            if(s.timestamp_ns-first_zero_sample_>=1'000'000'000ULL){
                if(value_.zero_samples>=80&&maximum_-minimum_<=100.0){
                    value_.zero=sum_/static_cast<double>(value_.zero_samples);
                    value_.zeroed=true;value_.zero_state=ZeroState::Complete;
                }else value_.zero_state=ZeroState::Failed;
            }
        }
    }
    value_.valid=value_.timestamp_ns!=0&&now>=value_.timestamp_ns&&now-value_.timestamp_ns<timeout_ns;
    if(value_.zero_state==ZeroState::Collecting&&
       ((!value_.valid&&now-zero_started_>timeout_ns)||now-zero_started_>2'000'000'000ULL))cancel_zero();
    value_.external=value_.zeroed?value_.raw-value_.zero:0.0;
    return value_;
}
}
