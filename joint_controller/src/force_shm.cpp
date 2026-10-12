#include "joint_controller/force_shm.hpp"
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <time.h>
namespace jc {
ForceShm::~ForceShm(){
    if(address_)shmdt(address_);
    if(semaphore_!=SEM_FAILED)sem_close(semaphore_);
    if(owner_fd_>=0)close(owner_fd_);
}
bool ForceShm::attach(bool writer,std::string& error,const std::string& path,const std::string& name){
    if(address_){error="force memory already attached";return false;}
    writer_=writer;
    if(writer){
        owner_fd_=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);
        if(owner_fd_<0||flock(owner_fd_,LOCK_EX|LOCK_NB)!=0){error="force publisher already running or key file unavailable";return false;}
    }
    const key_t key=ftok(path.c_str(),3);
    if(key==static_cast<key_t>(-1)){error="force key unavailable; start force_sensor first";return false;}
    const int id=shmget(key,sizeof(ProcessedForce),writer?IPC_CREAT|0600:0);
    shmid_ds info{};
    if(id<0||shmctl(id,IPC_STAT,&info)!=0||info.shm_segsz!=sizeof(ProcessedForce)){
        error="force shared memory missing or incompatible";return false;
    }
    void* mapped=shmat(id,nullptr,writer?0:SHM_RDONLY);
    if(mapped==reinterpret_cast<void*>(-1)){error=std::strerror(errno);return false;}
    semaphore_=sem_open(name.c_str(),writer?O_CREAT:0,0600,1);
    if(semaphore_==SEM_FAILED){shmdt(mapped);error=std::strerror(errno);return false;}
    address_=mapped;
    // Do not reset a lock left by a crashed writer: fail explicitly rather than race a reader.
    if(writer&&!publish(ProcessedForce{})){error="force lock unavailable; confirm all readers stopped before recovery";return false;}
    return true;
}
bool ForceShm::publish(const ProcessedForce& value) noexcept {
    if(!writer_||!address_||sem_trywait(semaphore_)!=0)return false;
    std::memcpy(address_,&value,sizeof(value));
    return sem_post(semaphore_)==0;
}
ProcessedForce ForceShm::read(std::uint64_t& read_time_ns) noexcept {
    if(address_&&sem_trywait(semaphore_)==0){
        ProcessedForce copy{};std::memcpy(&copy,address_,sizeof(copy));
        if(sem_post(semaphore_)==0)latest_=copy;
        else latest_.valid=false;
    }
    timespec time{};
    read_time_ns=clock_gettime(CLOCK_MONOTONIC,&time)==0
        ? static_cast<std::uint64_t>(time.tv_sec)*1'000'000'000ULL+static_cast<std::uint64_t>(time.tv_nsec) : 0;
    const auto now=read_time_ns;
    auto value=latest_;
    value.valid=value.valid&&value.session_id!=0&&value.timestamp_ns!=0&&now>=value.timestamp_ns&&
        now-value.timestamp_ns<ForceProcessor::timeout_ns&&std::isfinite(value.raw)&&
        std::isfinite(value.zero)&&std::isfinite(value.external);
    value.new_sample=value.valid&&(value.session_id!=previous_session_||value.sample_sequence!=previous_sample_);
    if(value.new_sample){previous_session_=value.session_id;previous_sample_=value.sample_sequence;}
    return value;
}
}
