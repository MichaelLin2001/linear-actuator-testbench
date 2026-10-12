#include "joint_controller/force_shm.hpp"
#include "joint_controller/data_logger.hpp"
#include "joint_controller/control_loop.hpp"
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/wait.h>
#include <unistd.h>
#define CHECK(x) do{if(!(x)){std::cerr<<__LINE__<<": "<<#x<<'\n';std::exit(1);}}while(false)
int main(){
    const std::string dir="/tmp/j3-process-tests-"+std::to_string(getpid());
    std::filesystem::create_directory(dir);
    const auto keyfile=dir+"/force";const auto name="/j3_force_test_"+std::to_string(getpid());
    std::string error;
    {
        jc::ForceShm writer,reader,second;
        CHECK(writer.attach(true,error,keyfile,name));
        CHECK(!second.attach(true,error,keyfile,name));
        CHECK(reader.attach(false,error,keyfile,name));
        std::uint64_t read_time=0;
        CHECK(!reader.read(read_time).valid);
        const auto pid=fork();CHECK(pid>=0);
        if(pid==0){
            jc::ProcessedForce value{};value.raw=12;value.zero=10;value.external=2;
            value.timestamp_ns=jc::monotonic_now_ns();value.session_id=123;value.sample_sequence=1;
            value.zero_version=1;value.zeroed=true;value.valid=true;value.zero_state=jc::ZeroState::Complete;
            _exit(writer.publish(value)?0:2);
        }
        int status=0;CHECK(waitpid(pid,&status,0)==pid&&WIFEXITED(status)&&WEXITSTATUS(status)==0);
        auto value=reader.read(read_time);
        CHECK(value.valid&&value.new_sample&&value.raw==12&&value.zero==10&&value.external==2);
        CHECK(!reader.read(read_time).new_sample);
        const auto baseline=value;
        CHECK(jc::same_force_zero(baseline,value));
        sem_t* lock=sem_open(name.c_str(),0);CHECK(lock!=SEM_FAILED&&sem_wait(lock)==0);
        const auto start=jc::monotonic_now_ns();
        CHECK(reader.read(read_time).valid); // contention reuses only a still-fresh snapshot
        CHECK(jc::monotonic_now_ns()-start<20'000'000ULL);
        CHECK(sem_post(lock)==0);sem_close(lock);
        // A publication after cycle wakeup is fresh when the snapshot is read.
        const auto cycle_wakeup=jc::monotonic_now_ns();
        usleep(1000);
        value=baseline;value.timestamp_ns=jc::monotonic_now_ns();++value.sample_sequence;
        CHECK(value.timestamp_ns>cycle_wakeup&&writer.publish(value));
        read_time=cycle_wakeup;
        const auto fresh=reader.read(read_time);
        CHECK(fresh.valid&&fresh.new_sample&&read_time>=fresh.timestamp_ns);
        jc::ExperimentConfig config{};config.position_min=0;config.position_max=1;
        config.velocity_limit=1;config.force_limit_n=100;config.force_timeout_s=.1;
        CHECK(jc::safe_measurement(config,.5,0,fresh.external,
            static_cast<double>(read_time-fresh.timestamp_ns)*1e-9,fresh.valid));
        // Truly expired and future-dated snapshots remain invalid, including cached reads.
        value.timestamp_ns=jc::monotonic_now_ns()-jc::ForceProcessor::timeout_ns;
        CHECK(writer.publish(value));CHECK(!reader.read(read_time).valid);
        lock=sem_open(name.c_str(),0);CHECK(lock!=SEM_FAILED&&sem_wait(lock)==0);
        CHECK(!reader.read(read_time).valid);CHECK(sem_post(lock)==0);sem_close(lock);
        value.timestamp_ns=jc::monotonic_now_ns()+10'000'000'000ULL;
        CHECK(writer.publish(value));CHECK(!reader.read(read_time).valid);
        value=baseline;
        value.zeroed=false;value.zero_state=jc::ZeroState::Collecting;++value.zero_version;
        CHECK(writer.publish(value));CHECK(!jc::same_force_zero(baseline,reader.read(read_time)));
        value=baseline;value.session_id=456;CHECK(writer.publish(value));
        const auto restart=reader.read(read_time);
        CHECK(restart.new_sample&&!jc::same_force_zero(baseline,restart));
        value.valid=false;CHECK(writer.publish(value));CHECK(!reader.read(read_time).valid);
    }
    const int id=shmget(ftok(keyfile.c_str(),3),sizeof(jc::ProcessedForce),0);CHECK(id>=0);
    CHECK(shmctl(id,IPC_RMID,nullptr)==0);sem_unlink(name.c_str());
    {
        jc::ForceProcessor processor;
        const auto now=jc::monotonic_now_ns();processor.begin_zero(now);
        jc::ProcessedForce value{};
        for(unsigned i=0;i<=100;++i)value=processor.update({12,static_cast<std::uint16_t>(i),now+i*10'000'000ULL,true},now+i*10'000'000ULL);
        CHECK(value.zeroed&&value.zero==12&&value.external==0);
        value=processor.update({15,101,now+1'010'000'000ULL,true},now+1'010'000'000ULL);
        CHECK(value.external==3);
    }
    {
        jc::DataLogger logger;jc::LoopConfig config{};
        CHECK(logger.reserve_record()==nullptr);CHECK(!std::filesystem::exists(dir+"/one.csv"));
        for(unsigned run=0;run<2;++run){
            const auto path=dir+"/"+std::to_string(run)+".csv";
            CHECK(logger.start(path,config,error));
            for(unsigned i=0;i<3;++i){auto* r=logger.reserve_record();CHECK(r);*r={};
                r->actual_wakeup_ns=1000+i*1'000'000ULL;
                r->force.timestamp_ns=r->actual_wakeup_ns+17;r->force_read_timestamp_ns=r->actual_wakeup_ns+100;
                r->force.raw=12;r->force.zero=10;r->force.external=2;
                r->force.session_id=123;r->force.zero_version=1;r->force.sample_sequence=i+1;
                logger.publish_record();}
            jc::LoopSummary summary{};summary.final_state=jc::State::Fault;summary.final_fault=jc::Fault::ForceSensor;
            logger.finish(summary);CHECK(logger.reserve_record()==nullptr);
            std::ifstream file(path);std::string line;unsigned lines=0;
            while(std::getline(file,line)){++lines;
                if(lines==1){CHECK(line.find("j3_current_raw_6077")!=std::string::npos);CHECK(line.find("j3_effort,")==std::string::npos);}
                if(lines>1)CHECK(line.find(",83,"+std::to_string(lines-1)+",123,1")!=std::string::npos);}
            CHECK(lines==4);
            std::ifstream summary_file(path+".summary.csv");
            const std::string text((std::istreambuf_iterator<char>(summary_file)),{});
            CHECK(text.find("FORCE_SENSOR_INVALID_OR_ZERO_CHANGED")!=std::string::npos);
            CHECK(text.find("cycles,3\n")!=std::string::npos);
            CHECK(!logger.start(path,config,error)); // no overwrite
        }
        // Stop while a producer has reserved a row: finish must wait for its publish.
        CHECK(logger.start(dir+"/concurrent.csv",config,error));
        auto* row=logger.reserve_record();CHECK(row);*row={};row->actual_wakeup_ns=1000;
        std::atomic<bool> done{false};std::thread finish([&]{logger.finish(jc::LoopSummary{});done.store(true);});
        usleep(5000);CHECK(!done.load());logger.publish_record();finish.join();CHECK(done.load());
    }
    {
        // Exercise the production serial parser with a pseudo-terminal, never hardware.
        const int master=posix_openpt(O_RDWR|O_NOCTTY);CHECK(master>=0);
        CHECK(grantpt(master)==0&&unlockpt(master)==0);
        jc::ForceSensorReader sensor;CHECK(sensor.start(ptsname(master),115200));
        const unsigned char frame[]={0xaa,0x55,0,10,0,7,0,0,0x40,0x41}; // 12 N, little-endian float
        CHECK(write(master,frame,3)==3);usleep(1000);
        CHECK(write(master,frame+3,sizeof(frame)-3)==7);
        jc::ForceSample sample{};
        for(unsigned i=0;i<100;++i){sample=sensor.get_latest();if(sample.valid)break;usleep(1000);}
        CHECK(sample.valid&&sample.force_n==12&&sample.package_no==7);
        jc::ForceProcessor processor;
        CHECK(processor.update(sample,jc::monotonic_now_ns()).valid);
        close(master);
        for(unsigned i=0;i<100&&sensor.running();++i)usleep(1000);
        CHECK(!sensor.running());
        CHECK(!processor.update(sample,sample.timestamp_ns+jc::ForceProcessor::timeout_ns).valid);
        sensor.stop();
    }
    std::filesystem::remove_all(dir);std::cout<<"process and recording contracts passed\n";
}
