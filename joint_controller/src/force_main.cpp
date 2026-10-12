#include "joint_controller/force_shm.hpp"
#include "joint_controller/control_loop.hpp"
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
namespace {
volatile sig_atomic_t stopped=0;
void stop(int){stopped=1;}
struct Terminal {
    termios original{}; bool changed{};
    ~Terminal(){if(changed)tcsetattr(STDIN_FILENO,TCSANOW,&original);}
    bool start(){
        if(!isatty(STDIN_FILENO)||tcgetattr(STDIN_FILENO,&original)!=0)return false;
        auto t=original;t.c_lflag&=static_cast<tcflag_t>(~(ICANON|ECHO));
        changed=tcsetattr(STDIN_FILENO,TCSANOW,&t)==0;return changed;
    }
};
}
int main(int argc,char** argv){
    std::string device="/dev/ttyUSB0";unsigned baud=115200;
    try{
        for(int i=1;i<argc;++i){
            const std::string option=argv[i];if(i+1>=argc)throw std::runtime_error("missing value");
            if(option=="--device")device=argv[++i];
            else if(option=="--baud")baud=static_cast<unsigned>(std::stoul(argv[++i]));
            else throw std::runtime_error("use --device PATH --baud RATE");
        }
        signal(SIGINT,stop);signal(SIGTERM,stop);
        std::filesystem::create_directories("/tmp/j3-testbench");
        jc::ForceShm memory;std::string error;
        if(!memory.attach(true,error))throw std::runtime_error(error);
        jc::ProcessedForce value{};const auto session=jc::monotonic_now_ns();value.session_id=session;
        if(!memory.publish(value))throw std::runtime_error("cannot initialize force snapshot");
        jc::ForceSensorReader sensor;
        if(!sensor.start(device,baud))throw std::runtime_error("cannot open force sensor");
        Terminal terminal;if(!terminal.start())throw std::runtime_error("force_sensor requires an interactive terminal");
        jc::ForceProcessor processor;std::uint64_t sequence=0,version=0,last_print=0;
        std::cout<<"无载且电缸静止时按 z 调零；q 退出。实验中调零会中止本轮实验。\n";
        while(!stopped){
            pollfd input{STDIN_FILENO,POLLIN,0};poll(&input,1,1);
            const auto now=jc::monotonic_now_ns();
            if(input.revents&POLLIN){char key=0;if(read(STDIN_FILENO,&key,1)==1){
                if(key=='q')break;
                if(key=='z'){++version;processor.begin_zero(now);}
            }}
            value=processor.update(sensor.get_latest(),now);
            if(value.new_sample)++sequence;
            value.session_id=session;value.sample_sequence=sequence;value.zero_version=version;
            value.valid=value.valid&&sensor.running();
            if(!memory.publish(value)){
                if(now-last_print>=100'000'000ULL)std::cerr<<"\n力共享内存忙，本次发布跳过\n";
            }
            if(now-last_print>=100'000'000ULL){
                static const char* states[]={"未调零","调零中","已调零","调零失败"};
                printf("\r\033[2Kraw=%.3f N zero=%.3f N F=%.3f N valid=%d %s samples=%llu",
                    value.raw,value.zero,value.external,value.valid,states[static_cast<unsigned>(value.zero_state)],
                    static_cast<unsigned long long>(value.zero_samples));fflush(stdout);last_print=now;
            }
            if(!sensor.running()){std::cerr<<"\n力传串口断开\n";break;}
        }
        value.valid=false;value.zeroed=false;
        for(unsigned attempt=0;attempt<20&&!memory.publish(value);++attempt)usleep(1000);
        sensor.stop();std::cout<<'\n';return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
