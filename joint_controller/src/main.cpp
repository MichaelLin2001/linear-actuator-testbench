#include <filesystem>
#include "joint_controller/control_loop.hpp"
#include "joint_controller/driver_manager.hpp"
#include <chrono>
#include <cstdio>
#include <csignal>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <poll.h>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
namespace {
volatile std::sig_atomic_t interrupted=0;
extern "C" void signal_handler(int){interrupted=1;}
struct SafetyGuard {
    std::atomic<bool>& done; std::thread& thread;
    ~SafetyGuard(){done.store(true);if(thread.joinable())thread.join();}
};
std::string quote(const std::string& s){
    std::string out="\"";for(char ch:s){if(ch=='\"'||ch=='\\')out+='\\';if(ch=='\n')out+="\\n";else if(static_cast<unsigned char>(ch)>=32)out+=ch;}return out+'\"';
}
std::string string_field(const std::string& s,const char* key){
    const std::regex pattern(std::string("\"")+key+"\"\\s*:\\s*\"([^\"\\\\]*)\"");std::smatch m;
    return std::regex_search(s,m,pattern)?m[1].str():std::string{};
}
double number_field(const std::string& s,const char* key){
    const std::regex pattern(std::string("\"")+key+"\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]+)?)(?:[eE][+-]?[0-9]+)?)(?=\\s*[,}])");std::smatch m;
    if(!std::regex_search(s,m,pattern))return std::numeric_limits<double>::quiet_NaN();
    try{return std::stod(m[1].str());}catch(...){return std::numeric_limits<double>::quiet_NaN();}
}
jc::ExperimentConfig config_from(const std::string& input){
    jc::ExperimentConfig c{};
    const double direction=number_field(input,"direction"),levels=number_field(input,"levels");
    const auto mode=string_field(input,"mode");
    const auto position_mode=string_field(input,"position_mode");
    if(input.find("\"position_mode\"")!=std::string::npos&&position_mode!="relative"&&position_mode!="absolute")
        throw std::runtime_error("position_mode must be relative or absolute");
    c.relative_position=position_mode!="absolute";
    if((direction!=1&&direction!=-1)||!std::isfinite(levels)||levels<1||levels>1000||std::floor(levels)!=levels||
        (mode!="linear"&&mode!="staircase"))throw std::runtime_error("invalid direction, levels or mode");
    c.direction=static_cast<int>(direction);c.levels=static_cast<unsigned>(levels);c.linear=mode=="linear";
#define PARAM(name) c.name=number_field(input,#name)
    PARAM(max_current_raw);PARAM(transition_s);PARAM(settle_s);PARAM(record_s);PARAM(rise_s);PARAM(peak_hold_s);PARAM(fall_s);
    PARAM(preload_speed);PARAM(contact_force_n);PARAM(contact_confirm_s);PARAM(max_travel);PARAM(preload_timeout_s);
    PARAM(unload_slew_raw_s);PARAM(retract_distance);PARAM(retract_speed);PARAM(current_limit_raw);PARAM(force_limit_n);
    PARAM(position_min);PARAM(position_max);PARAM(velocity_limit);PARAM(contact_displacement);PARAM(force_timeout_s);
    PARAM(experiment_timeout_s);PARAM(heartbeat_timeout_s);
#undef PARAM
    if(!jc::valid_experiment(c))throw std::runtime_error("unsafe or missing experiment parameters");
    return c;
}
int listen_socket(const std::string& path,int type){
    if(path.size()>=sizeof(sockaddr_un::sun_path))throw std::runtime_error("socket path too long");
    // Never unlink another running controller's socket.
    struct stat info{};if(lstat(path.c_str(),&info)==0)throw std::runtime_error("socket exists; confirm old controller stopped before removing it");
    const int fd=socket(AF_UNIX,type|SOCK_CLOEXEC,0);if(fd<0)throw std::runtime_error("socket failed");
    sockaddr_un address{};address.sun_family=AF_UNIX;std::strncpy(address.sun_path,path.c_str(),sizeof(address.sun_path)-1);
    if(bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0||(type==SOCK_STREAM&&listen(fd,8)!=0)){
        close(fd);throw std::runtime_error("bind/listen failed");}
    chmod(path.c_str(),0600);return fd;
}
bool submit(jc::CommandManager& commands,jc::ControlLoop& loop,jc::Command command,std::string& error){
    if(!commands.submit(command,jc::monotonic_now_ns())){error="command queue full";return false;}
    const auto id=commands.last_submitted_sequence();
    for(unsigned i=0;i<200;++i){jc::ControlStatus s{};
        if(loop.status(s)&&s.acknowledged_sequence==id){
            if(s.command_result==jc::CommandResult::Accepted)return true;
            error="controller rejected command; check state, zero, feedback and limits";return false;}
        std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    error="command acknowledgement timeout";return false;
}
std::string status_json(const jc::ControlStatus& s,bool disabled){
    constexpr const char* states[]={"INIT","IDLE","ARMED","RUN","STOP","FAULT"};
    const auto now=jc::monotonic_now_ns();
    std::ostringstream out;out.precision(12);
    out<<"\"state\":"<<quote(states[static_cast<unsigned>(s.state)])<<",\"fault\":"<<static_cast<unsigned>(s.fault)
       <<",\"stage\":"<<quote(jc::stage_name(s.stage))<<",\"configured\":"<<(s.configured?"true":"false")
       <<",\"feedback_valid\":"<<(s.feedback_valid?"true":"false")<<",\"stationary\":"<<(s.stationary?"true":"false")<<",\"position_mm\":"<<s.actual_position*1000
       <<",\"velocity_mm_s\":"<<s.actual_velocity*1000<<",\"reference_mm\":"<<s.reference_position*1000
       <<",\"current_raw_6077\":"<<s.effort<<",\"force_raw_n\":"<<s.force.raw<<",\"force_n\":"<<s.force.external
       <<",\"force_valid\":"<<(s.force.valid?"true":"false")<<",\"zeroed\":"<<(s.force.zeroed?"true":"false")
       <<",\"zero_state\":"<<static_cast<unsigned>(s.force.zero_state)<<",\"zero_sequence\":"<<s.zero_sequence
       <<",\"force_age_ms\":"<<(s.force.timestamp_ns&&now>=s.force.timestamp_ns?static_cast<double>(now-s.force.timestamp_ns)*1e-6:-1)
       <<",\"force_session_id\":"<<s.force.session_id<<",\"force_zero_version\":"<<s.force.zero_version
       <<",\"experiment_id\":"<<s.experiment_id<<",\"actual_current_raw\":"<<s.effort<<",\"actual_current_valid\":"<<(s.feedback_valid?"true":"false")
       <<",\"commanded_current_raw\":"<<s.commanded_current_raw<<",\"level\":"<<s.level<<",\"current_output_locked\":false,\"driver_disabled_confirmed\":"<<(disabled?"true":"false");
    return out.str();
}
}
int main(int argc,char** argv){
    std::string socket_path="/tmp/j3-testbench/controller.sock",csv="/tmp/j3-testbench/joint_controller.csv";
    bool output=false;std::string preview_path;double preview_initial=0;int server=-1,urgent=-1;std::thread safety;
    std::atomic<bool> safety_done{};
    try{
        jc::LoopConfig config{};config.state.active_mask=jc::kJ3Mask;config.duration_ns=86'400'000'000'000ULL;
        for(int i=1;i<argc;++i){const std::string option=argv[i];
            if(option=="--enable-output"){output=true;continue;}
            if(option=="--serve")continue;
            if(i+1>=argc)throw std::runtime_error("missing option value");
            const std::string value=argv[++i];
            if(option=="--preview")preview_path=value;else if(option=="--preview-initial-raw")preview_initial=std::stod(value);
            else if(option=="--socket")socket_path=value;else if(option=="--csv")csv=value;
            else throw std::runtime_error("unknown option: "+option);
        }
        if(!preview_path.empty()){
            std::string input;std::getline(std::cin,input);jc::CurrentTrajectory profile;
            if(!profile.start(config_from(input),preview_initial)||profile.duration_s()>86400)throw std::runtime_error("invalid offline profile");
            std::FILE* file=std::fopen(preview_path.c_str(),"wx");if(!file)throw std::runtime_error("preview output exists or cannot be created");
            std::fputs("time_s,hypothetical_current_magnitude_raw,stage,level\n",file);
            const auto samples=static_cast<std::uint64_t>(std::ceil(profile.duration_s()*100));
            for(std::uint64_t i=0;i<=samples;++i){const double t=std::min(static_cast<double>(i)*.01,profile.duration_s());const auto v=profile.sample(t);
                std::fprintf(file,"%.9g,%.9g,%s,%u\n",t,v.magnitude_raw,jc::stage_name(v.stage),v.level);}
            const bool failed=std::ferror(file)!=0;const int closed=std::fclose(file);
            if(failed||closed!=0)throw std::runtime_error("preview write failed");
            std::cout<<"OFFLINE ONLY, duration_s="<<profile.duration_s()<<'\n';return 0;
        }
        config.output_enabled=output;
        signal(SIGINT,signal_handler);signal(SIGTERM,signal_handler);signal(SIGPIPE,SIG_IGN);
        std::filesystem::create_directories("/tmp/j3-testbench");
        server=listen_socket(socket_path,SOCK_STREAM);urgent=listen_socket(socket_path+".urgent",SOCK_DGRAM);
        jc::ForceShm sensor;std::string force_error;
        if(!sensor.attach(false,force_error))throw std::runtime_error(force_error);
        jc::DriverManager driver;std::string error;
        if(output&&!driver.start(error))throw std::runtime_error(error);
        const bool initial_disabled=!output||driver.disable(error).state_confirmed;
        if(!initial_disabled)throw std::runtime_error("startup disable not confirmed: "+error);
        jc::JcShmAdapter feedback;if(!feedback.attach(error))throw std::runtime_error(error);
        jc::JcTargetAdapter target;std::unique_ptr<jc::OutputManager> manager;
        if(output){if(!target.attach(error))throw std::runtime_error(error);manager=std::make_unique<jc::OutputManager>(target);}
        jc::CommandManager commands;jc::DataLogger logger;
        jc::ControlLoop loop(feedback,commands,logger,config,manager.get(),&sensor);if(!loop.start(error))throw std::runtime_error(error);
        safety=std::thread([&]{while(!safety_done.load()){
            pollfd p{urgent,POLLIN,0};if(poll(&p,1,50)>0){char ch{};if(recv(urgent,&ch,1,0)==1){if(ch=='H')loop.heartbeat();else if(ch=='E'){loop.emergency_stop();driver.request_stop();}}}
            jc::ControlStatus status{};
            if(interrupted){loop.emergency_stop();driver.request_stop();}
            else if(loop.status(status)&&status.state==jc::State::Fault)driver.request_stop();
        }});
        SafetyGuard safety_guard{safety_done,safety};
        bool done=false,disabled=initial_disabled,disable_attempted=false,recording=false;
        unsigned round=0;std::string record_path,accepted_config_text;jc::ExperimentConfig accepted_config{};
        const auto finish_recording=[&](const jc::ControlStatus& current){
            jc::LoopSummary summary{};summary.final_state=current.state;summary.final_fault=current.fault;
            logger.finish(summary);recording=false;
        };
        while(!done&&!interrupted&&loop.running()){
            jc::ControlStatus s{};if(!loop.status(s))continue;
            if(s.state==jc::State::Fault&&!disabled&&!disable_attempted){
                disabled=driver.disable(error).state_confirmed;disable_attempted=true;
                if(!disabled)std::cerr<<"STOP NOT CONFIRMED: "<<error<<'\n';
            }
            if(s.state==jc::State::Run&&s.stage==jc::ExperimentStage::SwitchingCsp&&!disabled){
                const auto checked=driver.hold_csp(error);
                jc::Command resume{};resume.type=jc::CommandType::ResumeCsp;
                if(!checked.state_confirmed||!submit(commands,loop,resume,error)){
                    loop.emergency_stop();disabled=driver.disable(error).state_confirmed;disable_attempted=true;
                    std::cerr<<"Unload handover failed: "<<error<<'\n';
                }
                loop.status(s);
            }
            if(s.stage==jc::ExperimentStage::Complete&&!disabled){
                disabled=driver.disable(error).state_confirmed;disable_attempted=true;
                if(!disabled){loop.emergency_stop();std::cerr<<"STOP NOT CONFIRMED: "<<error<<'\n';}
                else {jc::Command stop_command{};stop_command.type=jc::CommandType::Stop;
                    (void)submit(commands,loop,stop_command,error);loop.status(s);}
            }
            if(recording&&(s.state==jc::State::Fault||s.stage==jc::ExperimentStage::Complete)&&disable_attempted)
                finish_recording(s);
            pollfd p{server,POLLIN,0};if(poll(&p,1,50)<=0)continue;
            const int client=accept4(server,nullptr,nullptr,SOCK_CLOEXEC);if(client<0)continue;
            timeval timeout{0,300000};setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
            std::string input;char buffer[2048];ssize_t count=0;
            while(input.size()<16384&&(count=read(client,buffer,sizeof(buffer)))>0){input.append(buffer,static_cast<std::size_t>(count));if(input.find('\n')!=std::string::npos)break;}
            bool ok=false;std::string message="invalid request";const auto op=string_field(input,"op"),request=string_field(input,"request_id");
            error.clear();
            if(number_field(input,"version")==1&&!request.empty()&&input.size()<16384){
                try{
                    jc::Command command{};
                    if(op=="status"||op=="force"||op=="start"){ok=true;message="status";}
                    else if(op=="configure"){
                        command.type=jc::CommandType::Configure;command.experiment=config_from(input);ok=submit(commands,loop,command,error);
                        if(ok){accepted_config=command.experiment;accepted_config_text=input;}
                    }else if(op=="force-zero"){
                        error="zero in terminal 3 (force_sensor), not in controller";
                    }else if(op=="preload"){
                        const auto now=jc::monotonic_now_ns();
                        const double age=s.force.timestamp_ns&&now>=s.force.timestamp_ns?static_cast<double>(now-s.force.timestamp_ns)*1e-9:1e9;
                        if(recording||s.state!=jc::State::Idle||!output||!s.configured||!s.feedback_valid||!s.force.zeroed||!s.stationary||
                            !jc::safe_measurement(accepted_config,s.actual_position,s.actual_velocity,s.force.external,age,s.force.valid))
                            error="configure, complete no-load zero and satisfy safety limits before preload";
                        else{
                            record_path=csv+".run"+std::to_string(++round)+".csv";
                            std::FILE* snapshot=std::fopen((record_path+".config.yaml").c_str(),"wx");
                            if(!snapshot)throw std::runtime_error("cannot create configuration snapshot");
                            const bool snapshot_failed=std::fputs(accepted_config_text.c_str(),snapshot)<0;
                            const int closed=std::fclose(snapshot);
                            if(snapshot_failed||closed!=0)throw std::runtime_error("configuration write failed");
                            if(!logger.start(record_path,config,error))throw std::runtime_error(error);
                            recording=true;
                            if(s.state==jc::State::Idle){
                                disabled=false;disable_attempted=false;driver.clear_stop();const auto ds=driver.enable(error);
                                jc::ControlStatus after{};loop.status(after);
                                ok=ds.state_confirmed&&ds.operation_enabled&&ds.mode_display==8&&after.state!=jc::State::Fault&&ds.position_known&&
                                    std::abs(after.actual_position-ds.position_m)<=0.00025&&jc::same_force_zero(s.force,after.force);
                                if(ok){command.type=jc::CommandType::Arm;ok=submit(commands,loop,command,error);}
                                if(ok){command.type=jc::CommandType::Run;ok=submit(commands,loop,command,error);}
                                if(!ok){loop.emergency_stop();disabled=driver.disable(error).state_confirmed;}
                            }else ok=s.state==jc::State::Run;
                            if(ok){command.type=jc::CommandType::Preload;ok=submit(commands,loop,command,error);}
                            if(!ok){loop.emergency_stop();disabled=driver.disable(error).state_confirmed;
                                disable_attempted=true;loop.status(s);finish_recording(s);}
                        }
                    }else if(op=="experiment-start"){
                        if(!output||disabled||!recording||s.state!=jc::State::Run||s.stage!=jc::ExperimentStage::Ready||!s.stationary)
                            error="loading requires stationary READY with recording active";
                        else{
                            command.type=jc::CommandType::BeginLoad;ok=submit(commands,loop,command,error);
                            if(!ok)error="load rejected: wait for 100 ms of stationary READY feedback, check raw driver version, matching load direction and peak above holding feedback";
                            else{
                                loop.status(s);
                                const auto checked=driver.load_raw(static_cast<int>(s.commanded_current_raw),static_cast<int>(accepted_config.current_limit_raw),error);
                                ok=checked.state_confirmed;
                                if(ok){command.type=jc::CommandType::CommitLoad;ok=submit(commands,loop,command,error);}
                                if(!ok){const auto reason=error;loop.emergency_stop();disabled=driver.disable(error).state_confirmed;disable_attempted=true;error=reason;}
                            }
                        }
                    }
                    else if(op=="unload"){command.type=jc::CommandType::Unload;ok=submit(commands,loop,command,error);}
                    else if(op=="reset"){
                        if(!disabled)error="drive disable is not confirmed";
                        else{command.type=jc::CommandType::Reset;ok=submit(commands,loop,command,error);}
                    }else if(op=="disable"||op=="shutdown"){
                        loop.emergency_stop();disabled=driver.disable(error).state_confirmed;disable_attempted=true;ok=disabled;
                        if(op=="shutdown"&&ok)done=true;
                    }else error="unsupported operation";
                    message=ok?"accepted":error.empty()?"operation rejected":error;
                }catch(const std::exception& e){message=e.what();}
            }
            loop.status(s);const std::string reply="{\"version\":1,\"request_id\":"+quote(request)+",\"ok\":"+(ok?"true":"false")+",\"message\":"+quote(message)+","+status_json(s,disabled)+",\"recording\":"+(recording?"true":"false")+",\"csv_path\":"+quote(record_path)+"}\n";
            std::size_t sent=0;while(sent<reply.size()){const auto n=write(client,reply.data()+sent,reply.size()-sent);if(n<=0)break;sent+=static_cast<std::size_t>(n);}close(client);
        }
        loop.emergency_stop();if(output&&!disabled){disabled=driver.disable(error).state_confirmed;if(!disabled)std::cerr<<"STOP NOT CONFIRMED: "<<error<<'\n';}
        safety_done.store(true);safety.join();loop.request_stop();loop.join();
        if(recording)logger.finish(loop.summary());
        // Keep an unconfirmed live driver available for manual stop; do not hide it by killing the launcher.
        if(output&&disabled)driver.shutdown(error);
        close(server);close(urgent);unlink(socket_path.c_str());unlink((socket_path+".urgent").c_str());return disabled?0:4;
    }catch(const std::exception& e){
        safety_done.store(true);if(safety.joinable())safety.join();
        if(server>=0){close(server);unlink(socket_path.c_str());}if(urgent>=0){close(urgent);unlink((socket_path+".urgent").c_str());}
        std::cerr<<e.what()<<'\n';return 2;
    }
}
