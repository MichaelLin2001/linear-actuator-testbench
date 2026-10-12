#pragma once
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

// The readline hook executes remote commands on the same thread as keyboard commands.
class CommandSession {
    std::string directory_;
    int lock_{-1}, log_{-1}, terminal_{-1};
public:
    explicit CommandSession(std::string directory):directory_(std::move(directory)) {
        lock_=open((directory_+"/driver.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);
        if(lock_<0||flock(lock_,LOCK_EX|LOCK_NB)!=0){
            if(lock_>=0)close(lock_);
            throw std::runtime_error("motor_test already running or runtime lock unavailable");
        }
        log_=open((directory_+"/session.log").c_str(),O_CREAT|O_RDWR|O_APPEND|O_CLOEXEC,0600);
        terminal_=dup(STDOUT_FILENO);
        std::ofstream pid(directory_+"/motor.pid");pid<<getpid()<<'\n';pid.close();
        if(log_<0||terminal_<0||!pid){
            if(log_>=0)close(log_);
            if(terminal_>=0)close(terminal_);
            close(lock_);throw std::runtime_error("cannot initialize driver command session");
        }
        unlink((directory_+"/command").c_str());
    }
    ~CommandSession(){unlink((directory_+"/motor.pid").c_str());close(log_);close(terminal_);close(lock_);}
    CommandSession(const CommandSession&)=delete;
    CommandSession& operator=(const CommandSession&)=delete;
    template<class Execute> bool poll(Execute execute) {
        std::ifstream input(directory_+"/command");if(!input)return false;
        char buffer[513]{};input.getline(buffer,sizeof(buffer));
        const bool valid=!input.fail();input.close();unlink((directory_+"/command").c_str());
        if(!valid||!buffer[0])return false;
        std::cout.flush();fflush(stdout);
        const auto offset=lseek(log_,0,SEEK_END);
        if(offset<0||dup2(log_,STDOUT_FILENO)<0)throw std::runtime_error("cannot capture command response");
        int result=0;
        try {
            std::cout<<buffer<<'\n';
            result=execute(std::string(buffer));
            std::cout<<"\n[Command] >"<<std::flush;fflush(stdout);
        }catch(...){dup2(terminal_,STDOUT_FILENO);throw;}
        if(dup2(terminal_,STDOUT_FILENO)<0)throw std::runtime_error("cannot restore driver terminal");
        std::ifstream response(directory_+"/session.log");response.seekg(offset);
        std::cout<<response.rdbuf()<<std::flush;
        return result==-1;
    }
};
