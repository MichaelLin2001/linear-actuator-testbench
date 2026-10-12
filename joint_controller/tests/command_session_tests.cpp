#include "../../jc_driver/tests/motor_test/command_session.hpp"
#include <filesystem>
#include <cstdlib>
#define CHECK(x) do{if(!(x)){std::cerr<<__LINE__<<": "<<#x<<'\n';std::exit(1);}}while(false)
int main(){
    const std::string dir="/tmp/j3-command-test-"+std::to_string(getpid());
    std::filesystem::create_directory(dir);
    {
        CommandSession session(dir);
        bool rejected=false;try{CommandSession duplicate(dir);}catch(const std::exception&){rejected=true;}
        CHECK(rejected);
        CHECK(std::filesystem::exists(dir+"/motor.pid"));
        unsigned calls=0;
        auto execute=[&](const std::string& command){++calls;
            if(command=="quit")return -1;
            CHECK(command=="AppM18Status");printf("J3 Statusword: 0x0040 (Switch on disabled)\n");
            std::cout<<"J3 Mode display: 8\n";return 0;};
        CHECK(!session.poll(execute)&&calls==0);
        std::ofstream(dir+"/command")<<"AppM18Status\n";
        CHECK(!session.poll(execute)&&calls==1);
        CHECK(!session.poll(execute)&&calls==1);
        std::ifstream response(dir+"/session.log");
        const std::string text((std::istreambuf_iterator<char>(response)),{});
        CHECK(text.find("AppM18Status\nJ3 Statusword:")!=std::string::npos);
        CHECK(text.find("J3 Mode display: 8\n\n[Command] >")!=std::string::npos);
        std::ofstream(dir+"/command")<<std::string(600,'x');
        CHECK(!session.poll(execute)&&calls==1);
        std::ofstream(dir+"/command")<<"quit\n";
        CHECK(session.poll(execute)&&calls==2);
    }
    CHECK(!std::filesystem::exists(dir+"/motor.pid"));
    {CommandSession restarted(dir);}
    std::filesystem::remove_all(dir);
}
