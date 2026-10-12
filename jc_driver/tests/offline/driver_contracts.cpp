#include "../../application/jc01/jc01.cpp"
#include <cstdlib>
#include "../../application/application.cpp"
#define CHECK(x) do { if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);} } while(false)
static bool enabled=false,fail_after_enable=false,fail_read=false;
static uint16_t status_override=0;
static int8_t mode_display=8;
static bool interrupt_move=false,ignore_mode=false,bad_torque_read=false;
static uint16_t torque_target=0;
static unsigned resets=0;
static unsigned sends=0;
static EtherCAT_Msg sent{};
extern "C" { int ec_slavecount=1; }
extern "C" int ethercat_send_message_continue(int slave,const EtherCAT_Msg message) {
    CHECK(slave==0);sent=message;++sends;return 0;
}
void m18_build_csp_message(can_frame* frame,uint32_t node,int32_t position) {
    frame->id=0x200+node;frame->dlc=6;std::memcpy(frame->data,&position,sizeof(position));
}
void m18_build_cst_message(can_frame* frame,uint32_t node,int16_t raw) { frame->id=0x200+node;frame->dlc=6;std::memcpy(frame->data+4,&raw,2); }
static void check_periodic_target(float target) {
    const auto before=sends;
    for(unsigned i=0;i<100;++i) {
        txMessageCallBack();
        CHECK(sent.motor[0].id==0x201&&sent.motor[0].dlc==6);
        CHECK(sent.motor[1].dlc==0&&sent.motor[2].id==0x80&&sent.motor[2].dlc==1);
        for(unsigned slot=3;slot<6;++slot)CHECK(sent.motor[slot].dlc==0);
        int32_t position=0;std::memcpy(&position,sent.motor[0].data,sizeof(position));
        CHECK(position==static_cast<int32_t>(MM_2_UNIT(target)));
    }
    CHECK(sends==before+100&&app_get_running()==APP_MODE_POS);
}

static int32_t actual=static_cast<int32_t>(MM_2_UNIT(12.0f)),hold=actual;
extern "C" void _JC_LOG(Level,int,const char*,const char*,...) {}
extern "C" int usleep(useconds_t) { return 0; }
extern "C" int clock_nanosleep(clockid_t,int,const timespec*,timespec*) { if(interrupt_move)app_exit_requested=1; return 0; }
extern "C" void ethercat_reset_continue_tx_buffers() { ++resets; }
int m18_reset_node(uint8_t,uint8_t,uint32_t) { enabled=false;return 0; }
int m18_start_node(uint8_t,uint8_t,uint32_t) { return 0; }
void m18_set_mode(uint8_t,uint8_t,uint32_t,uint8_t) {}
void m18_set_pdo_map(uint8_t,uint8_t,uint32_t,uint8_t,uint32_t*,uint8_t) {}
void m18_enable_driver(uint8_t,uint8_t,uint32_t) { enabled=true; }
void m18_disable_driver(uint8_t,uint8_t,uint32_t) { enabled=false; }
int m18_write_sdo(uint8_t,uint8_t,uint32_t,uint16_t index,uint8_t,uint32_t value,SDO_CMD) {
    if(index==0x607A)hold=static_cast<int32_t>(value);
    if(index==0x6060&&!ignore_mode)mode_display=static_cast<int8_t>(value);
    if(index==0x6071)torque_target=static_cast<uint16_t>(value);return 0;
}
int m18_read_sdo(uint8_t,uint8_t,uint32_t,uint16_t index,uint8_t,uint32_t* value,uint8_t* size) {
    if(fail_read || (enabled&&fail_after_enable))return -1;
    switch(index){
        case 0x6041:*value=status_override?status_override:(enabled?0x27:0x40);*size=2;break;
        case 0x6061:case 0x6060:*value=mode_display;*size=1;break;
        case 0x6064:*value=static_cast<uint32_t>(actual);*size=4;break;
        case 0x6071:*value=torque_target+(bad_torque_read?1:0);*size=2;break;
        case 0x607A:*value=static_cast<uint32_t>(hold);*size=4;break;
        default:return -1;
    }return 0;
}
int main() {
    CHECK(std::strcmp(m18_ds402_state_name(0x0638),"Fault")==0);
    CHECK(std::strcmp(m18_ds402_state_name(0x063F),"Fault reaction active")==0);
    CHECK(std::strcmp(m18_ds402_state_name(0x1737),"Operation enabled")==0);
    CHECK(std::strcmp(m18_ds402_state_name(0x0450),"Switch on disabled")==0);
    CHECK(triple_map_init(jx_map,id_map,1)==0);
    SharedMemory target("",480,0,0,""),feedback("",480,0,0,"");shm_target=&target;shm_feedback=&feedback;
    for(int32_t raw:{-32768,-123,-1,0,1,123,32767}){
        const auto bits=static_cast<uint16_t>(raw);
        const uint8_t bytes[]={0,0,0,0,static_cast<uint8_t>(bits),static_cast<uint8_t>(bits>>8),0,0};
        float position=0,feedback_raw=0;
        CHECK(decode_m18_txpdo1(bytes,&position,&feedback_raw)==0);
        CHECK(feedback_raw==static_cast<float>(raw));
        app_set_motor_fdb(3,0,position,feedback_raw);
        MHandle readback{};app_get_motor_fdb(3,&readback);
        CHECK(readback.feedback_torque==static_cast<float>(raw));
    }
    const float current=UNIT_2_MM(actual);app_set_motor_fdb(3,0,current,0);
    CHECK(app_m18_enable({"AppM18Enable","EN_DRV"})==0);
    CHECK(app_get_running()==APP_MODE_POS&&resets==1&&enabled);
    MHandle slot{};app_get_motor_target(3,&slot);CHECK(slot.target_pos==current);
    check_periodic_target(current);CHECK(resets==1);
    CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative","0.1"})==0);
    app_get_motor_target(3,&slot);CHECK(std::fabs(slot.target_pos-(current+.1f))<1e-6);
    check_periodic_target(slot.target_pos);CHECK(resets==1);
    CHECK(app_motor_write({"AppWrite","3",std::to_string(static_cast<double>(current)-.05)})==0);
    app_get_motor_target(3,&slot);CHECK(std::fabs(slot.target_pos-(current-.05f))<1e-6);
    check_periodic_target(slot.target_pos);CHECK(resets==1);
    CHECK(app_motor_write({"AppWrite","3",std::to_string(current)})==0);
    for(const auto& text:{"nan","inf","0.05junk","","1e20","-1e20"})
        CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative",text})!=0);
    CHECK(app_motor_write({"AppWrite","9","12"})!=0);
    CHECK(app_motor_write({"AppWrite","3","12junk"})!=0);
    CHECK(app_motor_write({"AppWrite","3","13"})==0);
    app_get_motor_target(3,&slot);CHECK(slot.target_pos==13);
    check_periodic_target(slot.target_pos);
    CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative","1"})==0);
    app_get_motor_target(3,&slot);CHECK(std::fabs(slot.target_pos-(current+1))<1e-6);
    CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative","-1"})==0);
    app_get_motor_target(3,&slot);CHECK(std::fabs(slot.target_pos-(current-1))<1e-6);
    CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative","0.00001"})==0);
    CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative","0"})==0);
    CHECK(app_motor_write({"AppWrite","3","1e20"})!=0);
    CHECK(app_motor_write({"AppWrite","3","-1e20"})!=0);
    CHECK(app_motor_write({"AppWrite","3",std::to_string(static_cast<double>(INT32_MAX)/POS_UNIT)})!=0);
    CHECK(app_get_running()==APP_MODE_POS&&resets==1);
    CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative","-.1"})==0);
    CHECK(app_m18_enable({"AppM18Enable","DIS_DRV"})==0&&app_get_running()==APP_MODE_STOP);
    const auto disabled_sends=sends;txMessageCallBack();CHECK(sends==disabled_sends);
    CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative",".05"})!=0);
    fail_after_enable=true;resets=0;
    CHECK(app_m18_enable({"AppM18Enable","EN_DRV"})!=0);
    CHECK(app_get_running()==APP_MODE_STOP&&!enabled&&resets==2);
    txMessageCallBack();CHECK(sends==disabled_sends);
    fail_after_enable=false;CHECK(app_m18_enable({"AppM18Enable","RESET"})==0&&app_get_running()==APP_MODE_STOP);
    CHECK(app_start({"AppStart","STOP"})==0&&app_get_running()==APP_MODE_STOP);
    txMessageCallBack();CHECK(sends==disabled_sends);
    // A fresh process has zero shared feedback even when the drive is still enabled.
    app_set_motor_fdb(3,0,0,0);app_set_motor_target(3,0,0,0);enabled=true;
    app_set_motor_runmode(3, RM_CST);
    CHECK(app_start({"AppStart","POS"})==0);
    app_get_motor_target(3,&slot);CHECK(slot.target_pos==current);
    check_periodic_target(current);
    for(int failure=0;failure<3;++failure) {
        fail_read=failure==0;status_override=failure==1?0x0638:0;mode_display=failure==2?10:8;
        CHECK(app_start({"AppStart","POS"})!=0&&app_get_running()==APP_MODE_STOP);
        const auto before=sends;txMessageCallBack();CHECK(sends==before);
    }
    fail_read=false;status_override=0;mode_display=0;enabled=false;
    CHECK(app_start({"AppStart","POS"})==0);
    check_periodic_target(current);
    mode_display=8;enabled=true;app_set_motor_fdb(3,0,current,0);
    CHECK(app_start({"AppStart","TOR"})!=0);
    for(int seed:{-10,0,10}) {
        CHECK(app_m18_load_raw({"AppM18LoadRaw",std::to_string(seed),"20"})==0);
        CHECK(mode_display==10&&app_get_running()==APP_MODE_TOR);
        txMessageCallBack();int16_t raw=0;std::memcpy(&raw,sent.motor[0].data+4,2);
        CHECK(raw==seed&&sent.motor[1].dlc==0);
        app_set_motor_target(3,0,current,0);
        CHECK(app_m18_csp_hold({"AppM18CspHold"})==0);
        CHECK(mode_display==8&&enabled);check_periodic_target(current);
    }
    CHECK(app_m18_load_raw({"AppM18LoadRaw","21","20"})!=0);
    CHECK(app_m18_load_raw({"AppM18LoadRaw","1.5","20"})!=0);
    CHECK(app_m18_load_raw({"AppM18LoadRaw","1","20"})==0);
    CHECK(app_m18_csp_hold({"AppM18CspHold"})!=0&&!enabled);
    enabled=true;mode_display=8;bad_torque_read=true;
    CHECK(app_m18_load_raw({"AppM18LoadRaw","1","20"})!=0&&!enabled);
    bad_torque_read=false;enabled=true;mode_display=8;ignore_mode=true;
    CHECK(app_m18_load_raw({"AppM18LoadRaw","1","20"})!=0&&!enabled);
    ignore_mode=false;enabled=true;mode_display=8;
    CHECK(app_m18_load_raw({"AppM18LoadRaw","1","20"})==0);
    app_set_motor_target(3,0,current,21);txMessageCallBack();
    int16_t zero=-1;std::memcpy(&zero,sent.motor[0].data+4,2);
    CHECK(zero==0&&sent.motor[1].dlc>0&&raw_fault.load());
    app_set_motor_target(3,0,current,0);txMessageCallBack();CHECK(raw_fault.load());
    CHECK(raw_lease_alive(65535,1,100));CHECK(raw_lease_alive(0,50000001,100));
    CHECK(raw_lease_alive(0,149999999,100));CHECK(!raw_lease_alive(0,150000001,100));
    CHECK(!raw_lease_alive(1,150000002,100));CHECK(raw_lease_alive(1,150000003,101));
    mode_display=8;enabled=true;raw_fault.store(false);app_set_running(APP_MODE_POS);
    interrupt_move=true;
    CHECK(app_m18_move_j3_relative({"AppMoveJ3Relative","5"})==0);
    app_get_motor_target(3,&slot);CHECK(slot.target_pos<current+0.1f);
    app_deinit();CHECK(!enabled&&app_get_running()==APP_MODE_STOP);
    const auto before=sends;txMessageCallBack();CHECK(sends==before);
    CHECK(app_start({"AppStart","POS"})!=0);
    app_exit_requested=0;interrupt_move=false;enabled=true;
    app_deinit();CHECK(!enabled&&app_get_running()==APP_MODE_STOP);
    std::puts("offline driver contracts passed");
}
