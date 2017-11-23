#include <MSP.hpp>
#include <msp_msg.hpp>

#include <iostream>

#include <msg_print.hpp>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    msp::MSP msp(device, baudrate);
    msp.setWait(1);

    // wait for flight controller to become ready
    {
    std::cout<<"Connecting FCU..."<<std::endl;
    msp::msg::Ident ident;
    if(msp.request_wait(ident, 10)) {
        std::cout<<"MSP version "<< size_t(ident.version)<<" ready"<<std::endl;
    }
    }

    std::cout<<"ready"<<std::endl;

    msp::msg::Ident ident;
    if(msp.request_block(ident))
        std::cout<<ident;
    else
        std::cerr<<"unsupported: "<< size_t(ident.id())<<std::endl;

    msp::msg::Status status;
    if(msp.request_block(status))
        std::cout<<status;
    else
        std::cerr<<"unsupported: "<< size_t(status.id())<<std::endl;

    msp::msg::ImuRaw imu_raw;
    if(msp.request_block(imu_raw))
        std::cout<<msp::msg::ImuSI(imu_raw, 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);
    else
        std::cerr<<"unsupported: "<< size_t(imu_raw.id())<<std::endl;

    msp::msg::Servo servo;
    if(msp.request_block(servo))
        std::cout<<servo;
    else
        std::cerr<<"unsupported: "<< size_t(servo.id())<<std::endl;

    msp::msg::Motor motor;
    if(msp.request_block(motor))
        std::cout<<motor;
    else
        std::cerr<<"unsupported: "<< size_t(motor.id())<<std::endl;

    msp::msg::Rc rc;
    if(msp.request_block(rc))
        std::cout<<rc;
    else
        std::cerr<<"unsupported: "<< size_t(rc.id())<<std::endl;

    msp::msg::Attitude attitude;
    if(msp.request_block(attitude))
        std::cout<<attitude;
    else
        std::cerr<<"unsupported: "<< size_t(attitude.id())<<std::endl;

    msp::msg::Altitude altitude;
    if(msp.request_block(altitude))
        std::cout<<altitude;
    else
        std::cerr<<"unsupported: "<< size_t(altitude.id())<<std::endl;

    msp::msg::Analog analog;
    if(msp.request_block(analog))
        std::cout<<analog;
    else
        std::cerr<<"unsupported: "<< size_t(analog.id())<<std::endl;

    msp::msg::RcTuning rc_tuning;
    if(msp.request_block(rc_tuning))
        std::cout<<rc_tuning;
    else
        std::cerr<<"unsupported: "<< size_t(rc_tuning.id())<<std::endl;

    msp::msg::Pid pid;
    if(msp.request_block(pid))
        std::cout<<pid;
    else
        std::cerr<<"unsupported: "<< size_t(pid.id())<<std::endl;

    msp::msg::Box box;
    if(msp.request_block(box))
        std::cout<<box;
    else
        std::cerr<<"unsupported: "<< size_t(box.id())<<std::endl;

    msp::msg::Misc misc;
    if(msp.request_block(misc))
        std::cout<<misc;
    else
        std::cerr<<"unsupported: "<< size_t(misc.id())<<std::endl;

    msp::msg::MotorPins pins;
    if(msp.request_block(pins))
        std::cout<<pins;
    else
        std::cerr<<"unsupported: "<< size_t(pins.id())<<std::endl;

    msp::msg::BoxNames box_names;
    if(msp.request_block(box_names))
        std::cout<<box_names;
    else
        std::cerr<<"unsupported: "<< size_t(box_names.id())<<std::endl;

    msp::msg::PidNames pid_names;
    if(msp.request_block(pid_names))
        std::cout<<pid_names;
    else
        std::cerr<<"unsupported: "<< size_t(pid_names.id())<<std::endl;

    msp::msg::BoxIds box_ids;
    if(msp.request_block(box_ids))
        std::cout<<box_ids;
    else
        std::cerr<<"unsupported: "<< size_t(box_ids.id())<<std::endl;

    msp::msg::ServoConf servo_conf;
    if(msp.request_block(servo_conf))
        std::cout<<servo_conf;
    else
        std::cerr<<"unsupported: "<< size_t(servo_conf.id())<<std::endl;

    // needs "#define DEBUGMSG" in MultiWii firmware
    msp::msg::DebugMessage debug_msg;
    if(msp.request_block(debug_msg)) {
        std::cout<<"#Debug message:"<<std::endl;
        std::cout<<debug_msg.msg<<std::endl;
    }
    else
        std::cerr<<"unsupported: "<< size_t(debug_msg.id())<<std::endl;

    msp::msg::Debug debug;
    if(msp.request_block(debug))
        std::cout<<debug;
    else
        std::cerr<<"unsupported: "<< size_t(debug.id())<<std::endl;
}
