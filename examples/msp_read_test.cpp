#include <MSP.hpp>
#include <msp_msg.hpp>

#include <iostream>

#include <msg_print.hpp>

typedef unsigned int uint;

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const uint baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    msp::MSP msp(device, baudrate);
    msp.setWait(1);

    // wait for flight controller to become ready
    {
    std::cout<<"Connecting FCU..."<<std::endl;
    msp::Ident ident;
    msp.request_wait(ident, 10);
    std::cout<<"MSP version "<<uint(ident.version)<<" ready"<<std::endl;
    }

    std::cout<<"ready"<<std::endl;

    msp::Ident ident;
    msp.request_block(ident);
    std::cout<<ident;

    msp::Status status;
    msp.request_block(status);
    std::cout<<status;

    msp::Imu imu(512.0, 1.0/4.096, 0.92/10.0, 9.80665);
    msp.request_block(imu);
    std::cout<<imu;

    msp::Servo servo;
    msp.request_block(servo);
    std::cout<<servo;

    msp::Motor motor;
    msp.request_block(motor);
    std::cout<<motor;

    msp::Rc rc;
    msp.request_block(rc);
    std::cout<<rc;

    msp::Attitude attitude;
    msp.request_block(attitude);
    std::cout<<attitude;

    msp::Altitude altitude;
    msp.request_block(altitude);
    std::cout<<altitude;

    msp::Analog analog;
    msp.request_block(analog);
    std::cout<<analog;

    msp::RcTuning rc_tuning;
    msp.request_block(rc_tuning);
    std::cout<<rc_tuning;

    msp::Pid pid;
    msp.request_block(pid);
    std::cout<<pid;

    msp::Box box;
    msp.request_block(box);
    std::cout<<box;

    msp::Misc misc;
    msp.request_block(misc);
    std::cout<<misc;

    msp::MotorPins pins;
    msp.request_block(pins);
    std::cout<<pins;

    msp::BoxNames box_names;
    msp.request_block(box_names);
    std::cout<<box_names;

    msp::PidNames pid_names;
    msp.request_block(pid_names);
    std::cout<<pid_names;

    msp::BoxIds box_ids;
    msp.request_block(box_ids);
    std::cout<<box_ids;

    msp::ServoConf servo_conf;
    msp.request_block(servo_conf);
    std::cout<<servo_conf;

    // needs "#define DEBUGMSG" in MultiWii firmware
    msp::DebugMessage debug_msg;
    if(msp.request_block(debug_msg)) {
        std::cout<<"#Debug message:"<<std::endl;
        std::cout<<debug_msg.msg<<std::endl;
    }

    msp::Debug debug;
    msp.request_block(debug);
    std::cout<<debug;
}
