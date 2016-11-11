#include <MSP.hpp>
#include <msp_msg.hpp>

#include <iostream>

#include <msg_print.hpp>

int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    msp::MSP msp(device);
    msp.setWait(1);

    sleep(8);

    // wait for flight controller to become ready
//    {
//    std::cout<<"Connecting FCU..."<<std::endl;
//    msp::Ident ident;
//    msp.request_timeout(ident, 10);
//    std::cout<<"MSP version "<<(int)ident.version<<" ready"<<std::endl;
//    }

    msp::Ident ident;
    msp.request_block(ident);
    std::cout<<fcu::Ident(ident);

    msp::Status status;
    msp.request_block(status);
    std::cout<<fcu::Status(status);

    msp::RawImu imu;
    msp.request_block(imu);
    std::cout<<fcu::Imu(imu, 512, 1/4096.0);

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
    std::cout<<fcu::Attitude(attitude);

    msp::Altitude altitude;
    msp.request_block(altitude);
    std::cout<<fcu::Altitude(altitude);

    msp::Analog analog;
    msp.request_block(analog);
    std::cout<<fcu::Analog(analog);

    msp::RcTuning rc_tuning;
    msp.request_block(rc_tuning);
    std::cout<<rc_tuning;

    msp::Pid pid;
    msp.request_block(pid);
    std::cout<<fcu::PID(pid);

    msp::Box box;
    msp.request_block(box);
    std::cout<<fcu::Box(box);

    msp::Misc misc;
    msp.request_block(misc);
    std::cout<<fcu::Misc(misc);

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
