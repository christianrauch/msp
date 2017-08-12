#include <Client.hpp>
#include <msp_msg.hpp>
#include <msg_print.hpp>

#include <iostream>

typedef unsigned int uint;

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const uint baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    msp::client::Client client;
    client.setPrintWarnings(true);
    client.connect(device, baudrate);
    client.start();

    msp::msg::Ident ident;
    if(client.request(ident)==1)
        std::cout<<ident;
    else
        std::cerr<<"unsupported: "<<uint(ident.id())<<std::endl;

    msp::msg::Status status;
    if(client.request(status)==1)
        std::cout<<status;
    else
        std::cerr<<"unsupported: "<<uint(status.id())<<std::endl;

    msp::msg::ImuRaw imu_raw;
    if(client.request(imu_raw)==1) {
        std::cout<<msp::msg::ImuSI(imu_raw, 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);
    }
    else
        std::cerr<<"unsupported: "<<uint(imu_raw.id())<<std::endl;

    msp::msg::Servo servo;
    if(client.request(servo)==1)
        std::cout<<servo;
    else
        std::cerr<<"unsupported: "<<uint(servo.id())<<std::endl;

    msp::msg::Motor motor;
    if(client.request(motor)==1)
        std::cout<<motor;
    else
        std::cerr<<"unsupported: "<<uint(motor.id())<<std::endl;

    msp::msg::Rc rc;
    if(client.request(rc)==1)
        std::cout<<rc;
    else
        std::cerr<<"unsupported: "<<uint(rc.id())<<std::endl;

    msp::msg::Attitude attitude;
    if(client.request(attitude)==1)
        std::cout<<attitude;
    else
        std::cerr<<"unsupported: "<<uint(attitude.id())<<std::endl;

    msp::msg::Altitude altitude;
    if(client.request(altitude)==1)
        std::cout<<altitude;
    else
        std::cerr<<"unsupported: "<<uint(altitude.id())<<std::endl;

    msp::msg::Analog analog;
    if(client.request(analog)==1)
        std::cout<<analog;
    else
        std::cerr<<"unsupported: "<<uint(analog.id())<<std::endl;

    msp::msg::RcTuning rc_tuning;
    if(client.request(rc_tuning)==1)
        std::cout<<rc_tuning;
    else
        std::cerr<<"unsupported: "<<uint(rc_tuning.id())<<std::endl;

    msp::msg::Pid pid;
    if(client.request(pid)==1)
        std::cout<<pid;
    else
        std::cerr<<"unsupported: "<<uint(pid.id())<<std::endl;

    msp::msg::Box box;
    if(client.request(box)==1)
        std::cout<<box;
    else
        std::cerr<<"unsupported: "<<uint(box.id())<<std::endl;

    msp::msg::Misc misc;
    if(client.request(misc)==1)
        std::cout<<misc;
    else
        std::cerr<<"unsupported: "<<uint(misc.id())<<std::endl;

    msp::msg::MotorPins pins;
    if(client.request(pins)==1)
        std::cout<<pins;
    else
        std::cerr<<"unsupported: "<<uint(pins.id())<<std::endl;

    msp::msg::BoxNames box_names;
    if(client.request(box_names)==1)
        std::cout<<box_names;
    else
        std::cerr<<"unsupported: "<<uint(box_names.id())<<std::endl;

    msp::msg::PidNames pid_names;
    if(client.request(pid_names)==1)
        std::cout<<pid_names;
    else
        std::cerr<<"unsupported: "<<uint(pid_names.id())<<std::endl;

    msp::msg::BoxIds box_ids;
    if(client.request(box_ids)==1)
        std::cout<<box_ids;
    else
        std::cerr<<"unsupported: "<<uint(box_ids.id())<<std::endl;

    msp::msg::ServoConf servo_conf;
    if(client.request(servo_conf)==1)
        std::cout<<servo_conf;
    else
        std::cerr<<"unsupported: "<<uint(servo_conf.id())<<std::endl;

    // needs "#define DEBUGMSG" in MultiWii firmware
    msp::msg::DebugMessage debug_msg;
    if(client.request(debug_msg)==1) {
            std::cout<<"#Debug message:"<<std::endl;
            std::cout<<debug_msg.msg<<std::endl;
    }
    else
        std::cerr<<"unsupported: "<<uint(debug_msg.id())<<std::endl;

    msp::msg::Debug debug;
    if(client.request(debug)==1)
        std::cout<<debug;
    else
        std::cerr<<"unsupported: "<<uint(debug.id())<<std::endl;

    client.stop();
}
