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

    msp::Ident ident;
    if(client.request(ident))
        std::cout<<ident;

    msp::Status status;
    if(client.request(status))
        std::cout<<status;

    msp::ImuRaw imu_raw;
    if(client.request(imu_raw)) {
        std::cout<<msp::ImuSI(imu_raw, 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);
    }

    msp::Servo servo;
    if(client.request(servo))
        std::cout<<servo;

    msp::Motor motor;
    if(client.request(motor))
        std::cout<<motor;

    msp::Rc rc;
    if(client.request(rc))
        std::cout<<rc;

    msp::Attitude attitude;
    if(client.request(attitude))
        std::cout<<attitude;

    msp::Altitude altitude;
    if(client.request(altitude))
        std::cout<<altitude;

    msp::Analog analog;
    if(client.request(analog))
        std::cout<<analog;

    msp::RcTuning rc_tuning;
    if(client.request(rc_tuning))
        std::cout<<rc_tuning;

    msp::Pid pid;
    if(client.request(pid))
        std::cout<<pid;

    msp::Box box;
    if(client.request(box))
        std::cout<<box;

    msp::Misc misc;
    if(client.request(misc))
        std::cout<<misc;

    msp::MotorPins pins;
    if(client.request(pins))
        std::cout<<pins;

    msp::BoxNames box_names;
    if(client.request(box_names))
        std::cout<<box_names;

    msp::PidNames pid_names;
    if(client.request(pid_names))
        std::cout<<pid_names;

    msp::BoxIds box_ids;
    if(client.request(box_ids))
        std::cout<<box_ids;

    msp::ServoConf servo_conf;
    if(client.request(servo_conf))
        std::cout<<servo_conf;

    // needs "#define DEBUGMSG" in MultiWii firmware
    msp::DebugMessage debug_msg;
    if(client.request(debug_msg)) {
            std::cout<<"#Debug message:"<<std::endl;
            std::cout<<debug_msg.msg<<std::endl;
    }

    msp::Debug debug;
    if(client.request(debug))
        std::cout<<debug;

    client.stop();
}
