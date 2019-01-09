#include <Client.hpp>
#include <msp_msg.hpp>
#include <msg_print.hpp>

#include <iostream>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    msp::client::Client client;
    client.setPrintWarnings(true);
    client.connect(device, baudrate);
    client.start();
    msp::FirmwareVariant firmware = msp::FirmwareVariant::CLFL;
    msp::msg::Ident ident(firmware);
    if(client.request(ident)==1)
        std::cout<<ident;
    else
        std::cerr<<"unsupported: "<< size_t(ident.id())<<std::endl;

    msp::msg::Status status(firmware);
    if(client.request(status)==1)
        std::cout<<status;
    else
        std::cerr<<"unsupported: "<< size_t(status.id())<<std::endl;

    msp::msg::RawImu imu_raw(firmware);
    if(client.request(imu_raw)==1) {
        std::cout<<msp::msg::ScaledImu(imu_raw, 9.80665f/512.0, 1.0/4.096, 0.92f/10.0f);
    }
    else
        std::cerr<<"unsupported: "<< size_t(imu_raw.id())<<std::endl;

    msp::msg::Servo servo(firmware);
    if(client.request(servo)==1)
        std::cout<<servo;
    else
        std::cerr<<"unsupported: "<< size_t(servo.id())<<std::endl;

    msp::msg::Motor motor(firmware);
    if(client.request(motor)==1)
        std::cout<<motor;
    else
        std::cerr<<"unsupported: "<< size_t(motor.id())<<std::endl;

    msp::msg::Rc rc(firmware);
    if(client.request(rc)==1)
        std::cout<<rc;
    else
        std::cerr<<"unsupported: "<< size_t(rc.id())<<std::endl;

    msp::msg::Attitude attitude(firmware);
    if(client.request(attitude)==1)
        std::cout<<attitude;
    else
        std::cerr<<"unsupported: "<< size_t(attitude.id())<<std::endl;

    msp::msg::Altitude altitude(firmware);
    if(client.request(altitude)==1)
        std::cout<<altitude;
    else
        std::cerr<<"unsupported: "<< size_t(altitude.id())<<std::endl;

    msp::msg::Analog analog(firmware);
    if(client.request(analog)==1)
        std::cout<<analog;
    else
        std::cerr<<"unsupported: "<< size_t(analog.id())<<std::endl;

    msp::msg::RcTuning rc_tuning(firmware);
    if(client.request(rc_tuning)==1)
        std::cout<<rc_tuning;
    else
        std::cerr<<"unsupported: "<< size_t(rc_tuning.id())<<std::endl;

    msp::msg::Pid pid(firmware);
    if(client.request(pid)==1)
        std::cout<<pid;
    else
        std::cerr<<"unsupported: "<< size_t(pid.id())<<std::endl;

    msp::msg::ActiveBoxes box(firmware);
    if(client.request(box)==1)
        std::cout<<box;
    else
        std::cerr<<"unsupported: "<< size_t(box.id())<<std::endl;

    msp::msg::Misc misc(firmware);
    if(client.request(misc)==1)
        std::cout<<misc;
    else
        std::cerr<<"unsupported: "<< size_t(misc.id())<<std::endl;

    msp::msg::MotorPins pins(firmware);
    if(client.request(pins)==1)
        std::cout<<pins;
    else
        std::cerr<<"unsupported: "<< size_t(pins.id())<<std::endl;

    msp::msg::BoxNames box_names(firmware);
    if(client.request(box_names)==1)
        std::cout<<box_names;
    else
        std::cerr<<"unsupported: "<< size_t(box_names.id())<<std::endl;

    msp::msg::PidNames pid_names(firmware);
    if(client.request(pid_names)==1)
        std::cout<<pid_names;
    else
        std::cerr<<"unsupported: "<< size_t(pid_names.id())<<std::endl;

    msp::msg::BoxIds box_ids(firmware);
    if(client.request(box_ids)==1)
        std::cout<<box_ids;
    else
        std::cerr<<"unsupported: "<< size_t(box_ids.id())<<std::endl;

    msp::msg::ServoConf servo_conf(firmware);
    if(client.request(servo_conf)==1)
        std::cout<<servo_conf;
    else
        std::cerr<<"unsupported: "<< size_t(servo_conf.id())<<std::endl;

    // needs "#define DEBUGMSG" in MultiWii firmware
    msp::msg::DebugMessage debug_msg(firmware);
    if(client.request(debug_msg)==1) {
            std::cout<<"#Debug message:"<<std::endl;
            std::cout<<debug_msg.debug_msg<<std::endl;
    }
    else
        std::cerr<<"unsupported: "<< size_t(debug_msg.id())<<std::endl;

    msp::msg::Debug debug(firmware);
    if(client.request(debug)==1)
        std::cout<<debug;
    else
        std::cerr<<"unsupported: "<< size_t(debug.id())<<std::endl;

    client.stop();
}
