#include <Client.hpp>
#include <iostream>
#include <msp_msg.hpp>

int main(int argc, char *argv[]) {
    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    msp::client::Client client;
    client.setLoggingLevel(msp::client::LoggingLevel::WARNING);
    client.setVariant(msp::FirmwareVariant::INAV);
    client.start(device, baudrate);

    msp::FirmwareVariant fw_variant = msp::FirmwareVariant::INAV;

    msp::msg::Ident ident(fw_variant);
    if(client.sendMessage(ident) == 1) {
        std::cout << ident;
    }
    else
        std::cerr << "unsupported: " << size_t(ident.id()) << std::endl;

    msp::msg::Status status(fw_variant);
    if(client.sendMessage(status) == 1)
        std::cout << status;
    else
        std::cerr << "unsupported: " << size_t(status.id()) << std::endl;

    msp::msg::RawImu imu_raw(fw_variant);
    if(client.sendMessage(imu_raw) == 1) {
        std::cout << imu_raw;
        std::cout << msp::msg::ImuSI(
            imu_raw, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
    }
    else
        std::cerr << "unsupported: " << size_t(imu_raw.id()) << std::endl;

    msp::msg::Servo servo(fw_variant);
    if(client.sendMessage(servo) == 1)
        std::cout << servo;
    else
        std::cerr << "unsupported: " << size_t(servo.id()) << std::endl;

    msp::msg::Motor motor(fw_variant);
    if(client.sendMessage(motor) == 1)
        std::cout << motor;
    else
        std::cerr << "unsupported: " << size_t(motor.id()) << std::endl;

    msp::msg::Rc rc(fw_variant);
    if(client.sendMessage(rc) == 1)
        std::cout << rc;
    else
        std::cerr << "unsupported: " << size_t(rc.id()) << std::endl;

    msp::msg::Attitude attitude(fw_variant);
    if(client.sendMessage(attitude) == 1)
        std::cout << attitude;
    else
        std::cerr << "unsupported: " << size_t(attitude.id()) << std::endl;

    msp::msg::Altitude altitude(fw_variant);
    if(client.sendMessage(altitude) == 1)
        std::cout << altitude;
    else
        std::cerr << "unsupported: " << size_t(altitude.id()) << std::endl;

    msp::msg::Analog analog(fw_variant);
    if(client.sendMessage(analog) == 1)
        std::cout << analog;
    else
        std::cerr << "unsupported: " << size_t(analog.id()) << std::endl;

    msp::msg::RcTuning rc_tuning(fw_variant);
    if(client.sendMessage(rc_tuning) == 1)
        std::cout << rc_tuning;
    else
        std::cerr << "unsupported: " << size_t(rc_tuning.id()) << std::endl;

    msp::msg::Pid pid(fw_variant);
    if(client.sendMessage(pid) == 1)
        std::cout << pid;
    else
        std::cerr << "unsupported: " << size_t(pid.id()) << std::endl;

    msp::msg::ActiveBoxes box(fw_variant);
    if(client.sendMessage(box) == 1)
        std::cout << box;
    else
        std::cerr << "unsupported: " << size_t(box.id()) << std::endl;

    msp::msg::Misc misc(fw_variant);
    if(client.sendMessage(misc) == 1)
        std::cout << misc;
    else
        std::cerr << "unsupported: " << size_t(misc.id()) << std::endl;

    msp::msg::MotorPins pins(fw_variant);
    if(client.sendMessage(pins) == 1)
        std::cout << pins;
    else
        std::cerr << "unsupported: " << size_t(pins.id()) << std::endl;

    msp::msg::BoxNames box_names(fw_variant);
    if(client.sendMessage(box_names) == 1)
        std::cout << box_names;
    else
        std::cerr << "unsupported: " << size_t(box_names.id()) << std::endl;

    msp::msg::PidNames pid_names(fw_variant);
    if(client.sendMessage(pid_names) == 1)
        std::cout << pid_names;
    else
        std::cerr << "unsupported: " << size_t(pid_names.id()) << std::endl;

    msp::msg::BoxIds box_ids(fw_variant);
    if(client.sendMessage(box_ids) == 1)
        std::cout << box_ids;
    else
        std::cerr << "unsupported: " << size_t(box_ids.id()) << std::endl;

    msp::msg::ServoConf servo_conf(fw_variant);
    if(client.sendMessage(servo_conf) == 1)
        std::cout << servo_conf;
    else
        std::cerr << "unsupported: " << size_t(servo_conf.id()) << std::endl;

    // needs "#define DEBUGMSG" in MultiWii firmware

    msp::msg::DebugMessage debug_msg(fw_variant);
    if(client.sendMessage(debug_msg) == 1) {
        std::cout << "#Debug message:" << std::endl;
        std::cout << debug_msg.debug_msg << std::endl;
    }
    else
        std::cerr << "unsupported: " << size_t(debug_msg.id()) << std::endl;

    msp::msg::Debug debug(fw_variant);
    if(client.sendMessage(debug) == 1)
        std::cout << debug;
    else
        std::cerr << "unsupported: " << size_t(debug.id()) << std::endl;

    client.stop();
    std::cout << "PROGRAM COMPLETE" << std::endl;
}
