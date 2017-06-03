#include <Client.hpp>
#include <iostream>

#include <msp_msg.hpp>
#include <msg_print.hpp>

struct SubCallbacks {
    void onIdent(const msp::Ident& ident) {
        std::cout<<ident;
    }

    void onStatus(const msp::Status& status) {
        std::cout<<status;
    }

    void onImu(const msp::ImuRaw& imu) {
        std::cout<<msp::ImuSI(imu, 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);
    }

    void onServo(const msp::Servo& servo) {
        std::cout<<servo;
    }

    void onMotor(const msp::Motor& motor) {
        std::cout<<motor;
    }

    void onRc(const msp::Rc& rc) {
        std::cout<<rc;
    }

    void onAttitude(const msp::Attitude& attitude) {
        std::cout<<attitude;
    }

    void onAltitude(const msp::Altitude& altitude) {
        std::cout<<altitude;
    }

    void onAnalog(const msp::Analog& analog) {
        std::cout<<analog;
    }

    void onRcTuning(const msp::RcTuning& rc_tuning) {
        std::cout<<rc_tuning;
    }

    void onPID(const msp::Pid& pid) {
        std::cout<<pid;
    }

    void onBox(const msp::Box& box) {
        std::cout<<box;
    }

    void onMisc(const msp::Misc& misc) {
        std::cout<<misc;
    }

    void onMotorPins(const msp::MotorPins& motor_pins) {
        std::cout<<motor_pins;
    }

    void onBoxNames(const msp::BoxNames& box_names) {
        std::cout<<box_names;
    }

    void onPidNames(const msp::PidNames& pid_names) {
        std::cout<<pid_names;
    }

    void onBoxIds(const msp::BoxIds& box_ids) {
        std::cout<<box_ids;
    }

    void onServoConf(const msp::ServoConf& servo_conf) {
        std::cout<<servo_conf;
    }

    void onDebugMessage(const msp::DebugMessage& debug_msg) {
        std::cout<<"#Debug message:"<<std::endl;
        std::cout<<debug_msg.msg<<std::endl;
    }

    void onDebug(const msp::Debug& debug) {
        std::cout<<debug;
    }
};

static bool running = true;

void onExit(int /*signal*/) { running = false; }

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const uint baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    SubCallbacks subs;

    msp::client::Client client;
    client.connect(device, baudrate);
    client.start();

    client.subscribe(&SubCallbacks::onImu, &subs, 0.1);
    client.subscribe(&SubCallbacks::onIdent, &subs, 10);
    client.subscribe(&SubCallbacks::onStatus, &subs, 1);
    client.subscribe(&SubCallbacks::onServo, &subs, 0.1);
    client.subscribe(&SubCallbacks::onMotor, &subs, 0.1);
    client.subscribe(&SubCallbacks::onRc, &subs, 0.1);
    client.subscribe(&SubCallbacks::onAttitude, &subs);
    client.subscribe(&SubCallbacks::onAltitude, &subs);
    client.subscribe(&SubCallbacks::onAnalog, &subs, 10);
    client.subscribe(&SubCallbacks::onRcTuning, &subs, 20);
    client.subscribe(&SubCallbacks::onPID, &subs, 20);
    client.subscribe(&SubCallbacks::onBox, &subs, 1);
    client.subscribe(&SubCallbacks::onMisc, &subs, 1);
    client.subscribe(&SubCallbacks::onMotorPins, &subs, 20);
    client.subscribe(&SubCallbacks::onBoxNames, &subs, 20);
    client.subscribe(&SubCallbacks::onPidNames, &subs, 20);
    client.subscribe(&SubCallbacks::onBoxIds, &subs, 20);
    client.subscribe(&SubCallbacks::onServoConf, &subs, 20);
    client.subscribe(&SubCallbacks::onDebugMessage, &subs,1);
    client.subscribe(&SubCallbacks::onDebug, &subs, 1);

    // we need to keep the main thread running to execute callbacks
    // stop with SIGINT (Ctrl+C)
    std::signal(SIGINT, onExit);
    while(running);

    client.stop();

    std::cout << "DONE" << std::endl;
}
