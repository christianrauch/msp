#include <Client.hpp>
#include <iostream>
#include <msp_msg.hpp>

struct SubCallbacks {
    void onIdent(const msp::msg::Ident& ident) { std::cout << ident; }

    void onStatus(const msp::msg::Status& status) { std::cout << status; }

    void onImu(const msp::msg::RawImu& imu) {
        std::cout << msp::msg::ImuSI(
            imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
    }

    void onServo(const msp::msg::Servo& servo) { std::cout << servo; }

    void onMotor(const msp::msg::Motor& motor) { std::cout << motor; }

    void onRc(const msp::msg::Rc& rc) { std::cout << rc; }

    void onAttitude(const msp::msg::Attitude& attitude) {
        std::cout << attitude;
    }

    void onAltitude(const msp::msg::Altitude& altitude) {
        std::cout << altitude;
    }

    void onAnalog(const msp::msg::Analog& analog) { std::cout << analog; }

    void onRcTuning(const msp::msg::RcTuning& rc_tuning) {
        std::cout << rc_tuning;
    }

    void onPID(const msp::msg::Pid& pid) { std::cout << pid; }

    void onBox(const msp::msg::ActiveBoxes& box) { std::cout << box; }

    void onMisc(const msp::msg::Misc& misc) { std::cout << misc; }

    void onMotorPins(const msp::msg::MotorPins& motor_pins) {
        std::cout << motor_pins;
    }

    void onBoxNames(const msp::msg::BoxNames& box_names) {
        std::cout << box_names;
    }

    void onPidNames(const msp::msg::PidNames& pid_names) {
        std::cout << pid_names;
    }

    void onBoxIds(const msp::msg::BoxIds& box_ids) { std::cout << box_ids; }

    void onServoConf(const msp::msg::ServoConf& servo_conf) {
        std::cout << servo_conf;
    }

    void onDebugMessage(const msp::msg::DebugMessage& debug_msg) {
        std::cout << "#Debug message:" << std::endl;
        std::cout << debug_msg.debug_msg << std::endl;
    }

    void onDebug(const msp::msg::Debug& debug) { std::cout << debug; }
};

static bool running = true;

void onExit(int /*signal*/) { running = false; }

int main(int argc, char* argv[]) {
    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    SubCallbacks subs;

    msp::client::Client client;
    client.start(device, baudrate);

    // using lambda callback with stored function object
    const std::function<void(const msp::msg::RawImu&)> imu_cb2 =
        [](const msp::msg::RawImu& imu) {
            std::cout << imu;
            std::cout << msp::msg::ImuSI(
                imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
        };
    client.subscribe(imu_cb2, 0.1);

    client.subscribe(&SubCallbacks::onIdent, &subs, 10);
    client.subscribe(&SubCallbacks::onStatus, &subs, 1);
    client.subscribe(&SubCallbacks::onServo, &subs, 0.1);
    client.subscribe(&SubCallbacks::onMotor, &subs, 0.1);
    client.subscribe(&SubCallbacks::onRc, &subs, 0.1);
    client.subscribe(&SubCallbacks::onAttitude, &subs, 0.1);
    client.subscribe(&SubCallbacks::onAltitude, &subs, 0.1);
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
    client.subscribe(&SubCallbacks::onDebugMessage, &subs, 1);
    client.subscribe(&SubCallbacks::onDebug, &subs, 1);

    // Ctrl+C to quit
    std::cin.get();

    client.stop();

    std::cout << "DONE" << std::endl;
}
