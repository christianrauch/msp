#include <FlightController.hpp>
#include <msp_msg.hpp>
#include <msg_print.hpp>

#include <iostream>

class App {
public:
    std::string name;

    App(const std::string name, const float acc_1g, const float gyro_unit, const float magn_gain, const float si_unit_1g) : acc_1g(acc_1g), gyro_unit(gyro_unit), magn_gain(magn_gain), si_unit_1g(si_unit_1g) {
        this->name = name;
    }

    void onIdent(const msp::msg::Ident& ident) {
        std::cout<<"Name: "<<name<<std::endl;
        std::cout<<ident;
    }

    void onStatus(const msp::msg::Status& status) {
        std::cout<<"Name: "<<name<<std::endl;
        std::cout<<status;
    }

    void onImu(const msp::msg::ImuRaw& imu_raw) {
        std::cout<<msp::msg::ImuSI(imu_raw, acc_1g, gyro_unit, magn_gain, si_unit_1g);
    }

    void onServo(const msp::msg::Servo& servo) {
        std::cout<<servo;
    }

    void onMotor(const msp::msg::Motor& motor) {
        std::cout<<motor;
    }

    void onRc(const msp::msg::Rc& rc) {
        std::cout<<rc;
    }

    void onAttitude(const msp::msg::Attitude& attitude) {
        std::cout<<attitude;
    }

    void onAltitude(const msp::msg::Altitude& altitude) {
        std::cout<<altitude;
    }

    void onAnalog(const msp::msg::Analog& analog) {
        std::cout<<analog;
    }

    void onRcTuning(const msp::msg::RcTuning& rc_tuning) {
        std::cout<<rc_tuning;
    }

    void onPID(const msp::msg::Pid& pid) {
        std::cout<<pid;
    }

    void onBox(const msp::msg::Box& box) {
        std::cout<<box;
    }

    void onMisc(const msp::msg::Misc& misc) {
        std::cout<<misc;
    }

    void onMotorPins(const msp::msg::MotorPins& motor_pins) {
        std::cout<<motor_pins;
    }

    void onBoxNames(const msp::msg::BoxNames& box_names) {
        std::cout<<box_names;
    }

    void onPidNames(const msp::msg::PidNames& pid_names) {
        std::cout<<pid_names;
    }

    void onBoxIds(const msp::msg::BoxIds& box_ids) {
        std::cout<<box_ids;
    }

    void onServoConf(const msp::msg::ServoConf& servo_conf) {
        std::cout<<servo_conf;
    }

    void onDebugMessage(const msp::msg::DebugMessage& debug_msg) {
        std::cout<<"#Debug message:"<<std::endl;
        std::cout<<debug_msg.msg<<std::endl;
    }

    void onDebug(const msp::msg::Debug& debug) {
        std::cout<<debug;
    }

private:
    const float acc_1g;
    const float gyro_unit;
    const float magn_gain;
    const float si_unit_1g;
};

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const uint baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    fcu::FlightController fcu(device, baudrate);

    // wait for connection
    fcu.initialise();

    App app("MultiWii", 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);
    // define subscriptions with specific period
    fcu.subscribe(&App::onIdent, &app, 10);
    fcu.subscribe(&App::onStatus, &app, 1);
    // no period => requested Imu each time callbacks are checked
    fcu.subscribe(&App::onImu, &app, 0.1);
    fcu.subscribe(&App::onServo, &app, 0.1);
    fcu.subscribe(&App::onMotor, &app, 0.1);
    fcu.subscribe(&App::onRc, &app, 0.1);
    // TODO: RawGPS
    // TODO: CompGPS
    fcu.subscribe(&App::onAttitude, &app);
    fcu.subscribe(&App::onAltitude, &app);
    fcu.subscribe(&App::onAnalog, &app, 10);
    fcu.subscribe(&App::onRcTuning, &app, 20);
    fcu.subscribe(&App::onPID, &app, 20);
    fcu.subscribe(&App::onBox, &app, 1);
    fcu.subscribe(&App::onMisc, &app, 1);
    fcu.subscribe(&App::onMotorPins, &app, 20);
    fcu.subscribe(&App::onBoxNames, &app, 20);
    fcu.subscribe(&App::onPidNames, &app, 20);
    // TODO: WayPoint
    fcu.subscribe(&App::onBoxIds, &app, 20);
    fcu.subscribe(&App::onServoConf, &app, 20);
    // TODO: NavStatus
    // TODO: NavConfig
    fcu.subscribe(&App::onDebugMessage, &app,1);
    fcu.subscribe(&App::onDebug, &app, 1);

    // Ctrl+C to quit
    std::cin.get();
}
