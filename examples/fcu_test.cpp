#include <FlightController.hpp>
#include <msp_msg.hpp>
#include <msg_print.hpp>

#include <iostream>

class App {
public:
    std::string name;

    App(std::string name) {
        this->name = name;
    }

    void onIdent(const msp::Ident& ident) {
        std::cout<<"Name: "<<name<<std::endl;
        std::cout<<ident;
    }

    void onStatus(const msp::Status& status) {
        std::cout<<"Name: "<<name<<std::endl;
        std::cout<<status;
    }

    void onImu(const msp::Imu& imu) {
        std::cout<<imu;
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

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const uint baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    fcu::FlightController fcu(device, baudrate);
    fcu.setAcc1G(512.0);
    fcu.setGyroUnit(1.0/4.096);
    fcu.setMagnGain(0.92/10.0); // for HMC5883L in default configuration (0.92 Mg/LSb)
    fcu.setStandardGravity(9.80665);

    // wait for connection
    fcu.initialise();

    App app("MultiWii");
    // define subscriptions with specific period
    fcu.subscribe(&App::onIdent, &app, 10);
    fcu.subscribe(&App::onStatus, &app, 1);
    // no period => requested Imu each time callbacks are checked
    fcu.subscribe(&App::onImu, &app);
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

    while(true) {
        fcu.sendRequests();
        fcu.handleRequests();
    }
}
