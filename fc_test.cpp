#include <FlightController.hpp>
#include <msp_msg.hpp>
#include <msp_msg.hpp>

#include <iostream>

#include <msg_print.hpp>

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
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    fcu::FlightController fcu(device);
    fcu.setAcc1G(512.0);
    fcu.setGyroUnit(1.0/4096);
    fcu.setMagnGain(1090.0/100.0);
    fcu.setStandardGravity(9.80665);
    fcu.populate_database();

    sleep(8);
    std::cout<<"MSP ready"<<std::endl;


    App app("MultiWii");
    fcu.subscribe(&App::onIdent, &app);
    fcu.subscribe(&App::onIdent, &app);
    fcu.subscribe(&App::onStatus, &app);
    fcu.subscribe(&App::onImu, &app);
    fcu.subscribe(&App::onServo, &app);
    fcu.subscribe(&App::onMotor, &app);
    fcu.subscribe(&App::onRc, &app);
    // TODO: RawGPS
    // TODO: CompGPS
    fcu.subscribe(&App::onAttitude, &app);
    fcu.subscribe(&App::onAltitude, &app);
    fcu.subscribe(&App::onAnalog, &app);
    fcu.subscribe(&App::onRcTuning, &app);
    fcu.subscribe(&App::onPID, &app);
    fcu.subscribe(&App::onBox, &app);
    fcu.subscribe(&App::onMisc, &app);
    fcu.subscribe(&App::onMotorPins, &app);
    fcu.subscribe(&App::onBoxNames, &app);
    fcu.subscribe(&App::onPidNames, &app);
    // TODO: WayPoint
    fcu.subscribe(&App::onBoxIds, &app);
    fcu.subscribe(&App::onServoConf, &app);
    // TODO: NavStatus
    // TODO: NavConfig
    fcu.subscribe(&App::onDebugMessage, &app);
    fcu.subscribe(&App::onDebug, &app);

    while(true) {
        fcu.handle();
    }
}
