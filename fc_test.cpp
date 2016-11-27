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

    // wait for connection
    fcu.waitForConnection();
    std::cout<<"MSP ready"<<std::endl;


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
        //fcu.handle();
        fcu.sendRequests();
        fcu.handleRequests();
    }
}
