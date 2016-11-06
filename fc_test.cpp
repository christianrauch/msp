#include <FlightController.hpp>
#include <msp_msg.hpp>
#include <fcu_msg.hpp>

#include <iostream>

#include <msg_print.hpp>

class App {
public:
    std::string name;

    App(std::string name) {
        this->name = name;
    }

    void onIdentFCU(const fcu::Ident* const ident) {
        std::cout<<"Name: "<<name<<std::endl;
        std::cout<<(*ident);
    }

    void onStatusFCU(const fcu::Status *status) {
        std::cout<<"Name: "<<name<<std::endl;
        std::cout<<(*status);
    }

    void onRawImu(const msp::RawImu* const imu) {
        std::cout<<"imu in raw sensor specific units"<<std::endl;
        std::cout<<"acc (x,y,z): "<<imu->accx<<", "<<imu->accy<<", "<<imu->accz<<std::endl;
        std::cout<<"gyro (x,y,z): "<<imu->gyrx<<", "<<imu->gyry<<", "<<imu->gyrz<<std::endl;
        std::cout<<"magn (x,y,z): "<<imu->magx<<", "<<imu->magy<<", "<<imu->magz<<std::endl;
    }

    void onImu(const fcu::Imu* const imu) {
        std::cout<<"Name: "<<name<<std::endl;
        std::cout<<(*imu);
    }
};

int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    fcu::FlightController fcu(device);
    sleep(8);
    std::cout<<"MSP ready"<<std::endl;
    fcu.setAcc1G(512.0);
    fcu.setGyroUnit(1.0/4096);

    App app("MultiWii");
    fcu.subscribe(msp::ID::MSP_IDENT, &App::onIdentFCU, &app);
    fcu.subscribe(msp::ID::MSP_STATUS, &App::onStatusFCU, &app);
    fcu.subscribe(msp::ID::MSP_RAW_IMU, &App::onImu, &app);

    while(true) {
        fcu.handle();
    }
}
