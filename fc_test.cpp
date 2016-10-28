#include <FlightController.hpp>
#include <msp_msg.hpp>
#include <fcu_msg.hpp>

#include <iostream>

void onIdent(const msp::Ident* const ident) {
    std::cout<<"MSP version: "<<(int)ident->version<<std::endl;
}

void onIdentFCU(const fcu::Ident* const ident) {
    std::cout<<"MSP version: "<<ident->version<<std::endl;
    std::cout<<"type QUADX: "<<(ident->type==fcu::MultiType::QUADX)<<std::endl;
    std::cout<<"type QUADP: "<<(ident->type==fcu::MultiType::QUADP)<<std::endl;
}

void onStatus(const msp::Status* const status) {
    std::cout<<"cycle time: "<<(int)status->time<<" us"<<std::endl;
    std::cout<<"error count: "<<(int)status->i2c_errors_count<<std::endl;
}

void onStatusFCU(const fcu::Status &status) {
    std::cout<<"ACC?: "<<status.hasAccelerometer()<<std::endl;
    std::cout<<"GPS?: "<<status.hasGPS()<<std::endl;
}

void onRawImu(const msp::RawImu* const imu) {
    std::cout<<"imu in raw sensor specific units"<<std::endl;
    std::cout<<"acc (x,y,z): "<<imu->accx<<", "<<imu->accy<<", "<<imu->accz<<std::endl;
    std::cout<<"gyro (x,y,z): "<<imu->gyrx<<", "<<imu->gyry<<", "<<imu->gyrz<<std::endl;
    std::cout<<"magn (x,y,z): "<<imu->magx<<", "<<imu->magy<<", "<<imu->magz<<std::endl;
}

void onImu(const fcu::Imu* const imu) {
    std::cout<<"imu in real SI units"<<std::endl;
    std::cout<<"acc (x,y,z): "<<imu->acc[0]<<", "<<imu->acc[1]<<", "<<imu->acc[2]<<std::endl;
    std::cout<<"gyro (x,y,z): "<<imu->gyro[0]<<", "<<imu->gyro[1]<<", "<<imu->gyro[2]<<std::endl;
    std::cout<<"magn (x,y,z): "<<imu->magn[0]<<", "<<imu->magn[1]<<", "<<imu->magn[2]<<std::endl;
}

int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    fcu::FlightController fcu(device);
    std::cout<<"MSP ready"<<std::endl;
    fcu.setAcc1G(512.0);
    fcu.setGyroUnit(1.0/4096);

    fcu.subscribe(msp::ID::MSP_IDENT, onIdent);
//    fcu.subscribe(msp::ID::MSP_IDENT, onIdentFCU);
//    fcu.subscribe(msp::ID::MSP_IDENT, onStatus); // not typesafe
//    fcu.subscribe(msp::ID::MSP_STATUS, onStatus);
//    fcu.subscribe(msp::ID::MSP_STATUS, onStatusFCU);
//    fcu.subscribe(msp::ID::MSP_RAW_IMU, onRawImu);
    fcu.subscribe(msp::ID::MSP_RAW_IMU, onImu);

    while(true) {
        fcu.handle();
    }
}
