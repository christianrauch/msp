#include <FlightController.hpp>
#include <msp_msg.hpp>
#include <fcu_msg.hpp>

#include <iostream>

//void onIdent(const msp::Ident &ident) {
//    std::cout<<"MSP version: "<<(int)ident.version<<std::endl;
//}

void onIdent(const msp::Ident* const ident) {
    std::cout<<"MSP version: "<<(int)ident->version<<std::endl;
}

//void onIdentFCU(const fcu::Ident &ident) {
//    std::cout<<"MSP version: "<<ident.version<<std::endl;
//    std::cout<<"type QUADX: "<<(ident.type==fcu::MultiType::QUADX)<<std::endl;
//    std::cout<<"type QUADP: "<<(ident.type==fcu::MultiType::QUADP)<<std::endl;
//}

void onIdentFCU(const fcu::Ident* const ident) {
    std::cout<<"MSP version: "<<ident->version<<std::endl;
    std::cout<<"type QUADX: "<<(ident->type==fcu::MultiType::QUADX)<<std::endl;
    std::cout<<"type QUADP: "<<(ident->type==fcu::MultiType::QUADP)<<std::endl;
}

//void onStatus(const msp::Status &status) {
//    std::cout<<"cycle time: "<<(int)status.time<<" us"<<std::endl;
//    std::cout<<"error count: "<<(int)status.i2c_errors_count<<std::endl;
//}

void onStatus(const msp::Status* const status) {
    std::cout<<"cycle time: "<<(int)status->time<<" us"<<std::endl;
    std::cout<<"error count: "<<(int)status->i2c_errors_count<<std::endl;
}

void onStatusFCU(const fcu::Status &status) {
    std::cout<<"ACC?: "<<status.hasAccelerometer()<<std::endl;
    std::cout<<"GPS?: "<<status.hasGPS()<<std::endl;
}

int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    fcu::FlightController fcu(device);
    std::cout<<"MSP ready"<<std::endl;

    fcu.subscribe(msp::ID::MSP_IDENT, onIdent);
//    fcu.subscribe(msp::ID::MSP_IDENT, onIdentFCU);
//    fcu.subscribe(msp::ID::MSP_IDENT, onStatus); // not typesafe
//    fcu.subscribe(msp::ID::MSP_STATUS, onStatus);
//    fcu.subscribe(msp::ID::MSP_STATUS, onStatusFCU);

    while(true) {
        fcu.handle();
    }
}
