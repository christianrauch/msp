#include <FlightController.hpp>
#include <msp_msg.hpp>

#include <iostream>

void onIdent(msp::Ident ident) {
    std::cout<<"MSP version: "<<(int)ident.version<<std::endl;
}

void onStatus(msp::Status status) {
    std::cout<<"cycle time: "<<(int)status.time<<" us"<<std::endl;
    std::cout<<"error count: "<<(int)status.i2c_errors_count<<std::endl;
}

int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    msp::FlightController fcu(device);
    std::cout<<"MSP ready"<<std::endl;

    fcu.subscribe(msp::ID::MSP_IDENT, onIdent);
    fcu.subscribe(msp::ID::MSP_STATUS, onStatus);

    while(true) {
        fcu.handle();
    }
}
