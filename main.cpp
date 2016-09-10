#include <iostream>
#include <MSP.hpp>

#include <unistd.h>

int main(int argc, char *argv[]) {
    std::cout << "Hello World!" << std::endl;

    MSP msp("/dev/ttyUSB0");

    std::vector<uint8_t> ret;
    uint8_t id;

    // try connecting until first package is received
    do {
        try {
            msp.sendRequest(MSP_IDENT);

            usleep(100);

            std::tie(ret, id) = msp.readData();

            if(id==MSP_IDENT) {
                msp::Ident ident;
                msp.unpack(ret, ident);
                std::cout<<"version: "<<(int)ident.version<<std::endl;
            }
            else {
                std::cout<<"wrong message:"<<(int)id<<std::endl;
            }
        }
        catch(boost::system::system_error &e) {
            std::cerr<<"Cannot reach MSP yet. "<<e.what()<<std::endl;
        }
    } while(id!=MSP_IDENT);


    while(true) {
        msp.sendRequest(MSP_STATUS);

        usleep(100);

        try {
            std::tie(ret, id) = msp.readData();

            if(id==MSP_STATUS) {
                msp::Status status;
                msp.unpack(ret, status);
                std::cout<<"time: "<<status.time<<std::endl;
            }
            else {
                std::cout<<"wrong message:"<<(int)id<<std::endl;
            }
        }
        catch(boost::system::system_error &e) {
            std::cerr << e.what() << std::endl;
        }
    }

    return 0;
}
