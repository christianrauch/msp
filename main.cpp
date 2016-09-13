#include <iostream>
#include <MSP.hpp>

#include <unistd.h>

int main(int argc, char *argv[]) {
    std::cout << "Hello World!" << std::endl;

    msp::MSP msp("/dev/ttyUSB0");

    //sleep(10);

    std::cout<<"ready"<<std::endl;

    bool success;

    // try connecting until first package is received
    msp::Ident ident;

    do {
        try {
            success = msp.request(ident);
            std::cout<<"success: "<<success<<std::endl;
        }
        catch(boost::system::system_error &e) {
            //std::cerr<<"Cannot reach MSP yet. "<<e.what()<<std::endl;
        }

    } while(success==false);

    std::cout<<"version: "<<(int)ident.version<<std::endl;


    while(true) {
        msp::Status status;
        try {
            success = msp.request(status);
        }
        catch(boost::system::system_error &e) {
            //std::cerr<<"No data. "<<e.what()<<std::endl;
        }

        if(success) {
            std::cout<<"time: "<<status.time<<" us"<<std::endl;
            success = false;
        }
    }

    return 0;
}
