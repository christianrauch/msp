#include <iostream>
#include <MSP.hpp>

#include <unistd.h>

int main(int argc, char *argv[]) {
    std::cout << "Hello World!" << std::endl;

    MSP msp("/dev/ttyUSB0");

    std::vector<uint8_t> ret(0);
    //uint8_t id;

    sleep(10);

    std::cout<<"ready"<<std::endl;

    // try connecting until first package is received
    msp::Ident ident;
    do {
        try {
            msp.sendData(ident.id);

            usleep(100);

            ret = msp.receiveData(ident.id);
            ident.decode(ret);
        }
        catch(std::exception &e) {
            std::cerr<<"Cannot reach MSP yet. "<<e.what()<<std::endl;
        }
    } while(ret.size()==0);

    std::cout<<"version: "<<(int)ident.version<<std::endl;


    while(true) {
        msp::Status status;
        msp.sendData(status.id);

        usleep(100);

        try {
            ret = msp.receiveData(status.id);
            status.decode(ret);
            std::cout<<"time: "<<status.time<<std::endl;
        }
        catch(std::exception &e) {
            std::cerr << e.what() << std::endl;
        }
    }

    return 0;
}
