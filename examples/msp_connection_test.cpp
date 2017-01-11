#include <iostream>
#include <MSP.hpp>
#include <msp_msg.hpp>

#include <chrono>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const uint baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    msp::MSP msp(device, baudrate);
    msp.setWait(1);

    // try connecting until first package is received
    {
    std::cout<<"Waiting for flight controller to become ready..."<<std::endl;
    auto start = std::chrono::steady_clock::now();
    msp::Ident ident;
    msp.request_wait(ident, 10);
    auto end = std::chrono::steady_clock::now();

    std::cout<<"MSP version "<<(int)ident.version<<" ready after: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<" ms"<<std::endl;
    }

    // test update rate for reading Gyro messages
    {
    const unsigned int max_msg = 1000;
    unsigned int n_msg = 0;
    auto start = std::chrono::steady_clock::now();
    while(n_msg!=max_msg) {
        msp::Imu status;
        msp.request_block(status);
        n_msg++;
    }
    auto end = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

    std::cout<<"read "<<max_msg<<" messages in: "<<duration<<" ms"<<std::endl;
    std::cout<<"messages per second: "<<max_msg/(duration/1000.0)<<" Hz"<<std::endl;
    }

    return 0;
}
