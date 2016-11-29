#include <FlightController.hpp>

#include <iostream>
#include <chrono>


int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    std::chrono::high_resolution_clock::time_point start, end;

    fcu::FlightController fcu(device);

    // wait until connection is established
    // get unique box IDs
    start = std::chrono::high_resolution_clock::now();
    fcu.initialise();
    end = std::chrono::high_resolution_clock::now();
    std::cout<<"ready after: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<" ms"<<std::endl;

    std::cout<<"Armed? "<<fcu.isArmed()<<std::endl;

    // arm the FC
    std::cout<<"Arming..."<<std::endl;
    start = std::chrono::high_resolution_clock::now();
    fcu.arm_block();
    end = std::chrono::high_resolution_clock::now();

    if(fcu.isArmed()) {
        std::cout<<"armed after: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<" ms"<<std::endl;
    }

    // disarm the FC
    std::cout<<"Disarming..."<<std::endl;
    start = std::chrono::high_resolution_clock::now();
    fcu.disarm_block();
    end = std::chrono::high_resolution_clock::now();

    if(!fcu.isArmed()) {
        std::cout<<"disarmed after: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<" ms"<<std::endl;
    }
}
