/**
  Example for demonstrating arming and disarming functionality.

  REMOVE YOUR PROPELLERS BEFORE EXECUTING THIS PROGRAM!

  DYNBALANCE needs to be deactivated to process RC commands from any source.
  (comment "#define DYNBALANCE" in MultiWii's 'config.h')

  Dis-/Arming is achieved by sending RC commands to the flight controller. The
  arming and disarming methods will block until they receive the expected arming
  status as feedback.
  If the FC is armed and no other source of RC commands (physical remote control
  or MSP_SET_RAW_RC packages) is provided, the FC will go into fail-safe mode and
  motors will turn!
**/

#include <FlightController.hpp>

#include <iostream>
#include <chrono>


int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const uint baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    std::chrono::high_resolution_clock::time_point start, end;
    bool feature_changed = false;
start:
    fcu::FlightController fcu(device, baudrate);

    // wait until connection is established
    // get unique box IDs
    start = std::chrono::high_resolution_clock::now();
    fcu.initialise();
    end = std::chrono::high_resolution_clock::now();
    std::cout<<"ready after: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<" ms"<<std::endl;

    // on cleanflight, we need to enable the "RX_MSP" feature
    if(fcu.isFirmwareCleanflight()) {
        if(fcu.enableRxMSP()==1) {
            std::cout<<"RX_MSP enabled, restart"<<std::endl;
            feature_changed = true;
            goto start;
        }

        if(feature_changed) {
            // if we rebooted after updating the RX_MSP feature, we need to sleep for a while
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

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
