/**
  Example for demonstrating direct motor access.

  REMOVE YOUR PROPELLERS BEFORE EXECUTING THIS PROGRAM!

  DYNBALANCE needs to be activated to gain direct access.
  (set "#define DYNBALANCE" in MultiWii's 'config.h')

  The motor values are directly passed to the ESCs instead of computing them by
  RC commands. Hence, there is no need to arm or disarm the flight controller.
  In fact, it is not possible to control the motors by RC signals if DYNBALANCE
  is active.
**/

#include <FlightController.hpp>

int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    fcu::FlightController fcu(device);
    fcu.initialise();

    // spin motors 1 to 4
    fcu.setMotors({1100,1100,1100,1100,0,0,0,0});

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // stop motors
    fcu.setMotors({1000,1000,1000,1000,1000,1000,1000,1000});
}
