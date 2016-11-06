//#include <msp_msg.hpp>
//#include <fcu_msg.hpp>
//#include <ostream>
#include "msg_print.hpp"

namespace fcu {

std::ostream& operator<<(std::ostream& s, const fcu::Ident& ident) {
    std::string type;
    switch(ident.type) {
    case fcu::MultiType::TRI:
        type = "Tricopter";
        break;
    case fcu::MultiType::QUADP:
        type = "Quadrocopter Plus";
        break;
    case fcu::MultiType::QUADX:
        type = "Quadrocopter X";
        break;
    case fcu::MultiType::BI:
        type = "BI-copter";
        break;
    default:
        type = "UNDEFINED";
        break;
    }

    s << "#Ident:" << std::endl;

    s << "Version: "<<ident.version << std::endl
      << "Type: " << type << std::endl
      << "Capabilities:" << std::endl;

    s << "    Bind:   ";
    ident.hasBind() ? s<<"ON" : s<< "OFF";
    s << std::endl;

    s << "    DynBal: ";
    ident.hasDynBal() ? s<<"ON" : s<< "OFF";
    s << std::endl;

    s << "    Flap:   ";
    ident.hasFlap() ? s<<"ON" : s<< "OFF";
    s << std::endl;

    return s;
}

std::ostream& operator<<(std::ostream& s, const fcu::Status& status) {
    s << "#Status:" << std::endl;
    s << "Cycle time: " << status.time<< " us" << std::endl;
    s << "I2C errors: " << status.errors<< std::endl;
    s << "Sensors:" << std::endl;

    s << "    Accelerometer: ";
    status.hasAccelerometer() ? s<<"ON" : s<< "OFF";
    s << std::endl;

    s << "    Barometer: ";
    status.hasBarometer() ? s<<"ON" : s<< "OFF";
    s << std::endl;

    s << "    Magnetometer: ";
    status.hasMagnetometer() ? s<<"ON" : s<< "OFF";
    s << std::endl;

    s << "    GPS: ";
    status.hasGPS() ? s<<"ON" : s<< "OFF";
    s << std::endl;

    s << "    Sonar: ";
    status.hasSonar() ? s<<"ON" : s<< "OFF";
    s << std::endl;
}

std::ostream& operator<<(std::ostream& s, const fcu::Imu& imu) {
    s << "#Imu:" << std::endl;
    s << "Linear acceleration: " << imu.acc[0] << ", " << imu.acc[1] << ", " << imu.acc[2] << " g" << std::endl;
    s << "Angular velocity: " << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2] << " deg/s" << std::endl;
    s << "Magnetomoeter: " << imu.magn[0] << ", " << imu.magn[1] << ", " << imu.magn[2] << std::endl;
}

} // namespace fcu
