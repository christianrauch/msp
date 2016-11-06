#include "msg_print.hpp"

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
    s << "Magnetometer: " << imu.magn[0] << ", " << imu.magn[1] << ", " << imu.magn[2] << std::endl;
}

std::ostream& operator<<(std::ostream& s, const msp::Servo& servo) {
    s << "#Servo:" << std::endl;
    s << servo.servo[0] << " " << servo.servo[1] << " " << servo.servo[2] << " " << servo.servo[3] << std::endl;
    s << servo.servo[4] << " " << servo.servo[5] << " " << servo.servo[6] << " " << servo.servo[7] << std::endl;
}

std::ostream& operator<<(std::ostream& s, const msp::Motor& motor) {
    s << "#Motor:" << std::endl;
    s << motor.motor[0] << " " << motor.motor[1] << " " << motor.motor[2] << " " << motor.motor[3] << std::endl;
    s << motor.motor[4] << " " << motor.motor[5] << " " << motor.motor[6] << " " << motor.motor[7] << std::endl;
}

std::ostream& operator<<(std::ostream& s, const msp::Rc& rc) {
    s << "#Rc:" << std::endl;
    s << rc.roll << " " << rc.pitch << " " << rc.yaw << " " << rc.throttle << std::endl;
    s << rc.aux1 << " " << rc.aux2 << " " << rc.aux3 << " " << rc.aux4 << std::endl;
}
