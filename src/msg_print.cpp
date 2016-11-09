#include "msg_print.hpp"
#include <iomanip>

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

std::ostream& operator<<(std::ostream& s, const fcu::Attitude& attitude) {
    s << "#Attitude:" << std::endl;
    s << "Ang : " << attitude.ang_x << ", " << attitude.ang_y << " deg" << std::endl;
    s << "Heading: " << attitude.heading << " deg" << std::endl;
}

std::ostream& operator<<(std::ostream& s, const fcu::Altitude& altitude) {
    s << "#Altitude:" << std::endl;
    s << "Altitude: " << altitude.altitude << " m, var: " << altitude.vario << " m/s" << std::endl;
}

std::ostream& operator<<(std::ostream& s, const fcu::Analog& analog) {
    s << "#Analog:" << std::endl;
    s << "Battery Voltage: " << analog.vbat << " V" << std::endl;
    s << "Current: " << analog.amperage << " A" << std::endl;
    s << "Power consumption: " << analog.powerMeterSum << " Ah" << std::endl;
    s << "RSSI: " << analog.rssi << std::endl;
}

std::ostream& operator<<(std::ostream& s, const msp::RcTuning& rc_tuning) {
    s << "#Rc Tuning:" << std::endl;
    s << "Rc Rate: " << (uint)rc_tuning.RC_RATE << std::endl;
    s << "Rc Expo: " << (uint)rc_tuning.RC_EXPO << std::endl;
    s << "Roll/Pitch Rate: " << (uint)rc_tuning.RollPitchRate << std::endl;
    s << "Yaw Rate: " << (uint)rc_tuning.YawRate << std::endl;

    s << "Dynamic Throttle PID: " << (uint)rc_tuning.DynThrPID << std::endl;
    s << "Throttle MID: " << (uint)rc_tuning.Throttle_MID << std::endl;
    s << "Throttle Expo: " << (uint)rc_tuning.Throttle_EXPO << std::endl;
}

std::ostream& operator<<(std::ostream& s, const fcu::PID& pid) {
    s << std::setprecision(3);
    s << "#PID:" << std::endl;
    s << "Name      P     | I     | D     |" << std::endl;
    s << "----------------|-------|-------|" << std::endl;
    s << "Roll:      " << pid.roll.P << "\t| " << pid.roll.I << "\t| " << pid.roll.D << std::endl;
    s << "Pitch:     " << pid.pitch.P << "\t| " << pid.pitch.I << "\t| " << pid.pitch.D << std::endl;
    s << "Yaw:       " << pid.yaw.P << "\t| " << pid.yaw.I << "\t| " << pid.yaw.D << std::endl;
    s << "Altitude:  " << pid.alt.P << "\t| " << pid.alt.I << "\t| " << pid.alt.D << std::endl;

    s << "Position:  " << pid.pos.P << "\t| " << pid.pos.I << "\t| " << pid.pos.D << std::endl;
    s << "PositionR: " << pid.posr.P << "\t| " << pid.posr.I << "\t| " << pid.posr.D << std::endl;
    s << "NavR:      " << pid.navr.P << "\t| " << pid.navr.I << "\t| " << pid.navr.D << std::endl;
    s << "Level:     " << pid.level.P << "\t| " << pid.level.I << "\t| " << pid.level.D << std::endl;
    s << "Magn:      " << pid.mag.P << "\t| " << pid.mag.I << "\t| " << pid.mag.D << std::endl;
    s << "Vel:       " << pid.vel.P << "\t| " << pid.vel.I << "\t| " << pid.vel.D << std::endl;
}

std::ostream& operator<<(std::ostream& s, const fcu::Misc& misc) {
    s << "#Miscellaneous:" << std::endl;
    s << "Power Trigger: " << misc.powerTrigger << std::endl;
    s << "Min Throttle: " << misc.minThrottle << std::endl;
    s << "Max Throttle: " << misc.maxThrottle << std::endl;
    s << "Failsafe Throttle: " << misc.failsafeThrottle << std::endl;

    s << "Arm Counter: " << misc.arm << std::endl;
    s << "Lifetime: " << misc.lifetime << std::endl;

    s << "Magnetic Declination: " << misc.mag_declination << " deg" << std::endl;
    s << "Battery Voltage Scale: " << misc.vbatScale << " V" << std::endl;
    s << "Battery Warning Level 1: " << misc.vbatLevelWarn1 << " V" << std::endl;
    s << "Battery Warning Level 2: " << misc.vbatLevelWarn2 << " V" << std::endl;
    s << "Battery Critical Level: " << misc.vbatLevelCrit << " V" << std::endl;
}

std::ostream& operator<<(std::ostream& s, const fcu::Box& box) {
    s << "#Box:" << std::endl;
    for(uint ibox(0); ibox<box.boxs.size(); ibox++) {
        s << ibox << " ";
        for(uint iaux(0); iaux<box.boxs[ibox].size(); iaux++) {
            s << "aux" << iaux+1 << ": ";
            if(box.boxs[ibox][iaux].count(fcu::SwitchPosition::LOW))
                s << "L";
            else
                s << "_";
            if(box.boxs[ibox][iaux].count(fcu::SwitchPosition::MID))
                s << "M";
            else
                s << "_";
            if(box.boxs[ibox][iaux].count(fcu::SwitchPosition::HIGH))
                s << "H";
            else
                s << "_";
            s << ", ";
        }
        s << std::endl;
    }
}
