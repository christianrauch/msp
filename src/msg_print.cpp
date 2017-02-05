#include "msg_print.hpp"
#include <iomanip>

typedef unsigned int uint;

std::ostream& operator<<(std::ostream& s, const msp::ApiVersion& api_version) {
    s << "#Api Version:" << std::endl;
    s << "API: " << api_version.major << "." << api_version.minor << std::endl;
    s << "Protocol: " << api_version.protocol << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::FcVariant& fc_variant) {
    s << "#FC variant:" << std::endl;
    s << "Identifier: " << fc_variant.identifier << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::FcVersion& fc_version) {
    s << "#FC version:" << std::endl;
    s << "Version: " << fc_version.major << "." << fc_version.minor << "." << fc_version.patch_level << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::BoardInfo& board_info) {
    s << "#Board Info:" << std::endl;
    s << "Identifier: " << board_info.identifier << std::endl;
    s << "Version: " << board_info.version << std::endl;
    s << "Type: " << uint(board_info.type) << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::BuildInfo& build_info) {
    s << "#Build Info:" << std::endl;
    s << "Date: " << build_info.buildDate << std::endl;
    s << "Time: " << build_info.buildTime << std::endl;
    s << "Git revision: " << build_info.shortGitRevision << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Feature& feature) {
    s << "#Features:" << std::endl;
    for(const std::string &f : feature.features) {
        s << f << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::RxMap& rx_map) {
    s << "#Channel mapping:" << std::endl;
    for(uint i(0); i<rx_map.map.size(); i++) {
        s << i << ": " << uint(rx_map.map[i]) << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Ident& ident) {
    std::string type;
    switch(ident.type) {
    case msp::MultiType::TRI:
        type = "Tricopter";
        break;
    case msp::MultiType::QUADP:
        type = "Quadrocopter Plus";
        break;
    case msp::MultiType::QUADX:
        type = "Quadrocopter X";
        break;
    case msp::MultiType::BI:
        type = "BI-copter";
        break;
    default:
        type = "UNDEFINED";
        break;
    }

    s << "#Ident:" << std::endl;

    s << "MultiWii Version: "<<ident.version << std::endl
      << "MSP Version: "<<ident.msp_version << std::endl
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

std::ostream& operator<<(std::ostream& s, const msp::Status& status) {
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

    s << "Active Boxes (by ID):";
    for(const uint box_id : status.active_box_id) {
        s << " " << box_id;
    }
    s << std::endl;

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Imu& imu) {
    s << "#Imu:" << std::endl;
    s << "Linear acceleration: " << imu.acc[0] << ", " << imu.acc[1] << ", " << imu.acc[2] << " m/s²" << std::endl;
    s << "Angular velocity: " << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2] << " deg/s" << std::endl;
    s << "Magnetometer: " << imu.magn[0] << ", " << imu.magn[1] << ", " << imu.magn[2] << " uT" << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Servo& servo) {
    s << "#Servo:" << std::endl;
    s << servo.servo[0] << " " << servo.servo[1] << " " << servo.servo[2] << " " << servo.servo[3] << std::endl;
    s << servo.servo[4] << " " << servo.servo[5] << " " << servo.servo[6] << " " << servo.servo[7] << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Motor& motor) {
    s << "#Motor:" << std::endl;
    s << motor.motor[0] << " " << motor.motor[1] << " " << motor.motor[2] << " " << motor.motor[3] << std::endl;
    s << motor.motor[4] << " " << motor.motor[5] << " " << motor.motor[6] << " " << motor.motor[7] << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Rc& rc) {
    s << "#Rc:" << std::endl;
    s << rc.roll << " " << rc.pitch << " " << rc.yaw << " " << rc.throttle << std::endl;
    s << rc.aux1 << " " << rc.aux2 << " " << rc.aux3 << " " << rc.aux4 << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Attitude& attitude) {
    s << "#Attitude:" << std::endl;
    s << "Ang : " << attitude.ang_x << ", " << attitude.ang_y << " deg" << std::endl;
    s << "Heading: " << attitude.heading << " deg" << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Altitude& altitude) {
    s << "#Altitude:" << std::endl;
    s << "Altitude: " << altitude.altitude << " m, var: " << altitude.vario << " m/s" << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Analog& analog) {
    s << "#Analog:" << std::endl;
    s << "Battery Voltage: " << analog.vbat << " V" << std::endl;
    s << "Current: " << analog.amperage << " A" << std::endl;
    s << "Power consumption: " << analog.powerMeterSum << " Ah" << std::endl;
    s << "RSSI: " << analog.rssi << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::RcTuning& rc_tuning) {
    s << "#Rc Tuning:" << std::endl;
    s << "Rc Rate: " << rc_tuning.RC_RATE << std::endl;
    s << "Rc Expo: " << rc_tuning.RC_EXPO << std::endl;
    s << "Roll/Pitch Rate: " << rc_tuning.RollPitchRate << std::endl;
    s << "Yaw Rate: " << rc_tuning.YawRate << std::endl;

    s << "Dynamic Throttle PID: " << rc_tuning.DynThrPID << std::endl;
    s << "Throttle MID: " << rc_tuning.Throttle_MID << std::endl;
    s << "Throttle Expo: " << rc_tuning.Throttle_EXPO << std::endl;

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Pid& pid) {
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

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Box& box) {
    s << "#Box:" << std::endl;
    for(uint ibox(0); ibox<box.box_pattern.size(); ibox++) {
        s << ibox << " ";
        for(uint iaux(0); iaux<box.box_pattern[ibox].size(); iaux++) {
            s << "aux" << iaux+1 << ": ";
            if(box.box_pattern[ibox][iaux].count(msp::SwitchPosition::LOW))
                s << "L";
            else
                s << "_";
            if(box.box_pattern[ibox][iaux].count(msp::SwitchPosition::MID))
                s << "M";
            else
                s << "_";
            if(box.box_pattern[ibox][iaux].count(msp::SwitchPosition::HIGH))
                s << "H";
            else
                s << "_";
            s << ", ";
        }
        s << std::endl;
    }

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Misc& misc) {
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

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::MotorPins& pin) {
    s << "#Motor pins:" << std::endl;
    for(uint imotor(0); imotor<msp::N_MOTOR; imotor++) {
        s << "Motor " << imotor << ": pin " << (uint)pin.pwm_pin[imotor] << std::endl;
    }

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::BoxNames& box_names) {
    s << "#Box names:" << std::endl;
    for(uint ibox(0); ibox<box_names.box_names.size(); ibox++) {
        s << ibox << ": " << box_names.box_names[ibox] << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::PidNames& pid_names) {
    s << "#PID names:" << std::endl;
    for(uint ipid(0); ipid<pid_names.pid_names.size(); ipid++) {
        s << ipid << ": " << pid_names.pid_names[ipid] << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::BoxIds& box_ids) {
    s << "#Box IDs:" << std::endl;
    for(uint ibox(0); ibox<box_ids.box_ids.size(); ibox++) {
        s << ibox << ": " << (uint)box_ids.box_ids[ibox] << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::ServoConf& servo_conf) {
    s << "#Servo conf:" << std::endl;
    s << "Nr. | [min | middle | max] (rate)" << std::endl;
    for(uint iservo(0); iservo<msp::N_SERVO; iservo++) {
        const msp::ServoConfRange servo = servo_conf.servo_conf[iservo];
        s << iservo << ":  | "<< "["<< servo.min <<" | "<< servo.middle <<" | "<< servo.max <<"] ("<< (uint)servo.rate <<")"  << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::Debug& debug) {
    s << "#Debug:" << std::endl;
    s << "debug1: " << debug.debug1 << std::endl;
    s << "debug2: " << debug.debug2 << std::endl;
    s << "debug3: " << debug.debug3 << std::endl;
    s << "debug4: " << debug.debug4 << std::endl;
    return s;
}
