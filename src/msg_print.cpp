#include "msg_print.hpp"
#include <iomanip>

std::ostream& operator<<(std::ostream& s, const msp::msg::ApiVersion& api_version) {
    s << "#Api Version:" << std::endl;
    s << "API: " << api_version.major() << "." << api_version.minor() << std::endl;
    s << "Protocol: " << api_version.protocol() << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::FcVariant& fc_variant) {
    s << "#FC variant:" << std::endl;
    s << "Identifier: " << fc_variant.identifier() << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::FcVersion& fc_version) {
    s << "#FC version:" << std::endl;
    s << "Version: " << (uint32_t)fc_version.major() << "." << (uint32_t)fc_version.minor() << "." << (uint32_t)fc_version.patch_level() << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::BoardInfo& board_info) {
    s << "#Board Info:" << std::endl;
    s << "Identifier: " << board_info.identifier() << std::endl;
    s << "Version: " << board_info.version() << std::endl;
    s << "OSD support: " << size_t(board_info.osd_support()) << std::endl;
    s << "Comms bitmask: " << size_t(board_info.comms_capabilites()) << std::endl;
    s << "Board Name: " << board_info.name() << std::endl;
    
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::BuildInfo& build_info) {
    s << "#Build Info:" << std::endl;
    s << "Date: " << build_info.buildDate << std::endl;
    s << "Time: " << build_info.buildTime << std::endl;
    s << "Git revision: " << build_info.shortGitRevision << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Feature& feature) {
    s << "#Features:" << std::endl;
    for(const std::string &f : feature.features) {
        s << f << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::RxMap& rx_map) {
    s << "#Channel mapping:" << std::endl;
    for(size_t i(0); i<rx_map.map.size(); i++) {
        s << i << ": " << size_t(rx_map.map[i]) << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Ident& ident) {
    std::string type;
    switch(ident.type) {
    case msp::msg::MultiType::TRI:
        type = "Tricopter";
        break;
    case msp::msg::MultiType::QUADP:
        type = "Quadrocopter Plus";
        break;
    case msp::msg::MultiType::QUADX:
        type = "Quadrocopter X";
        break;
    case msp::msg::MultiType::BI:
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

std::ostream& operator<<(std::ostream& s, const msp::msg::Status& status) {
    s << "#Status:" << std::endl;
    s << "Cycle time: " << status.cycle_time<< " us" << std::endl;
    s << "I2C errors: " << status.i2c_errors<< std::endl;
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
    for(const size_t box_id : status.box_mode_flags) {
        s << " " << box_id;
    }
    s << std::endl;

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::ImuRaw& imu) {
    s << "#Imu:" << std::endl;
    s << "Linear acceleration: " << imu.acc[0] << ", " << imu.acc[1] << ", " << imu.acc[2] << std::endl;
    s << "Angular velocity: " << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2] << std::endl;
    s << "Magnetometer: " << imu.mag[0] << ", " << imu.mag[1] << ", " << imu.mag[2] << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::ImuSI& imu) {
    s << "#Imu:" << std::endl;
    s << "Linear acceleration: " << imu.acc[0] << ", " << imu.acc[1] << ", " << imu.acc[2] << " m/s²" << std::endl;
    s << "Angular velocity: " << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2] << " deg/s" << std::endl;
    s << "Magnetometer: " << imu.mag[0] << ", " << imu.mag[1] << ", " << imu.mag[2] << " uT" << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Servo& servo) {
    s << "#Servo:" << std::endl;
    s << servo.servo[0] << " " << servo.servo[1] << " " << servo.servo[2] << " " << servo.servo[3] << std::endl;
    s << servo.servo[4] << " " << servo.servo[5] << " " << servo.servo[6] << " " << servo.servo[7] << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Motor& motor) {
    s << "#Motor:" << std::endl;
    s << motor.motor[0] << " " << motor.motor[1] << " " << motor.motor[2] << " " << motor.motor[3] << std::endl;
    s << motor.motor[4] << " " << motor.motor[5] << " " << motor.motor[6] << " " << motor.motor[7] << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Rc& rc) {
    s << "#Rc channels (" << rc.channels.size() << ") :" << std::endl;
    for(const uint16_t c : rc.channels) { s << c << " "; }
    s << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Attitude& attitude) {
    s << "#Attitude:" << std::endl;
    s << "Ang : " << attitude.roll << ", " << attitude.pitch << " deg" << std::endl;
    s << "Heading: " << attitude.yaw << " deg" << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Altitude& altitude) {
    s << "#Altitude:" << std::endl;
    s << "Altitude: " << altitude.altitude << " m, var: " << altitude.vario << " m/s" << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Analog& analog) {
    s << "#Analog:" << std::endl;
    s << "Battery Voltage: " << analog.vbat << " V" << std::endl;
    s << "Current: " << analog.amperage << " A" << std::endl;
    s << "Power consumption: " << analog.powerMeterSum << " Ah" << std::endl;
    s << "RSSI: " << analog.rssi << std::endl;
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::RcTuning& rc_tuning) {
    s << "#Rc Tuning:" << std::endl;
    s << "Rc Rate: " << rc_tuning.rcRates[0] << " " << rc_tuning.rcRates[1] << " " << rc_tuning.rcRates[2] << std::endl;
    s << "Rc Expo: " << rc_tuning.rcExpo[0] << " " << rc_tuning.rcExpo[1] << " " << rc_tuning.rcExpo[2] << std::endl;

    s << "Dynamic Throttle PID: " << rc_tuning.dynamic_throttle_pid << std::endl;
    s << "Throttle MID: " << rc_tuning.throttle_mid << std::endl;
    s << "Throttle Expo: " << rc_tuning.throttle_expo << std::endl;

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Pid& pid) {
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

std::ostream& operator<<(std::ostream& s, const msp::msg::ActiveBoxes& box) {
    s << "#Box:" << std::endl;
    for(size_t ibox(0); ibox<box.box_pattern.size(); ibox++) {
        s << ibox << " ";
        for(size_t iaux(0); iaux<box.box_pattern[ibox].size(); iaux++) {
            s << "aux" << iaux+1 << ": ";
            if(box.box_pattern[ibox][iaux].count(msp::msg::SwitchPosition::LOW))
                s << "L";
            else
                s << "_";
            if(box.box_pattern[ibox][iaux].count(msp::msg::SwitchPosition::MID))
                s << "M";
            else
                s << "_";
            if(box.box_pattern[ibox][iaux].count(msp::msg::SwitchPosition::HIGH))
                s << "H";
            else
                s << "_";
            s << ", ";
        }
        s << std::endl;
    }

    return s;
}
uint16_t mid_rc;
    uint16_t min_throttle;
    uint16_t max_throttle;
    uint16_t min_command;
    uint16_t failsafe_throttle;
    uint8_t gps_provider;
    uint8_t gps_baudrate;
    uint8_t gps_usb_sbas;
    uint8_t multiwii_current_meter_output;
    uint8_t rssi_channel;
    uint8_t reserved;
    float mag_declination; // degree
    float voltage_scale, cell_min, cell_max, cell_warning;

std::ostream& operator<<(std::ostream& s, const msp::msg::Misc& misc) {
    s << "#Miscellaneous:" << std::endl;
    s << "Mid rc: " << misc.mid_rc << std::endl;
    s << "Min Throttle: " << misc.min_throttle << std::endl;
    s << "Max Throttle: " << misc.max_throttle << std::endl;
    s << "Failsafe Throttle: " << misc.failsafe_throttle << std::endl;

    s << "Magnetic Declination: " << misc.mag_declination << " deg" << std::endl;
    s << "Battery Voltage Scale: " << misc.voltage_scale << " V" << std::endl;
    s << "Battery Warning Level 1: " << misc.cell_min << " V" << std::endl;
    s << "Battery Warning Level 2: " << misc.cell_max << " V" << std::endl;
    s << "Battery Critical Level: " << misc.cell_warning << " V" << std::endl;

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::MotorPins& pin) {
    s << "#Motor pins:" << std::endl;
    for(size_t imotor(0); imotor<msp::msg::N_MOTOR; imotor++) {
        s << "Motor " << imotor << ": pin " << size_t(pin.pwm_pin[imotor]()) << std::endl;
    }

    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::BoxNames& box_names) {
    s << "#Box names:" << std::endl;
    for(size_t ibox(0); ibox<box_names.box_names.size(); ibox++) {
        s << ibox << ": " << box_names.box_names[ibox] << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::PidNames& pid_names) {
    s << "#PID names:" << std::endl;
    for(size_t ipid(0); ipid<pid_names.pid_names.size(); ipid++) {
        s << ipid << ": " << pid_names.pid_names[ipid] << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::BoxIds& box_ids) {
    s << "#Box IDs:" << std::endl;
    for(size_t ibox(0); ibox<box_ids.box_ids.size(); ibox++) {
        s << ibox << ": " << size_t(box_ids.box_ids[ibox]) << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::ServoConf& servo_conf) {
    s << "#Servo conf:" << std::endl;
    s << "Nr. | [min | middle | max] (rate)" << std::endl;
    for(size_t iservo(0); iservo<msp::msg::N_SERVO; iservo++) {
        const msp::msg::ServoConfRange servo = servo_conf.servo_conf[iservo];
        s << iservo << ":  | "<< "["<< servo.min() <<" | "<< servo.middle() <<" | "<< servo.max() <<"] ("<< size_t(servo.rate()) <<")"  << std::endl;
    }
    return s;
}

std::ostream& operator<<(std::ostream& s, const msp::msg::Debug& debug) {
    s << "#Debug:" << std::endl;
    s << "debug1: " << debug.debug1 << std::endl;
    s << "debug2: " << debug.debug2 << std::endl;
    s << "debug3: " << debug.debug3 << std::endl;
    s << "debug4: " << debug.debug4 << std::endl;
    return s;
}
