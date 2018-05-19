#include "msg_print.hpp"
#include <iomanip>




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
