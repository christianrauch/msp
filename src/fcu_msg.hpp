#ifndef FCU_MSG_HPP
#define FCU_MSG_HPP

#include "msp_msg.hpp"

#include <set>

namespace fcu {

enum class Sensor {
    Accelerometer,
    Barometer,
    Magnetometer,
    GPS,
    Sonar
};

enum class MultiType {
    TRI,    // 1
    QUADP,  // 2
    QUADX,  // 3
    BI,     // 4
    GIMBAL, // 5
    Y6,     // 6
    HEX6,   // 7
    FLYING_WING, // 8
    Y4,     // 9
    HEX6X,  // 10
    OCTOX8, // 11
    OCTOFLATP,  // 12
    OCTOFLATX,  // 13
    AIRPLANE,   // 14
    HELI_120_CCPM,  // 15
    HELI_90_DEG,    //16
    VTAIL4,     // 17
    HEX6H,      // 18
    SINGLECOPTER,   // 21
    DUALCOPTER, // 20
};

enum class Capability {
    BIND,
    DYNBAL,
    FLAP
};

struct Ident {
    uint version;
    MultiType type;
    std::set<Capability> capabilities;

    Ident(const msp::Ident &ident) {
        version = ident.version;
        switch(ident.type) {
        case 1: type = MultiType::TRI; break;
        case 2: type = MultiType::QUADP; break;
        case 3: type = MultiType::QUADX; break;
        case 4: type = MultiType::BI; break;
        default:
            break;
        }

        if(ident.capability & (1 << 0))
            capabilities.insert(Capability::BIND);
        if(ident.capability & (1 << 1))
            capabilities.insert(Capability::DYNBAL);
        if(ident.capability & (1 << 2))
            capabilities.insert(Capability::FLAP);
    }

    bool has(const Capability &cap) const {
        return capabilities.count(cap);
    }

    bool hasBind() const {
        return has(Capability::BIND);
    }

    bool hasDynBal() const {
        return has(Capability::DYNBAL);
    }

    bool hasFlap() const {
        return has(Capability::FLAP);
    }
};

struct Status {
    std::set<Sensor> sensors;
    uint time;
    uint errors;

    Status(const msp::Status &status) {
        if(status.sensor & (1 << 0))
            sensors.insert(Sensor::Accelerometer);
        if(status.sensor & (1 << 1))
            sensors.insert(Sensor::Barometer);
        if(status.sensor & (1 << 2))
            sensors.insert(Sensor::Magnetometer);
        if(status.sensor & (1 << 3))
            sensors.insert(Sensor::GPS);
        if(status.sensor & (1 << 4))
            sensors.insert(Sensor::Sonar);

        time = status.time;
        errors = status.i2c_errors_count;
    }

    bool hasAccelerometer() const { return sensors.count(Sensor::Accelerometer); }

    bool hasBarometer() const { return sensors.count(Sensor::Barometer); }

    bool hasMagnetometer() const { return sensors.count(Sensor::Magnetometer); }

    bool hasGPS() const { return sensors.count(Sensor::GPS); }

    bool hasSonar() const { return sensors.count(Sensor::Sonar); }

};

struct Imu {
    std::array<float, 3> acc;
    std::array<float, 3> gyro;
    std::array<float, 3> magn;

    Imu(const msp::RawImu &imu, const float acc_1g, const float gyro_unit) {
        acc = {float(imu.accx/acc_1g), float(imu.accy/acc_1g), float(imu.accz/acc_1g)};
        gyro = {imu.gyrx*gyro_unit, imu.gyry*gyro_unit, imu.gyrz*gyro_unit};
        // TODO: transform to real values
        magn = {float(imu.magx), float(imu.magy), float(imu.magz)};
    }
};

}

#endif // FCU_MSG_HPP
