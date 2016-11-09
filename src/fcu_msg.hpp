#ifndef FCU_MSG_HPP
#define FCU_MSG_HPP

#include "msp_msg.hpp"

#include <set>
#include <vector>
#include <array>

namespace fcu {

const static uint NAUX = 4;

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

enum class SwitchPosition : uint {
    LOW  = 0,
    MID  = 1,
    HIGH = 2,
};

// 100
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

// 101
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

// 102
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

// 108
struct Attitude {
    float ang_x;    // degree
    float ang_y;    // degree
    int heading;    // degree

    Attitude(const msp::Attitude &attitude) {
        ang_x = attitude.angx/10.0;
        ang_y = attitude.angy/10.0;
        heading = attitude.heading;
    }
};

// 109
struct Altitude {
    float altitude; // m
    float vario;    // m/s

    Altitude(const msp::Altitude &alt) {
        altitude = alt.EstAlt/100.0;
        vario = alt.vario/100.0;
    }
};

// 110
struct Analog {
    float vbat;             // Volt
    float powerMeterSum;  // Ah
    uint rssi;  // Received Signal Strength Indication [0; 1023]
    uint amperage;          // Ampere

    Analog(const msp::Analog &analog) {
        vbat = analog.vbat/10.0;
        powerMeterSum = analog.intPowerMeterSum/1000.0;
        rssi = analog.rssi;
        amperage = analog.amperage/10.0;
    }
};

struct PidTerms {
    float P;
    float I;
    float D;

    PidTerms() { }

    PidTerms(const msp::PidTerms &pid) {
        P = pid.P / 10.0;
        I = pid.I / 10.0;
        D = pid.D / 10.0;
    }
};

// 112
struct PID {
    PidTerms roll, pitch, yaw, alt;
    PidTerms pos, posr, navr, level, mag, vel;

    PID(const msp::Pid &pid) :
        roll(pid.roll), pitch(pid.pitch), yaw(pid.yaw), alt(pid.alt),
        pos(pid.pos), posr(pid.posr), navr(pid.navr), level(pid.level),
        mag(pid.mag), vel(pid.vel)
    {
        //
    }

};

// 113
struct Box {
    // box activation pattern
    std::vector<std::array<std::set<SwitchPosition>,NAUX>> boxs;

    Box(const msp::Box &box) {
        for(uint16_t b : box.box_conf) {
            std::array<std::set<SwitchPosition>,NAUX> aux_sp;
            for(uint iaux(0); iaux<NAUX; iaux++) {
                for(uint ip(0); ip<3; ip++) {
                    if(b & (1<<(iaux*3+ip)))
                        aux_sp[iaux].insert((SwitchPosition)ip);
                } // each position (L,M,H)
            } // each aux switch
            boxs.push_back(aux_sp);
        } // each box
    }
};

struct Misc {
    uint powerTrigger;
    uint minThrottle, maxThrottle, failsafeThrottle;
    uint arm, lifetime;
    float mag_declination; // degree

    float vbatScale, vbatLevelWarn1, vbatLevelWarn2, vbatLevelCrit;

    Misc(const msp::Misc &misc) {
        powerTrigger = misc.intPowerTrigger;
        minThrottle = misc.minThrottle;
        maxThrottle = misc.minThrottle;
        failsafeThrottle = misc.failsafeThrottle;

        arm = misc.arm;
        lifetime = misc.lifetime;
        mag_declination = misc.mag_declination / 10.0;

        vbatScale = misc.vbatScale / 10.0;
        vbatLevelWarn1 = misc.vbatLevelWarn1 / 10.0;
        vbatLevelWarn2 = misc.vbatLevelWarn2 / 10.0;
        vbatLevelCrit = misc.vbatLevelCrit / 10.0;
    }
};

}

#endif // FCU_MSG_HPP
