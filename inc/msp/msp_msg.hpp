// MSP message definitions
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

#ifndef MSP_MSG_HPP
#define MSP_MSG_HPP

#include <string>
#include <array>
#include <sstream>
#include <set>
#include <climits>
#include <cassert>

#include "types.hpp"

#include "deserialise.hpp"

namespace msp {

const static uint N_SERVO = 8;
const static uint N_MOTOR = 8;
const static uint RC_CHANS = 8;

const static uint BOARD_IDENTIFIER_LENGTH = 4;

const static uint BUILD_DATE_LENGTH = 11;
const static uint BUILD_TIME_LENGTH = 8;
const static uint GIT_SHORT_REVISION_LENGTH = 7;

enum class MultiType : uint8_t {
    TRI             = 1,
    QUADP,  		// 2
    QUADX,  		// 3
    BI,     		// 4
    GIMBAL, 		// 5
    Y6,     		// 6
    HEX6,   		// 7
    FLYING_WING,	// 8
    Y4,     		// 9
    HEX6X,  		// 10
    OCTOX8, 		// 11
    OCTOFLATP,  	// 12
    OCTOFLATX,  	// 13
    AIRPLANE,   	// 14
    HELI_120_CCPM,  // 15
    HELI_90_DEG,    // 16
    VTAIL4,     	// 17
    HEX6H,      	// 18
    DUALCOPTER      = 20,
    SINGLECOPTER,   // 21
};

enum class Capability {
    BIND,
    DYNBAL,
    FLAP,
    NAVCAP,
    EXTAUX
};

enum class Sensor {
    Accelerometer,
    Barometer,
    Magnetometer,
    GPS,
    Sonar
};

const static uint NAUX = 4;

enum class SwitchPosition : uint {
    LOW  = 0,
    MID  = 1,
    HIGH = 2,
};

static const std::vector<std::string> FEATURES = {
    "RX_PPM", "VBAT", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "FAILSAFE",
    "SONAR", "TELEMETRY", "AMPERAGE_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DISPLAY", "ONESHOT125",
    "BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", "OSD"
};

/////////////////////////////////////////////////////////////////////
/// Cleanflight

// MSP_API_VERSION: 1
struct ApiVersion : public Request {
    ID id() const { return ID::MSP_API_VERSION; }

    uint protocol;
    uint major;
    uint minor;

    void decode(const std::vector<uint8_t> &data) {
        protocol = data[0];
        major = data[1];
        minor = data[2];
    }
};

// MSP_FC_VARIANT: 2
struct FcVariant : public Request {
    ID id() const { return ID::MSP_FC_VARIANT; }

    std::string identifier;

    void decode(const std::vector<uint8_t> &data) {
        identifier = std::string(data.begin(), data.end());
    }
};

// MSP_FC_VERSION: 3
struct FcVersion : public Request {
    ID id() const { return ID::MSP_FC_VERSION; }

    uint major;
    uint minor;
    uint patch_level;

    void decode(const std::vector<uint8_t> &data) {
        major = data[0];
        minor = data[1];
        patch_level = data[2];
    }
};

// MSP_BOARD_INFO: 4
struct BoardInfo : public Request {
    ID id() const { return ID::MSP_BOARD_INFO; }

    std::string identifier;
    uint16_t version;
    uint8_t type;

    void decode(const std::vector<uint8_t> &data) {
        identifier = std::string(data.begin(), data.begin()+BOARD_IDENTIFIER_LENGTH);
        version = deserialise_uint16(data,BOARD_IDENTIFIER_LENGTH);
        type = data[BOARD_IDENTIFIER_LENGTH+2];
    }
};

// MSP_BUILD_INFO: 5
struct BuildInfo : public Request {
    ID id() const { return ID::MSP_BUILD_INFO; }

    std::string buildDate;
    std::string buildTime;
    std::string shortGitRevision;

    void decode(const std::vector<uint8_t> &data) {
        buildDate = std::string((const char*)&data[0], BUILD_DATE_LENGTH);
        buildTime = std::string((const char*)&data[BUILD_DATE_LENGTH], BUILD_TIME_LENGTH);
        shortGitRevision = std::string((const char*)&data[BUILD_DATE_LENGTH+BUILD_TIME_LENGTH], GIT_SHORT_REVISION_LENGTH);
    }
};

// MSP_FEATURE: 36
struct Feature : public Request {
    ID id() const { return ID::MSP_FEATURE; }

    std::set<std::string> features;

    void decode(const std::vector<uint8_t> &data) {
        const uint32_t mask = deserialise_uint32(data,0);
        for(uint ifeat(0); ifeat<FEATURES.size(); ifeat++) {
            if(mask & (1<<ifeat))
                features.insert(FEATURES[ifeat]);
        }
    }
};

// MSP_SET_FEATURE: 37
struct SetFeature : public Response {
    ID id() const { return ID::MSP_SET_FEATURE; }

    std::set<std::string> features;

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> data;
        uint32_t mask = 0;
        for(uint ifeat(0); ifeat<FEATURES.size(); ifeat++) {
            if(features.count(FEATURES[ifeat]))
                mask |= 1<<ifeat;
        }
        serialise_uint32(mask, data);
        return data;
    }
};

// MSP_RX_CONFIG: 44
struct RxConfig : public Request {
    ID id() const { return ID::MSP_RX_CONFIG; }

    uint8_t serialrx_provider;
    uint16_t maxcheck;
    uint16_t midrc;
    uint16_t mincheck;
    uint8_t spektrum_sat_bind;
    uint16_t rx_min_usec;
    uint16_t rx_max_usec;

    void decode(const std::vector<uint8_t> &data) {
        serialrx_provider = data[0];
        maxcheck = deserialise_uint16(data, 1);
        midrc = deserialise_uint16(data, 3);
        mincheck = deserialise_uint16(data, 5);
        spektrum_sat_bind = data[7];
        rx_min_usec = deserialise_uint16(data, 8);
        rx_max_usec = deserialise_uint16(data, 10);
    }
};

// MSP_RX_MAP: 64
struct RxMap : public Request {
    ID id() const { return ID::MSP_RX_MAP; }

    std::vector<uint8_t> map;

    void decode(const std::vector<uint8_t> &data) {
        map = data;
    }
};

// MSP_SET_RX_MAP: 65
struct SetRxMap : public Response {
    ID id() const { return ID::MSP_SET_RX_MAP; }

    std::vector<uint8_t> map;

    std::vector<uint8_t> encode() const {
        return map;
    }
};

// MSP_REBOOT: 68
struct Reboot : public Response {
    ID id() const { return ID::MSP_REBOOT; }
    std::vector<uint8_t> encode() const {
        return std::vector<uint8_t>();
    }
};


/////////////////////////////////////////////////////////////////////
/// Requests (1xx)

// MSP_IDENT: 100
struct Ident : public Request {
    ID id() const { return ID::MSP_IDENT; }

    uint version;
    MultiType type;
    uint msp_version;
    std::set<Capability> capabilities;

    void decode(const std::vector<uint8_t> &data) {
        version = data[0];

        // determine multicopter type
        type = MultiType(data[1]);

        msp_version = data[2];

        const uint32_t capability = deserialise_uint32(data, 3);
        if(capability & (1 << 0))
            capabilities.insert(Capability::BIND);
        if(capability & (1 << 2))
            capabilities.insert(Capability::DYNBAL);
        if(capability & (1 << 3))
            capabilities.insert(Capability::FLAP);
        if(capability & (1 << 4))
            capabilities.insert(Capability::NAVCAP);
        if(capability & (1 << 5))
            capabilities.insert(Capability::EXTAUX);
    }

    bool has(const Capability &cap) const { return capabilities.count(cap); }

    bool hasBind() const { return has(Capability::BIND); }

    bool hasDynBal() const { return has(Capability::DYNBAL); }

    bool hasFlap() const { return has(Capability::FLAP); }
};

// MSP_STATUS: 101
struct Status : public Request {
    ID id() const { return ID::MSP_STATUS; }

    uint16_t    time;   // in us
    uint16_t    errors;
    std::set<Sensor> sensors;
    uint        current_setting;
    std::set<uint> active_box_id;

    void decode(const std::vector<uint8_t> &data) {
        time = deserialise_uint16(data, 0);

        errors = deserialise_uint16(data, 2);

        // get sensors
        sensors.clear();
        const uint16_t sensor = deserialise_uint16(data, 4);
        if(sensor & (1 << 0))
            sensors.insert(Sensor::Accelerometer);
        if(sensor & (1 << 1))
            sensors.insert(Sensor::Barometer);
        if(sensor & (1 << 2))
            sensors.insert(Sensor::Magnetometer);
        if(sensor & (1 << 3))
            sensors.insert(Sensor::GPS);
        if(sensor & (1 << 4))
            sensors.insert(Sensor::Sonar);

        // check active boxes
        active_box_id.clear();
        const uint32_t flag = deserialise_uint32(data, 6);
        for(uint ibox(0); ibox<sizeof(flag)*CHAR_BIT; ibox++) {
            if(flag & (1 << ibox))
                active_box_id.insert(ibox);
        }

        current_setting = data[10];
    }

    bool hasAccelerometer() const { return sensors.count(Sensor::Accelerometer); }

    bool hasBarometer() const { return sensors.count(Sensor::Barometer); }

    bool hasMagnetometer() const { return sensors.count(Sensor::Magnetometer); }

    bool hasGPS() const { return sensors.count(Sensor::GPS); }

    bool hasSonar() const { return sensors.count(Sensor::Sonar); }
};

// MSP_RAW_IMU: 102
struct Imu : public Request {
    ID id() const { return ID::MSP_RAW_IMU; }

    std::array<float, 3> acc;   // m/s^2
    std::array<float, 3> gyro;  // deg/s
    std::array<float, 3> magn;  // uT

    // conversion units
    float acc_1g;       // sensor value at 1g
    float gyro_unit;    // resolution in 1/(deg/s)
    float magn_gain;    // scale magnetic value to uT (micro Tesla)
    float si_unit_1g;   // acceleration at 1g (in m/s^2)

    Imu(float acc_1g = 1.0, float gyro_unit = 1.0, float magn_gain = 1.0, float si_unit_1g = 1.0)
        : acc_1g(acc_1g), gyro_unit(gyro_unit), magn_gain(magn_gain), si_unit_1g(si_unit_1g)
    { }

    void decode(const std::vector<uint8_t> &data) {
        acc = {{deserialise_int16(data, 0)/acc_1g*si_unit_1g,
                deserialise_int16(data, 2)/acc_1g*si_unit_1g,
                deserialise_int16(data, 4)/acc_1g*si_unit_1g}};

        gyro = {{deserialise_int16(data, 6)*gyro_unit,
                 deserialise_int16(data, 8)*gyro_unit,
                 deserialise_int16(data, 10)*gyro_unit}};

        magn = {{deserialise_int16(data, 12)*magn_gain,
                 deserialise_int16(data, 14)*magn_gain,
                 deserialise_int16(data, 16)*magn_gain}};
    }
};

// MSP_SERVO: 103
struct Servo : public Request {
    ID id() const { return ID::MSP_SERVO; }

    uint16_t servo[N_SERVO];

    void decode(const std::vector<uint8_t> &data) {
        for(unsigned int i=0; i<N_SERVO; i++)
            servo[i] = deserialise_uint16(data, 2*i);
    }
};

// MSP_MOTOR: 104
struct Motor : public Request {
    ID id() const { return ID::MSP_MOTOR; }

    uint16_t motor[N_MOTOR];

    void decode(const std::vector<uint8_t> &data) {
        for(unsigned int i=0; i<N_MOTOR; i++)
            motor[i] = deserialise_uint16(data, 2*i);
    }
};

// MSP_RC: 105
struct Rc : public Request {
    ID id() const { return ID::MSP_RC; }

    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t throttle;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;

    void decode(const std::vector<uint8_t> &data) {
        // If feature 'RX_MSP' is active but "USE_RX_MSP" is undefined for the target,
        // no RC data is provided as feedback. See also description at 'MSP_SET_RAW_RC'.
        // In this case, return 0 for all RC channels.
        if(data.size()==0) {
            return;
        }

        roll        = deserialise_uint16(data, 0);
        pitch       = deserialise_uint16(data, 2);
        yaw         = deserialise_uint16(data, 4);
        throttle    = deserialise_uint16(data, 6);

        aux1        = deserialise_uint16(data, 8);
        aux2        = deserialise_uint16(data, 10);
        aux3        = deserialise_uint16(data, 12);
        aux4        = deserialise_uint16(data, 14);
    }
};

// MSP_RAW_GPS: 106
struct RawGPS : public Request {
    ID id() const { return ID::MSP_RAW_GPS; }

    uint8_t fix;
    uint8_t numSat;
    uint32_t lat;
    uint32_t lon;
    uint16_t altitude;
    uint16_t speed;
    uint16_t ground_course;

    void decode(const std::vector<uint8_t> &data) {
        fix             = data[0];
        numSat          = data[1];
        lat             = deserialise_uint32(data, 2);
        lon             = deserialise_uint32(data, 6);
        altitude        = deserialise_uint16(data, 10);
        speed           = deserialise_uint16(data, 12);
        ground_course   = deserialise_uint16(data, 14);
    }
};

// MSP_COMP_GPS: 107
struct CompGPS : public Request {
    ID id() const { return ID::MSP_COMP_GPS; }

    uint16_t distanceToHome;    // meter
    uint16_t directionToHome;   // degree
    uint8_t update;

    void decode(const std::vector<uint8_t> &data) {
        distanceToHome  = deserialise_uint16(data, 0);
        directionToHome = deserialise_uint16(data, 2);
        update          = data[4];
    }
};

// MSP_ATTITUDE: 108
struct Attitude : public Request {
    ID id() const { return ID::MSP_ATTITUDE; }

    float ang_x;        // degree
    float ang_y;        // degree
    int16_t heading;    // degree

    void decode(const std::vector<uint8_t> &data) {
        ang_x   = deserialise_int16(data, 0)/10.0f;
        ang_y   = deserialise_int16(data, 2)/10.0f;
        heading = deserialise_int16(data, 4);
    }
};

// MSP_ALTITUDE: 109
struct Altitude : public Request {
    ID id() const { return ID::MSP_ALTITUDE; }

    float altitude; // m
    float vario;    // m/s

    void decode(const std::vector<uint8_t> &data) {
        altitude = deserialise_int32(data, 0)/100.0f;
        vario    = deserialise_int16(data, 4)/100.0f;
    }
};

// MSP_ANALOG: 110
struct Analog : public Request {
    ID id() const { return ID::MSP_ANALOG; }

    float vbat;           // Volt
    float powerMeterSum;  // Ah
    uint rssi;  // Received Signal Strength Indication [0; 1023]
    float amperage;       // Ampere

    void decode(const std::vector<uint8_t> &data) {
        vbat          = data[0]/10.0f;
        powerMeterSum = deserialise_uint16(data, 1)/1000.0f;
        rssi          = deserialise_uint16(data, 3);
        amperage      = deserialise_uint16(data, 5)/10.0f;
    }
};

// MSP_RC_TUNING: 111
struct RcTuning : Request {
    ID id() const { return ID::MSP_RC_TUNING; }

    double RC_RATE;
    double RC_EXPO;
    double RollPitchRate;
    double YawRate;
    double DynThrPID;
    double Throttle_MID;
    double Throttle_EXPO;

    void decode(const std::vector<uint8_t> &data) {
        RC_RATE         = data[0] / 100.0;
        RC_EXPO         = data[1] / 100.0;
        RollPitchRate   = data[2] / 100.0;
        YawRate         = data[3] / 100.0;
        DynThrPID       = data[4] / 100.0;
        Throttle_MID    = data[5] / 100.0;
        Throttle_EXPO   = data[6] / 100.0;
    }
};

// PID struct for messages 112 and 204
struct PidTerms {
    float P;
    float I;
    float D;

    PidTerms() { }

    PidTerms(const uint8_t P, const uint8_t I, const uint8_t D)
        : P(P/10.0f), I(I/10.0f), D(D/10.0f)
    { }
};

// MSP_PID: 112
struct Pid : public Request {
    ID id() const { return ID::MSP_PID; }

    PidTerms roll, pitch, yaw, alt;
    PidTerms pos, posr, navr, level, mag, vel;

    void decode(const std::vector<uint8_t> &data) {
        roll  = PidTerms(data[0], data[1], data[2]);
        pitch = PidTerms(data[3], data[4], data[5]);
        yaw   = PidTerms(data[6], data[7], data[8]);
        alt   = PidTerms(data[9], data[10], data[11]);
        pos   = PidTerms(data[12], data[13], data[14]);
        posr  = PidTerms(data[15], data[16], data[17]);
        navr  = PidTerms(data[18], data[19], data[20]);
        level = PidTerms(data[21], data[22], data[23]);
        mag   = PidTerms(data[24], data[25], data[26]);
        vel   = PidTerms(data[27], data[28], data[29]);
    }
};

// MSP_BOX: 113
struct Box : public Request {
    ID id() const { return ID::MSP_BOX; }

    // box activation pattern
    std::vector<std::array<std::set<SwitchPosition>,NAUX>> box_pattern;

    void decode(const std::vector<uint8_t> &data) {
        box_pattern.clear();
        for(uint i(0); i<data.size(); i+=2) {
            const uint16_t box_conf = deserialise_uint16(data, i);
            //box_conf.push_back(deser16(data, i));

            std::array<std::set<SwitchPosition>,NAUX> aux_sp;
            for(uint iaux(0); iaux<NAUX; iaux++) {
                for(uint ip(0); ip<3; ip++) {
                    if(box_conf & (1<<(iaux*3+ip)))
                        aux_sp[iaux].insert(SwitchPosition(ip));
                } // each position (L,M,H)
            } // each aux switch
            box_pattern.push_back(aux_sp);
        } // each box
    }
};

// MSP_MISC: 114
struct Misc : public Request {
    ID id() const { return ID::MSP_MISC; }

    uint powerTrigger;
    uint minThrottle, maxThrottle, failsafeThrottle;
    uint minCommand;
    uint arm, lifetime;
    float mag_declination; // degree
    float vbatScale, vbatLevelWarn1, vbatLevelWarn2, vbatLevelCrit;

    void decode(const std::vector<uint8_t> &data) {
        powerTrigger     = deserialise_uint16(data, 0);
        minThrottle         = deserialise_uint16(data, 2);
        maxThrottle         = deserialise_uint16(data, 4);
        minCommand          = deserialise_uint16(data, 6);

        failsafeThrottle    = deserialise_uint16(data, 8);
        arm                 = deserialise_uint16(data, 10);
        lifetime            = deserialise_uint32(data, 12);
        mag_declination     = deserialise_uint16(data, 16) / 10.0f;

        vbatScale           = data[18] / 10.0f;
        vbatLevelWarn1      = data[19] / 10.0f;
        vbatLevelWarn2      = data[20] / 10.0f;
        vbatLevelCrit       = data[21] / 10.0f;
    }
};

// MSP_MOTOR_PINS: 115
struct MotorPins : public Request {
    ID id() const { return ID::MSP_MOTOR_PINS; }

    uint8_t pwm_pin[N_MOTOR];

    void decode(const std::vector<uint8_t> &data) {
        for(uint i(0); i<N_MOTOR; i++)
            pwm_pin[i] = data[i];
    }
};

// MSP_BOXNAMES: 116
struct BoxNames : public Request {
    ID id() const { return ID::MSP_BOXNAMES; }

    std::vector<std::string> box_names;

    void decode(const std::vector<uint8_t> &data) {
        box_names.clear();

        std::stringstream ss(std::string(data.begin(), data.end()));
        std::string bname;
        while(getline(ss, bname, ';')) {
            box_names.push_back(bname);
        }
    }
};

// MSP_PIDNAMES: 117
struct PidNames : public Request {
    ID id() const { return ID::MSP_PIDNAMES; }

    std::vector<std::string> pid_names;

    void decode(const std::vector<uint8_t> &data) {
        pid_names.clear();

        std::stringstream ss(std::string(data.begin(), data.end()));
        std::string pname;
        while(getline(ss, pname, ';')) {
            pid_names.push_back(pname);
        }
    }
};

// MSP_WP: 118
struct WayPoint : public Request {
    ID id() const { return ID::MSP_WP; }

    uint8_t wp_no;
    uint32_t lat;
    uint32_t lon;
    uint32_t altHold;
    uint16_t heading;
    uint16_t staytime;
    uint8_t navflag;

    void decode(const std::vector<uint8_t> &data) {
        wp_no = data[0];
        lat = deserialise_uint32(data, 1);
        lon = deserialise_uint32(data, 5);
        altHold = deserialise_uint32(data, 9);
        heading = deserialise_uint16(data, 13);
        staytime = deserialise_uint16(data, 15);
        navflag = data[18];
    }
};

// MSP_BOXIDS: 119
struct BoxIds : public Request {
    ID id() const { return ID::MSP_BOXIDS; }

    std::vector<uint8_t> box_ids;

    void decode(const std::vector<uint8_t> &data) {
        box_ids.clear();

        for(uint8_t bi : data)
            box_ids.push_back(bi);;
    }
};

struct ServoConfRange {
    uint16_t min;
    uint16_t max;
    uint16_t middle;
    uint8_t rate;
};

// MSP_SERVO_CONF: 120
struct ServoConf : public Request {
    ID id() const { return ID::MSP_SERVO_CONF; }

    ServoConfRange servo_conf[N_SERVO];

    void decode(const std::vector<uint8_t> &data) {
        for(uint i(0); i<N_SERVO; i++) {
            servo_conf[i].min = deserialise_uint16(data, 7*i);
            servo_conf[i].max = deserialise_uint16(data, 7*i+2);
            servo_conf[i].middle = deserialise_uint16(data, 7*i+4);
            servo_conf[i].rate = data[7*i+6];
        }
    }
};

// MSP_NAV_STATUS: 121
struct NavStatus: public Request {
    ID id() const { return ID::MSP_NAV_STATUS; }

    uint8_t GPS_mode;
    uint8_t NAV_state;
    uint8_t mission_action;
    uint8_t mission_number;
    uint8_t NAV_error;
    int16_t target_bearing; // degrees

    void decode(const std::vector<uint8_t> &data) {
        GPS_mode = data[0];
        NAV_state = data[1];
        mission_action = data[2];
        mission_number = data[3];
        NAV_error = data[4];
        target_bearing = deserialise_int16(data, 5);
    }
};


struct GpsConf {
  uint8_t filtering;
  uint8_t lead_filter;
  uint8_t dont_reset_home_at_arm;
  uint8_t nav_controls_heading;

  uint8_t nav_tail_first;
  uint8_t nav_rth_takeoff_heading;
  uint8_t slow_nav;
  uint8_t wait_for_rth_alt;

  uint8_t ignore_throttle;
  uint8_t takeover_baro;

  uint16_t wp_radius;           // in cm
  uint16_t safe_wp_distance;    // in meter
  uint16_t nav_max_altitude;    // in meter
  uint16_t nav_speed_max;       // in cm/s
  uint16_t nav_speed_min;       // in cm/s

  uint8_t  crosstrack_gain;     // * 100 (0-2.56)
  uint16_t nav_bank_max;        // degree * 100; (3000 default)
  uint16_t rth_altitude;        // in meter
  uint8_t  land_speed;          // between 50 and 255 (100 approx = 50cm/sec)
  uint16_t fence;               // fence control in meters

  uint8_t  max_wp_number;

  uint8_t  checksum;
};

// MSP_NAV_CONFIG: 122
struct NavConfig: public Request {
    ID id() const { return ID::MSP_NAV_CONFIG; }

    GpsConf gps_conf;

    void decode(const std::vector<uint8_t> &data) {
        gps_conf.filtering = data[0];
        gps_conf.lead_filter = data[1];
        gps_conf.dont_reset_home_at_arm = data[2];
        gps_conf.nav_controls_heading = data[3];

        gps_conf.nav_tail_first = data[4];
        gps_conf.nav_rth_takeoff_heading = data[5];
        gps_conf.slow_nav = data[6];
        gps_conf.wait_for_rth_alt = data[7];

        gps_conf.ignore_throttle = data[8];
        gps_conf.takeover_baro = data[9];

        gps_conf.wp_radius = deserialise_uint16(data, 10);
        gps_conf.safe_wp_distance = deserialise_uint16(data, 12);
        gps_conf.nav_max_altitude = deserialise_uint16(data, 14);
        gps_conf.nav_speed_max = deserialise_uint16(data, 16);
        gps_conf.nav_speed_min = deserialise_uint16(data, 18);

        gps_conf.crosstrack_gain = data[20];
        gps_conf.nav_bank_max = deserialise_uint16(data, 21);
        gps_conf.rth_altitude = deserialise_uint16(data, 23);
        gps_conf.land_speed = data[25];
        gps_conf.fence = deserialise_uint16(data, 26);

        gps_conf.max_wp_number = data[28];

        gps_conf.checksum = data[29];
    }
};

// MSP_DEBUGMSG: 253
struct DebugMessage : public Request {
    ID id() const { return ID::MSP_DEBUGMSG; }

    std::string msg;

    void decode(const std::vector<uint8_t> &data) {
        msg = std::string(data.begin(), data.end());
    }
};

// MSP_DEBUG: 254
struct Debug : public Request {
    ID id() const { return ID::MSP_DEBUG; }

    uint16_t debug1;
    uint16_t debug2;
    uint16_t debug3;
    uint16_t debug4;

    void decode(const std::vector<uint8_t> &data) {
        debug1 = deserialise_uint16(data, 0);
        debug2 = deserialise_uint16(data, 2);
        debug3 = deserialise_uint16(data, 4);
        debug4 = deserialise_uint16(data, 6);
    }
};


/////////////////////////////////////////////////////////////////////
/// Response (2xx)

// MSP_SET_RAW_RC: 200
// This message is accepted but ignored on betaflight 3.0.1 onwards
// if "USE_RX_MSP" is not defined for the target. In this case, you can manually
// add "#define USE_RX_MSP" to your 'target.h'.
struct SetRc : public Response {
    ID id() const { return ID::MSP_SET_RAW_RC; }

    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t throttle;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> data;
        for(auto channel : {roll, pitch, yaw, throttle,
                            aux1, aux2, aux3, aux4})
        {
            serialise_uint16(channel, data);
        }
        assert(data.size()==RC_CHANS*2);
        return data;
    }
};

// MSP_SET_RAW_GPS: 201
struct SetRawGPS : public Request {
    ID id() const { return ID::MSP_SET_RAW_GPS; }

    uint8_t fix;
    uint8_t numSat;
    uint32_t lat;
    uint32_t lon;
    uint16_t altitude;
    uint16_t speed;

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> data;
        data.push_back(fix);
        data.push_back(numSat);
        serialise_uint32(lat, data);
        serialise_uint32(lon, data);
        serialise_uint16(altitude, data);
        serialise_uint16(speed, data);
        assert(data.size()==14);
        return data;
    }
};

// MSP_SET_RC_TUNING: 204
struct SetRcTuning : public Response {
    ID id() const { return ID::MSP_SET_RC_TUNING; }

    double RC_RATE;
    double RC_EXPO;
    double RollPitchRate;
    double YawRate;
    double DynThrPID;
    double Throttle_MID;
    double Throttle_EXPO;

    /**
     * @brief SetRcTuning construct a RC tuning response from a request
     * @param rc_tuning RcTuning response
     */
    SetRcTuning(const RcTuning &rc_tuning) {
        RC_RATE         = rc_tuning.RC_RATE;
        RC_EXPO         = rc_tuning.RC_EXPO;
        RollPitchRate   = rc_tuning.RollPitchRate;
        YawRate         = rc_tuning.YawRate;
        DynThrPID       = rc_tuning.DynThrPID;
        Throttle_MID    = rc_tuning.Throttle_MID;
        Throttle_EXPO   = rc_tuning.Throttle_EXPO;
    }

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> data(7);
        data[0] = uint8_t(RC_RATE * 100);
        data[1] = uint8_t(RC_EXPO * 100);
        data[2] = uint8_t(RollPitchRate * 100);
        data[3] = uint8_t(YawRate * 100);
        data[4] = uint8_t(DynThrPID * 100);
        data[5] = uint8_t(Throttle_MID * 100);
        data[6] = uint8_t(Throttle_EXPO * 100);
        return data;
    }
};

// MSP_ACC_CALIBRATION: 205
struct AccCalibration : public Response {
    ID id() const { return ID::MSP_ACC_CALIBRATION; }
    std::vector<uint8_t> encode() const {
        return std::vector<uint8_t>();
    }
};

// MSP_MAG_CALIBRATION: 206
struct MagCalibration : public Response {
    ID id() const { return ID::MSP_MAG_CALIBRATION; }
    std::vector<uint8_t> encode() const {
        return std::vector<uint8_t>();
    }
};

// MSP_RESET_CONF: 208
struct ResetConfig : public Response {
    ID id() const { return ID::MSP_RESET_CONF; }
    std::vector<uint8_t> encode() const {
        return std::vector<uint8_t>();
    }
};

// MSP_SELECT_SETTING: 210
struct SelectSetting : public Response {
    ID id() const { return ID::MSP_SELECT_SETTING; }

    uint current_setting;

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> data(1);
        data[0] = uint8_t(current_setting);
        return data;
    }
};

// MSP_SET_HEAD: 211
struct SetHeading : public Response {
    ID id() const { return ID::MSP_SET_HEAD; }

    int heading;

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> data;
        serialise_int16(int16_t(heading), data);
        assert(data.size()==2);
        return data;
    }
};

// MSP_SET_MOTOR: 214
struct SetMotor : public Response {
    ID id() const { return ID::MSP_SET_MOTOR; }

    std::array<uint16_t,N_MOTOR> motor;

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> data;
        for(uint i(0); i<N_MOTOR; i++)
            serialise_uint16(motor[i], data);
        assert(data.size()==N_MOTOR*2);
        return data;
    }
};

// MSP_EEPROM_WRITE: 250
struct WriteEEPROM : public Response {
    ID id() const { return ID::MSP_EEPROM_WRITE; }
    std::vector<uint8_t> encode() const {
        return std::vector<uint8_t>();
    }
};

} // namespace msp

#endif // MSP_MSG_HPP
