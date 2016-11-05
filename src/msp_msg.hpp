// MSP message definitions
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

#ifndef MSP_MSG_HPP
#define MSP_MSG_HPP

#include "types.hpp"

#include "deserialise.hpp"


#define N_SERVO     8
#define N_MOTOR     8
#define RC_CHANS    8
#define PIDITEMS    10

namespace msp {

/////////////////////////////////////////////////////////////////////
/// Requests (1xx)

// MSP_IDENT: 100
struct Ident : public Request {
    ID id() const { return ID::MSP_IDENT; }

    uint8_t     version;
    uint8_t     type;
    uint8_t     msp_version;
    uint32_t    capability;

    void decode(const std::vector<uint8_t> &data) {
        version     = data[0];
        type        = data[1];
        msp_version = data[2];
        capability  = deser32(data, 3);
    }
};

// MSP_STATUS: 101
struct Status : public Request {
    ID id() const { return ID::MSP_STATUS; }

    uint16_t    time;   // in us
    uint16_t    i2c_errors_count;
    uint16_t    sensor;
    uint32_t    flag;
    uint8_t     current_setting;

    void decode(const std::vector<uint8_t> &data) {
        time                = deser16(data, 0);
        i2c_errors_count    = deser16(data, 2);
        sensor              = deser16(data, 4);
        flag                = deser32(data, 6);
        current_setting     = data[10];
    }
};

// MSP_RAW_IMU: 102
struct RawImu : public Request {
    ID id() const { return ID::MSP_RAW_IMU; }

    int16_t accx;
    int16_t accy;
    int16_t accz;

    int16_t gyrx;
    int16_t gyry;
    int16_t gyrz;

    int16_t magx;
    int16_t magy;
    int16_t magz;

    void decode(const std::vector<uint8_t> &data) {
        accx = deser_int16(data, 0);
        accy = deser_int16(data, 2);
        accz = deser_int16(data, 4);

        gyrx = deser_int16(data, 6);
        gyry = deser_int16(data, 8);
        gyrz = deser_int16(data, 10);

        magx = deser_int16(data, 12);
        magy = deser_int16(data, 14);
        magz = deser_int16(data, 16);
    }
};

// MSP_SERVO: 103
struct Servo : public Request {
    ID id() const { return ID::MSP_SERVO; }

    uint16_t servo[N_SERVO];

    void decode(const std::vector<uint8_t> &data) {
        for(unsigned int i=0; i<N_SERVO; i++)
            servo[i] = deser16(data, 2*i);
    }
};

// MSP_MOTOR: 104
struct Motor : public Request {
    ID id() const { return ID::MSP_MOTOR; }

    uint16_t motor[N_MOTOR];

    void decode(const std::vector<uint8_t> &data) {
        for(unsigned int i=0; i<N_MOTOR; i++)
            motor[i] = deser16(data, 2*i);
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
        size_t i = 0;
        for(auto channel : {roll, pitch, yaw, throttle,
                            aux1, aux2, aux3, aux4})
        {
            channel = deser16(data, i);
            i +=2;
        }
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
        lat             = deser32(data, 2);
        lon             = deser32(data, 6);
        altitude        = deser16(data, 10);
        speed           = deser16(data, 12);
        ground_course   = deser16(data, 14);
    }
};

// MSP_COMP_GPS: 107
struct CompGPS : public Request {
    ID id() const { return ID::MSP_COMP_GPS; }

    uint16_t distanceToHome;    // meter
    uint16_t directionToHome;   // degree
    uint8_t update;

    void decode(const std::vector<uint8_t> &data) {
        distanceToHome  = deser16(data, 0);
        directionToHome = deser16(data, 2);
        update          = data[4];
    }
};

// MSP_ATTITUDE: 108
struct Attitude : public Request {
    ID id() const { return ID::MSP_ATTITUDE; }

    uint16_t angx;
    uint16_t angy;
    uint16_t heading;

    void decode(const std::vector<uint8_t> &data) {
        angx    = deser16(data, 0);
        angy    = deser16(data, 2);
        heading = deser16(data, 4);
    }
};

// MSP_ALTITUDE: 109
struct Altitude : public Request {
    ID id() const { return ID::MSP_ALTITUDE; }

    uint32_t EstAlt;
    uint16_t vario;

    void decode(const std::vector<uint8_t> &data) {
        EstAlt  = deser32(data, 0);
        vario   = deser16(data, 4);
    }
};

// MSP_ANALOG: 110
struct Analog : public Request {
    ID id() const { return ID::MSP_ANALOG; }

    uint8_t vbat;
    uint16_t intPowerMeterSum;
    uint16_t rssi;
    uint16_t amperage;

    void decode(const std::vector<uint8_t> &data) {
        vbat                = data[0];
        intPowerMeterSum    = deser16(data, 1);
        rssi                = deser16(data, 3);
        amperage            = deser16(data, 5);
    }
};

// MSP_RC_TUNING: 111
struct RcTuning : Request {
    ID id() const { return ID::MSP_RC_TUNING; }

    uint8_t RC_RATE;
    uint8_t RC_EXPO;
    uint8_t RollPitchRate;
    uint8_t YawRate;
    uint8_t DynThrPID;
    uint8_t Throttle_MID;
    uint8_t Throttle_EXPO;

    void decode(const std::vector<uint8_t> &data) {
        RC_RATE         = data[0];
        RC_EXPO         = data[1];
        RollPitchRate   = data[2];
        YawRate         = data[3];
        DynThrPID       = data[4];
        Throttle_MID    = data[5];
        Throttle_EXPO   = data[6];
    }
};

// PID struct for messages 112 and 204
struct PidTerms {
    uint8_t P;
    uint8_t I;
    uint8_t D;

    PidTerms(uint8_t p=0, uint8_t i=0, uint8_t d=0) {
        P = p;
        I = i;
        D = d;
    }
};

// MSP_PID: 112
struct Pid : public Request {
    ID id() const { return ID::MSP_PID; }

    PidTerms roll;
    PidTerms pitch;
    PidTerms yaw;
    PidTerms alt;
    PidTerms pos;
    PidTerms posr;
    PidTerms navr;
    PidTerms level;
    PidTerms mag;
    PidTerms vel;

    void decode(const std::vector<uint8_t> &data) {
        roll = PidTerms(data[0], data[1], data[2]);
        pitch = PidTerms(data[3], data[4], data[5]);
        yaw = PidTerms(data[6], data[7], data[8]);
        alt = PidTerms(data[9], data[10], data[11]);
        pos = PidTerms(data[12], data[13], data[14]);
        posr = PidTerms(data[15], data[16], data[17]);
        navr = PidTerms(data[18], data[19], data[20]);
        level = PidTerms(data[21], data[22], data[23]);
        mag = PidTerms(data[24], data[25], data[26]);
        vel = PidTerms(data[27], data[28], data[29]);
    }
};

// MSP_BOX: 113
struct Box : public Request {
    ID id() const { return ID::MSP_BOX; }

    // after decode, box_conf should have length of BOXITEMS
    std::vector<uint16_t> box_conf;

    void decode(const std::vector<uint8_t> &data) {
        box_conf.clear();
        for(uint i(0); i<data.size(); i+=4)
            box_conf.push_back(deser16(data, i));
    }
};

// MSP_MISC: 114
struct Misc : public Request {
    ID id() const { return ID::MSP_MISC; }

    uint16_t intPowerTrigger;
    uint16_t minThrottle;
    uint16_t maxThrottle;
    uint16_t minCommand;

    uint16_t failsafeThrottle;
    uint16_t arm;
    uint32_t lifetime;
    uint16_t mag_declination;

    uint8_t vbatScale;
    uint8_t vbatLevelWarn1;
    uint8_t vbatLevelWarn2;
    uint8_t vbatLevelCrit;

    void decode(const std::vector<uint8_t> &data) {
        intPowerTrigger     = deser16(data, 0);
        minThrottle         = deser16(data, 2);
        maxThrottle         = deser16(data, 4);
        minCommand          = deser16(data, 6);

        failsafeThrottle    = deser16(data, 8);
        arm                 = deser16(data, 10);
        lifetime            = deser32(data, 12);
        mag_declination     = deser16(data, 16);

        vbatScale           = data[18];
        vbatLevelWarn1      = data[19];
        vbatLevelWarn2      = data[20];
        vbatLevelCrit       = data[21];
    }
};

// MSP_MOTOR_PINS: 115
struct MotorPin : public Request {
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
        lat = deser32(data, 1);
        lon = deser32(data, 5);
        altHold = deser32(data, 9);
        heading = deser16(data, 13);
        staytime = deser16(data, 15);
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
            servo_conf[i].min = deser16(data, 7*i);
            servo_conf[i].max = deser16(data, 7*i+2);
            servo_conf[i].middle = deser16(data, 7*i+4);
            servo_conf[i].rate = deser16(data, 7*i+6);
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
        target_bearing = deser_int16(data, 5);
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

        gps_conf.wp_radius = deser16(data, 10);
        gps_conf.safe_wp_distance = deser16(data, 12);
        gps_conf.nav_max_altitude = deser16(data, 14);
        gps_conf.nav_speed_max = deser16(data, 16);
        gps_conf.nav_speed_min = deser16(data, 18);

        gps_conf.crosstrack_gain = data[20];
        gps_conf.nav_bank_max = deser16(data, 21);
        gps_conf.rth_altitude = deser16(data, 23);
        gps_conf.land_speed = data[25];
        gps_conf.fence = deser16(data, 26);

        gps_conf.max_wp_number = data[28];

        gps_conf.checksum = data[29];
    }
};


/////////////////////////////////////////////////////////////////////
/// Response (2xx)

// MSP_SET_RAW_RC: 200
struct SetRc : public Response {
    ID id() { return ID::MSP_SET_RAW_RC; }

    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t throttle;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;

    std::vector<uint8_t> encode() {
        std::vector<uint8_t> data(RC_CHANS*2);
        for(auto channel : {roll, pitch, yaw, throttle,
                            aux1, aux2, aux3, aux4})
        {
            ser16(channel, data);
        }
        return data;
    }
};

// MSP_SET_RAW_GPS: 201
struct SetRawGPS : public Request {
    ID id() { return ID::MSP_SET_RAW_GPS; }

    uint8_t fix;
    uint8_t numSat;
    uint32_t lat;
    uint32_t lon;
    uint16_t altitude;
    uint16_t speed;

    std::vector<uint8_t> encode() {
        std::vector<uint8_t> data(14);
        data[0] = fix;
        data[1] = numSat;
        ser32(lat, data);
        ser32(lon, data);
        ser16(altitude, data);
        ser16(speed, data);
        return data;
    }
};

// MSP_SET_RC_TUNING: 204
struct SetRcTuning : public Response {
    ID id() { return ID::MSP_SET_RC_TUNING; }

    uint8_t RC_RATE;
    uint8_t RC_EXPO;
    uint8_t RollPitchRate;
    uint8_t YawRate;
    uint8_t DynThrPID;
    uint8_t Throttle_MID;
    uint8_t Throttle_EXPO;

    /**
     * @brief SetRcTuning construct a RC tuning response from a request
     * @param rc_tuning RcTuning response
     */
    SetRcTuning(const RcTuning &rc_tuning) {
        RC_RATE = rc_tuning.RC_RATE;
        RC_EXPO = rc_tuning.RC_EXPO;
        RollPitchRate = rc_tuning.RollPitchRate;
        YawRate = rc_tuning.YawRate;
        DynThrPID = rc_tuning.DynThrPID;
        Throttle_MID = rc_tuning.Throttle_MID;
        Throttle_EXPO = rc_tuning.Throttle_EXPO;
    }

    std::vector<uint8_t> encode() {
        std::vector<uint8_t> data(7);
        data[0] = RC_RATE;
        data[1] = RC_EXPO;
        data[2] = RollPitchRate;
        data[3] = YawRate;
        data[4] = DynThrPID;
        data[5] = Throttle_MID;
        data[6] = Throttle_EXPO;
        return data;
    }
};

// MSP_SET_MOTOR: 214
struct SetMotor : public Response {
    ID id() { return ID::MSP_SET_MOTOR; }

    uint16_t motor[N_MOTOR];

    std::vector<uint8_t> encode() {
        std::vector<uint8_t> data(N_MOTOR*2);
        for(unsigned int i=0; i<N_MOTOR; i++)
            ser16(motor[i], data);
        return data;
    }
};

} // namespace msp

#endif // MSP_MSG_HPP
