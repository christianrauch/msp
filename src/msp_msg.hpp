// MSP message definitions
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

#ifndef MSP_MSG_HPP
#define MSP_MSG_HPP

#include "types.hpp"

#define N_SERVO     8
#define N_MOTOR     8
#define RC_CHANS    8
#define PIDITEMS    10

namespace msp {

/////////////////////////////////////////////////////////////////////
/// de-/serialization for 16 and 32 bit unsigned integer

void ser16(const uint16_t val, ByteVector &data) {
    data.push_back(val>>0);
    data.push_back(val>>8);
}

uint16_t deser16(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8);
}

void ser32(const uint32_t val, ByteVector &data) {
    data.push_back(val>>0);
    data.push_back(val>>8);
    data.push_back(val>>16);
    data.push_back(val>>24);
}

uint32_t deser32(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8) | (data[start+2]<<16) | (data[start+3]<<24);
}


/////////////////////////////////////////////////////////////////////
/// Requests (1xx)

// MSP_IDENT: 100
struct Ident : public Request {
    ID id() { return 100; }

    uint8_t     version;
    uint8_t     type;
    uint8_t     msp_version;
    uint32_t    capability;

    void decode(const std::vector<uint8_t> &data) {
        version     = data[0];
        type        = data[1];
        msp_version = data[2];
        capability  = (data[3]<<0) | (data[4]<<8) |
                      (data[5]<<16) | (data[6]<<24);
    }
};

// MSP_STATUS: 101
struct Status : public Request {
    ID id() { return 101; }

    uint16_t    time;   // in us
    uint16_t    i2c_errors_count;
    uint16_t    sensor;
    uint32_t    flag;
    uint8_t     current_setting;

    void decode(const std::vector<uint8_t> &data) {
        time                = (data[0]<<0) | (data[1]<<8);
        i2c_errors_count    = (data[2]<<0) | (data[3]<<8);
        sensor              = (data[4]<<0) | (data[5]<<8);
        flag                = (data[6]<<0) | (data[7]<<8) |
                              (data[8]<<16) | (data[9]<<24);
        current_setting     = data[10];
    }
};

// MSP_RAW_IMU: 102
struct RawImu : public Request {
    ID id() { return 102; }

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
        accx = (data[0]<<0) | (data[1]<<8);
        accy = (data[2]<<0) | (data[3]<<8);
        accz = (data[4]<<0) | (data[5]<<8);

        gyrx = (data[6]<<0) | (data[7]<<8);
        gyry = (data[8]<<0) | (data[9]<<8);
        gyrz = (data[10]<<0) | (data[11]<<8);

        magx = (data[12]<<0) | (data[13]<<8);
        magy = (data[14]<<0) | (data[15]<<8);
        magz = (data[16]<<0) | (data[17]<<8);
    }
};

// MSP_SERVO: 103
struct Servo : public Request {
    ID id() { return 103; }

    uint16_t servo[N_SERVO];

    void decode(const std::vector<uint8_t> &data) {
        for(unsigned int i=0; i<N_SERVO; i++)
            servo[i] = deser16(data, 2*i);
    }
};

// MSP_MOTOR: 104
struct Motor : public Request {
    ID id() { return 104; }

    uint16_t motor[N_MOTOR];

    void decode(const std::vector<uint8_t> &data) {
        for(unsigned int i=0; i<N_MOTOR; i++)
            motor[i] = deser16(data, 2*i);
    }
};

// MSP_RC: 105
struct Rc : public Request {
    ID id() { return 105; }

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
    ID id() { return 106; }

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
    ID id() { return 107; }

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
    ID id() { return 108; }

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
    ID id() { return 109; }

    uint32_t EstAlt;
    uint16_t vario;

    void decode(const std::vector<uint8_t> &data) {
        EstAlt    = deser32(data, 0);
        vario    = deser16(data, 4);
    }
};

// MSP_ANALOG: 110
struct Analog : public Request {
    ID id() { return 110; }

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
    ID id() { return 111; }

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
    ID id() { return 112; }

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


/////////////////////////////////////////////////////////////////////
/// Response (2xx)

// MSP_SET_RAW_RC: 200
struct SetRc : public Response {
    ID id() { return 200; }

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
    ID id() { return 201; }

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
    ID id() { return 204; }

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

} // namespace msp

#endif // MSP_MSG_HPP
