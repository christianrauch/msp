// MSP message definitions
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

#ifndef MSP_MSG_HPP
#define MSP_MSG_HPP

#include <vector>

namespace msp {

/**
 * @brief ByteVector vector of bytes
 */
typedef std::vector<uint8_t> ByteVector;

/**
 * @brief ID id of a message
 */
typedef uint8_t ID;

struct Message {
    //static const uint8_t id = 0;
    //Message() : id(getID()) {}
    //Message(uint8_t id) : id(getID()) {}
    virtual ID id() = 0;
};

// send to FC
struct Request : public Message {
    //using Message::Message;
    virtual void decode(const std::vector<uint8_t> &data) = 0;
};

// received from FC
struct Response : public Message {
    //using Message::Message;
    virtual std::vector<uint8_t> encode() = 0;
};

// MSP_IDENT: 100
struct Ident : public Request {
    ID id() { return 100; }

    uint8_t     version;
    uint8_t     type;
    uint8_t     msp_version;
    uint32_t    capability;

    //Ident() : Request(100) {}

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

// MSP_PID: 112
struct Pid : public Request {
    ID id() { return 112; }

    static const uint8_t PIDITEMS = 10;
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
