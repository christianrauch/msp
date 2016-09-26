#ifndef MSP_ID_HPP
#define MSP_ID_HPP

namespace msp {

enum class ID : uint8_t {
    MSP_IDENT       = 100,
    MSP_STATUS      = 101,
    MSP_RAW_IMU     = 102,
    MSP_SERVO       = 103,
    MSP_MOTOR       = 104,
    MSP_RC          = 105,
    MSP_RAW_GPS     = 106,
    MSP_COMP_GPS    = 107,
    MSP_ATTITUDE    = 108,
    MSP_ALTITUDE    = 109,
    MSP_ANALOG      = 110,
    MSP_RC_TUNING   = 111,
    MSP_PID         = 112,
    MSP_BOX         = 113,
    MSP_MISC        = 114,
    MSP_MOTOR_PINS  = 115,
    MSP_BOXNAMES    = 116,
    MSP_PIDNAMES    = 117,
    MSP_WP          = 118,
    MSP_BOXIDS      = 119,
    MSP_SERVO_CONF  = 120,

    MSP_NAV_STATUS  = 121,
    MSP_NAV_CONFIG  = 122,

    MSP_CELLS       = 130,

    MSP_SET_RAW_RC      = 200,
    MSP_SET_RAW_GPS     = 201,
    MSP_SET_PID         = 202,
    MSP_SET_BOX         = 203,
    MSP_SET_RC_TUNING   = 204,
    MSP_ACC_CALIBRATION = 205,
    MSP_MAG_CALIBRATION = 206,
    MSP_SET_MISC        = 207,
    MSP_RESET_CONF      = 208,
    MSP_SET_WP          = 209,
    MSP_SELECT_SETTING  = 210,
    MSP_SET_HEAD        = 211,
    MSP_SET_SERVO_CONF  = 212,
    MSP_SET_MOTOR       = 214,
    MSP_SET_NAV_CONFIG  = 215,

    MSP_SET_ACC_TRIM    = 239,
    MSP_ACC_TRIM        = 240,
    MSP_BIND            = 241,

    MSP_EEPROM_WRITE    = 250,

    MSP_DEBUGMSG        = 253,
    MSP_DEBUG           = 254,
};

} // namespace msp

#endif // MSP_ID_HPP
