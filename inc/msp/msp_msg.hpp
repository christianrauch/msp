// MSP message definitions
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

#ifndef MSP_MSG_HPP
#define MSP_MSG_HPP

#include <array>
#include <cassert>
#include <climits>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include "Message.hpp"

/*================================================================
 * actual messages have id and the relevant encode decode methods
 * the logic for encoding and decoding must be within a message-derived class
 * non message-derived structs must have pack/unpack subroutines
 *
 */

// undefine macros defined by GNU C std library
#undef major
#undef minor

namespace msp {

enum class ID : uint16_t {
    MSP_API_VERSION                    = 1,
    MSP_FC_VARIANT                     = 2,
    MSP_FC_VERSION                     = 3,
    MSP_BOARD_INFO                     = 4,
    MSP_BUILD_INFO                     = 5,
    MSP_INAV_PID                       = 6,
    MSP_SET_INAV_PID                   = 7,
    MSP_NAME                           = 10,  // out message
    MSP_SET_NAME                       = 11,  // in message
    MSP_NAV_POSHOLD                    = 12,  // only in iNav
    MSP_SET_NAV_POSHOLD                = 13,  // only in iNav
    MSP_CALIBRATION_DATA               = 14,
    MSP_SET_CALIBRATION_DATA           = 15,
    MSP_POSITION_ESTIMATION_CONFIG     = 16,
    MSP_SET_POSITION_ESTIMATION_CONFIG = 17,
    MSP_WP_MISSION_LOAD                = 18,  // Load mission from NVRAM
    MSP_WP_MISSION_SAVE                = 19,  // Save mission to NVRAM
    MSP_WP_GETINFO                     = 20,
    MSP_RTH_AND_LAND_CONFIG            = 21,
    MSP_SET_RTH_AND_LAND_CONFIG        = 22,
    MSP_FW_CONFIG                      = 23,
    MSP_SET_FW_CONFIG                  = 24,
    MSP_BATTERY_CONFIG                 = 32,  // not avaialable in iNav
    MSP_SET_BATTERY_CONFIG             = 33,  // not avaialable in iNav
    MSP_MODE_RANGES                    = 34,
    MSP_SET_MODE_RANGE                 = 35,
    MSP_FEATURE                        = 36,
    MSP_SET_FEATURE                    = 37,
    MSP_BOARD_ALIGNMENT                = 38,
    MSP_SET_BOARD_ALIGNMENT            = 39,
    MSP_CURRENT_METER_CONFIG           = 40,
    MSP_SET_CURRENT_METER_CONFIG       = 41,
    MSP_MIXER                          = 42,
    MSP_SET_MIXER                      = 43,
    MSP_RX_CONFIG                      = 44,
    MSP_SET_RX_CONFIG                  = 45,
    MSP_LED_COLORS                     = 46,
    MSP_SET_LED_COLORS                 = 47,
    MSP_LED_STRIP_CONFIG               = 48,
    MSP_SET_LED_STRIP_CONFIG           = 49,
    MSP_RSSI_CONFIG                    = 50,
    MSP_SET_RSSI_CONFIG                = 51,
    MSP_ADJUSTMENT_RANGES              = 52,
    MSP_SET_ADJUSTMENT_RANGE           = 53,
    MSP_CF_SERIAL_CONFIG               = 54,
    MSP_SET_CF_SERIAL_CONFIG           = 55,
    MSP_VOLTAGE_METER_CONFIG           = 56,
    MSP_SET_VOLTAGE_METER_CONFIG       = 57,
    MSP_SONAR_ALTITUDE                 = 58,
    MSP_PID_CONTROLLER                 = 59,
    MSP_SET_PID_CONTROLLER             = 60,
    MSP_ARMING_CONFIG                  = 61,
    MSP_SET_ARMING_CONFIG              = 62,
    MSP_RX_MAP                         = 64,
    MSP_SET_RX_MAP                     = 65,
    MSP_BF_CONFIG                      = 66,  // depricated, out message
    MSP_SET_BF_CONFIG                  = 67,  // depricated, in message
    MSP_REBOOT                         = 68,
    MSP_BF_BUILD_INFO                  = 69,  // depricated, iNav
    MSP_DATAFLASH_SUMMARY              = 70,
    MSP_DATAFLASH_READ                 = 71,
    MSP_DATAFLASH_ERASE                = 72,
    MSP_LOOP_TIME                      = 73,  // depricated, iNav
    MSP_SET_LOOP_TIME                  = 74,  // depricated, iNav
    MSP_FAILSAFE_CONFIG                = 75,
    MSP_SET_FAILSAFE_CONFIG            = 76,
    MSP_RXFAIL_CONFIG                  = 77,  // depricated, iNav
    MSP_SET_RXFAIL_CONFIG              = 78,  // depricated, iNav
    MSP_SDCARD_SUMMARY                 = 79,
    MSP_BLACKBOX_CONFIG                = 80,
    MSP_SET_BLACKBOX_CONFIG            = 81,
    MSP_TRANSPONDER_CONFIG             = 82,
    MSP_SET_TRANSPONDER_CONFIG         = 83,
    MSP_OSD_CONFIG                     = 84,  // out message, betaflight
    MSP_SET_OSD_CONFIG                 = 85,  // in message, betaflight
    MSP_OSD_CHAR_READ                  = 86,  // out message, betaflight
    MSP_OSD_CHAR_WRITE                 = 87,
    MSP_VTX_CONFIG                     = 88,
    MSP_SET_VTX_CONFIG                 = 89,
    MSP_ADVANCED_CONFIG                = 90,
    MSP_SET_ADVANCED_CONFIG            = 91,
    MSP_FILTER_CONFIG                  = 92,
    MSP_SET_FILTER_CONFIG              = 93,
    MSP_PID_ADVANCED                   = 94,
    MSP_SET_PID_ADVANCED               = 95,
    MSP_SENSOR_CONFIG                  = 96,
    MSP_SET_SENSOR_CONFIG              = 97,
    MSP_CAMERA_CONTROL                 = 98,   // MSP_SPECIAL_PARAMETERS
    MSP_SET_ARMING_DISABLED            = 99,   // MSP_SET_SPECIAL_PARAMETERS
    MSP_IDENT                          = 100,  // depricated
    MSP_STATUS                         = 101,
    MSP_RAW_IMU                        = 102,
    MSP_SERVO                          = 103,
    MSP_MOTOR                          = 104,
    MSP_RC                             = 105,
    MSP_RAW_GPS                        = 106,
    MSP_COMP_GPS                       = 107,
    MSP_ATTITUDE                       = 108,
    MSP_ALTITUDE                       = 109,
    MSP_ANALOG                         = 110,
    MSP_RC_TUNING                      = 111,
    MSP_PID                            = 112,
    MSP_ACTIVEBOXES                    = 113,  // depricated, iNav
    MSP_MISC                           = 114,  // depricated, iNav
    MSP_MOTOR_PINS                     = 115,  // depricated, iNav
    MSP_BOXNAMES                       = 116,
    MSP_PIDNAMES                       = 117,
    MSP_WP                             = 118,
    MSP_BOXIDS                         = 119,
    MSP_SERVO_CONF                     = 120,
    MSP_NAV_STATUS                     = 121,
    MSP_NAV_CONFIG                     = 122,
    MSP_MOTOR_3D_CONFIG                = 124,
    MSP_RC_DEADBAND                    = 125,
    MSP_SENSOR_ALIGNMENT               = 126,
    MSP_LED_STRIP_MODECOLOR            = 127,
    MSP_VOLTAGE_METERS                 = 128,  // not present in iNav
    MSP_CURRENT_METERS                 = 129,  // not present in iNav
    MSP_BATTERY_STATE                  = 130,  // not present in iNav
    MSP_MOTOR_CONFIG                   = 131,  // out message
    MSP_GPS_CONFIG                     = 132,  // out message
    MSP_COMPASS_CONFIG                 = 133,  // out message
    MSP_ESC_SENSOR_DATA                = 134,  // out message
    MSP_STATUS_EX                      = 150,
    MSP_SENSOR_STATUS                  = 151,  // only iNav
    MSP_UID                            = 160,
    MSP_GPSSVINFO                      = 164,
    MSP_GPSSTATISTICS                  = 166,
    MSP_OSD_VIDEO_CONFIG               = 180,
    MSP_SET_OSD_VIDEO_CONFIG           = 181,
    MSP_DISPLAYPORT                    = 182,
    MSP_COPY_PROFILE                   = 183,  // not in iNav
    MSP_BEEPER_CONFIG                  = 184,  // not in iNav
    MSP_SET_BEEPER_CONFIG              = 185,  // not in iNav
    MSP_SET_TX_INFO                    = 186,  // in message
    MSP_TX_INFO                        = 187,  // out message
    MSP_SET_RAW_RC                     = 200,
    MSP_SET_RAW_GPS                    = 201,
    MSP_SET_PID                        = 202,
    MSP_SET_BOX                        = 203,  // depricated
    MSP_SET_RC_TUNING                  = 204,
    MSP_ACC_CALIBRATION                = 205,
    MSP_MAG_CALIBRATION                = 206,
    MSP_SET_MISC                       = 207,  // depricated
    MSP_RESET_CONF                     = 208,
    MSP_SET_WP                         = 209,
    MSP_SELECT_SETTING                 = 210,
    MSP_SET_HEADING                    = 211,
    MSP_SET_SERVO_CONF                 = 212,
    MSP_SET_MOTOR                      = 214,
    MSP_SET_NAV_CONFIG                 = 215,
    MSP_SET_MOTOR_3D_CONF              = 217,
    MSP_SET_RC_DEADBAND                = 218,
    MSP_SET_RESET_CURR_PID             = 219,
    MSP_SET_SENSOR_ALIGNMENT           = 220,
    MSP_SET_LED_STRIP_MODECOLOR        = 221,
    MSP_SET_MOTOR_CONFIG               = 222,  // out message
    MSP_SET_GPS_CONFIG                 = 223,  // out message
    MSP_SET_COMPASS_CONFIG             = 224,  // out message
    MSP_SET_ACC_TRIM                   = 239,  // in message
    MSP_ACC_TRIM                       = 240,  // out message
    MSP_SERVO_MIX_RULES                = 241,  // out message
    MSP_SET_SERVO_MIX_RULE             = 242,  // in message
    MSP_PASSTHROUGH_SERIAL             = 244,  // not used in CF, BF, iNav
    MSP_SET_4WAY_IF                    = 245,  // in message
    MSP_SET_RTC                        = 246,  // in message
    MSP_RTC                            = 247,  // out message
    MSP_EEPROM_WRITE                   = 250,  // in message
    MSP_RESERVE_1                      = 251,  // reserved for system usage
    MSP_RESERVE_2                      = 252,  // reserved for system usage
    MSP_DEBUGMSG                       = 253,  // out message
    MSP_DEBUG                          = 254,  // out message
    MSP_V2_FRAME                       = 255,  // MSPv2 over MSPv1

    MSP2_COMMON_TZ               = 0x1001,  // out message, TZ offset
    MSP2_COMMON_SET_TZ           = 0x1002,  // in message, sets the TZ offset
    MSP2_COMMON_SETTING          = 0x1003,  // in/out message, returns setting
    MSP2_COMMON_SET_SETTING      = 0x1004,  // in message, sets a setting value
    MSP2_COMMON_MOTOR_MIXER      = 0x1005,
    MSP2_COMMON_SET_MOTOR_MIXER  = 0x1006,
    MSP2_INAV_STATUS             = 0x2000,
    MSP2_INAV_OPTICAL_FLOW       = 0x2001,
    MSP2_INAV_ANALOG             = 0x2002,
    MSP2_INAV_MISC               = 0x2003,
    MSP2_INAV_SET_MISC           = 0x2004,
    MSP2_INAV_BATTERY_CONFIG     = 0x2005,
    MSP2_INAV_SET_BATTERY_CONFIG = 0x2006,
    MSP2_INAV_RATE_PROFILE       = 0x2007,
    MSP2_INAV_SET_RATE_PROFILE   = 0x2008,
    MSP2_INAV_AIR_SPEED          = 0x2009
};

enum class ArmingFlags : uint32_t {
    ARMED          = (1 << 2),
    WAS_EVER_ARMED = (1 << 3),

    ARMING_DISABLED_FAILSAFE_SYSTEM = (1 << 7),

    ARMING_DISABLED_NOT_LEVEL                    = (1 << 8),
    ARMING_DISABLED_SENSORS_CALIBRATING          = (1 << 9),
    ARMING_DISABLED_SYSTEM_OVERLOADED            = (1 << 10),
    ARMING_DISABLED_NAVIGATION_UNSAFE            = (1 << 11),
    ARMING_DISABLED_COMPASS_NOT_CALIBRATED       = (1 << 12),
    ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED = (1 << 13),
    ARMING_DISABLED_ARM_SWITCH                   = (1 << 14),
    ARMING_DISABLED_HARDWARE_FAILURE             = (1 << 15),
    ARMING_DISABLED_BOXFAILSAFE                  = (1 << 16),
    ARMING_DISABLED_BOXKILLSWITCH                = (1 << 17),
    ARMING_DISABLED_RC_LINK                      = (1 << 18),
    ARMING_DISABLED_THROTTLE                     = (1 << 19),
    ARMING_DISABLED_CLI                          = (1 << 20),
    ARMING_DISABLED_CMS_MENU                     = (1 << 21),
    ARMING_DISABLED_OSD_MENU                     = (1 << 22),
    ARMING_DISABLED_ROLLPITCH_NOT_CENTERED       = (1 << 23),
    ARMING_DISABLED_SERVO_AUTOTRIM               = (1 << 24),
    ARMING_DISABLED_OOM                          = (1 << 25),
    ARMING_DISABLED_INVALID_SETTING              = (1 << 26),

    ARMING_DISABLED_ALL_FLAGS =
        (ARMING_DISABLED_FAILSAFE_SYSTEM | ARMING_DISABLED_NOT_LEVEL |
         ARMING_DISABLED_SENSORS_CALIBRATING |
         ARMING_DISABLED_SYSTEM_OVERLOADED | ARMING_DISABLED_NAVIGATION_UNSAFE |
         ARMING_DISABLED_COMPASS_NOT_CALIBRATED |
         ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED |
         ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_HARDWARE_FAILURE |
         ARMING_DISABLED_BOXFAILSAFE | ARMING_DISABLED_BOXKILLSWITCH |
         ARMING_DISABLED_RC_LINK | ARMING_DISABLED_THROTTLE |
         ARMING_DISABLED_CLI | ARMING_DISABLED_CMS_MENU |
         ARMING_DISABLED_OSD_MENU | ARMING_DISABLED_ROLLPITCH_NOT_CENTERED |
         ARMING_DISABLED_SERVO_AUTOTRIM | ARMING_DISABLED_OOM |
         ARMING_DISABLED_INVALID_SETTING)
};

inline std::string armingFlagToString(uint32_t flag) {
    std::string val;
    if(flag & (1 << 2)) val += "ARMED ";
    if(flag & (1 << 3)) val += "WAS_EVER_ARMED ";
    if(flag & (1 << 7)) val += "ARMING_DISABLED_FAILSAFE_SYSTEM ";
    if(flag & (1 << 8)) val += "ARMING_DISABLED_NOT_LEVEL ";
    if(flag & (1 << 9)) val += "ARMING_DISABLED_SENSORS_CALIBRATING ";
    if(flag & (1 << 10)) val += "ARMING_DISABLED_SYSTEM_OVERLOADED ";
    if(flag & (1 << 11)) val += "ARMING_DISABLED_NAVIGATION_UNSAFE ";
    if(flag & (1 << 12)) val += "ARMING_DISABLED_COMPASS_NOT_CALIBRATED ";
    if(flag & (1 << 13)) val += "ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED ";
    if(flag & (1 << 14)) val += "ARMING_DISABLED_ARM_SWITCH ";
    if(flag & (1 << 15)) val += "ARMING_DISABLED_HARDWARE_FAILURE ";
    if(flag & (1 << 16)) val += "ARMING_DISABLED_BOXFAILSAFE ";
    if(flag & (1 << 17)) val += "ARMING_DISABLED_BOXKILLSWITCH ";
    if(flag & (1 << 18)) val += "ARMING_DISABLED_RC_LINK ";
    if(flag & (1 << 19)) val += "ARMING_DISABLED_THROTTLE ";
    if(flag & (1 << 20)) val += "ARMING_DISABLED_CLI ";
    if(flag & (1 << 21)) val += "ARMING_DISABLED_CMS_MENU ";
    if(flag & (1 << 22)) val += "ARMING_DISABLED_OSD_MENU ";
    if(flag & (1 << 23)) val += "ARMING_DISABLED_ROLLPITCH_NOT_CENTERED ";
    if(flag & (1 << 24)) val += "ARMING_DISABLED_SERVO_AUTOTRIM ";
    if(flag & (1 << 25)) val += "ARMING_DISABLED_OOM ";
    if(flag & (1 << 26)) val += "ARMING_DISABLED_INVALID_SETTING ";
    return val;
}

namespace msg {

const static size_t N_SERVO = 8;
const static size_t N_MOTOR = 8;

const static size_t BOARD_IDENTIFIER_LENGTH = 4;

const static size_t BUILD_DATE_LENGTH         = 11;
const static size_t BUILD_TIME_LENGTH         = 8;
const static size_t GIT_SHORT_REVISION_LENGTH = 7;

const static size_t MAX_NAME_LENGTH                     = 16;
const static size_t MAX_MODE_ACTIVATION_CONDITION_COUNT = 20;

const static size_t LED_CONFIGURABLE_COLOR_COUNT = 16;
const static size_t LED_MAX_STRIP_LENGTH         = 32;

const static size_t MAX_ADJUSTMENT_RANGE_COUNT        = 12;
const static size_t MAX_SIMULTANEOUS_ADJUSTMENT_COUNT = 6;

const static size_t OSD_ITEM_COUNT = 41;  // manual count from iNav io/osd.h

const static size_t MAX_MAPPABLE_RX_INPUTS = 4;  // unique to REVO?

const static size_t LED_MODE_COUNT          = 6;
const static size_t LED_DIRECTION_COUNT     = 6;
const static size_t LED_SPECIAL_COLOR_COUNT = 11;

enum class MultiType : uint8_t {
    TRI = 1,
    QUADP,          // 2
    QUADX,          // 3
    BI,             // 4
    GIMBAL,         // 5
    Y6,             // 6
    HEX6,           // 7
    FLYING_WING,    // 8
    Y4,             // 9
    HEX6X,          // 10
    OCTOX8,         // 11
    OCTOFLATP,      // 12
    OCTOFLATX,      // 13
    AIRPLANE,       // 14
    HELI_120_CCPM,  // 15
    HELI_90_DEG,    // 16
    VTAIL4,         // 17
    HEX6H,          // 18
    DUALCOPTER = 20,
    SINGLECOPTER,  // 21
};

enum class Capability { BIND, DYNBAL, FLAP, NAVCAP, EXTAUX };

enum class Sensor {
    Accelerometer,
    Barometer,
    Magnetometer,
    GPS,
    Sonar,
    OpticalFlow,
    Pitot,
    GeneralHealth
};

const static size_t NAUX = 4;

enum class SwitchPosition : size_t {
    LOW  = 0,
    MID  = 1,
    HIGH = 2,
};

static const std::vector<std::string> FEATURES = {"RX_PPM",
                                                  "VBAT",
                                                  "INFLIGHT_ACC_CAL",
                                                  "RX_SERIAL",
                                                  "MOTOR_STOP",
                                                  "SERVO_TILT",
                                                  "SOFTSERIAL",
                                                  "GPS",
                                                  "FAILSAFE",
                                                  "SONAR",
                                                  "TELEMETRY",
                                                  "AMPERAGE_METER",
                                                  "3D",
                                                  "RX_PARALLEL_PWM",
                                                  "RX_MSP",
                                                  "RSSI_ADC",
                                                  "LED_STRIP",
                                                  "DISPLAY",
                                                  "ONESHOT125",
                                                  "BLACKBOX",
                                                  "CHANNEL_FORWARDING",
                                                  "TRANSPONDER",
                                                  "OSD"};

enum class PID_Element : uint8_t {
    PID_ROLL = 0,
    PID_PITCH,
    PID_YAW,
    PID_POS_Z,
    PID_POS_XY,
    PID_VEL_XY,
    PID_SURFACE,
    PID_LEVEL,
    PID_HEADING,
    PID_VEL_Z,
    PID_ITEM_COUNT
};

/////////////////////////////////////////////////////////////////////
/// Cleanflight

// MSP_API_VERSION: 1
struct ApiVersion : public Message {
    ApiVersion(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_API_VERSION; }

    Value<uint8_t> protocol;
    Value<uint8_t> major;
    Value<uint8_t> minor;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(protocol);
        rc &= data.unpack(major);
        rc &= data.unpack(minor);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Api Version:" << std::endl;
        s << " API: " << major << "." << minor << std::endl;
        s << " Protocol: " << protocol << std::endl;
        return s;
    }
};

// MSP_FC_VARIANT: 2
struct FcVariant : public Message {
    FcVariant(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_FC_VARIANT; }

    Value<std::string> identifier;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(identifier, data.size());
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#FC variant:" << std::endl;
        s << " Identifier: " << identifier << std::endl;
        return s;
    }
};

// MSP_FC_VERSION: 3
struct FcVersion : public Message {
    FcVersion(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_FC_VERSION; }

    Value<uint8_t> major;
    Value<uint8_t> minor;
    Value<uint8_t> patch_level;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(major);
        rc &= data.unpack(minor);
        rc &= data.unpack(patch_level);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#FC version:" << std::endl;
        s << " Version: " << major << "." << minor << "." << patch_level
          << std::endl;
        return s;
    }
};

// MSP_BOARD_INFO: 4
struct BoardInfo : public Message {
    BoardInfo(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BOARD_INFO; }

    Value<std::string> identifier;
    Value<uint16_t> version;
    Value<uint8_t> osd_support;
    Value<uint8_t> comms_capabilites;
    Value<std::string> name;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(identifier, BOARD_IDENTIFIER_LENGTH);
        rc &= data.unpack(version);
        rc &= data.unpack(osd_support);
        rc &= data.unpack(comms_capabilites);
        uint8_t name_len = 0;
        rc &= data.unpack(name_len);
        rc &= data.unpack(name, name_len);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Board Info:" << std::endl;
        s << " Identifier: " << identifier << std::endl;
        s << " Version: " << version << std::endl;
        s << " OSD support: " << osd_support << std::endl;
        s << " Comms bitmask: " << comms_capabilites << std::endl;
        s << " Board Name: " << name << std::endl;
        return s;
    }
};

// MSP_BUILD_INFO: 5
struct BuildInfo : public Message {
    BuildInfo(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BUILD_INFO; }

    Value<std::string> buildDate;
    Value<std::string> buildTime;
    Value<std::string> shortGitRevision;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(buildDate, BUILD_DATE_LENGTH);
        rc &= data.unpack(buildTime, BUILD_TIME_LENGTH);
        rc &= data.unpack(shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Build Info:" << std::endl;
        s << " Date: " << buildDate << std::endl;
        s << " Time: " << buildTime << std::endl;
        s << " Git revision: " << shortGitRevision << std::endl;
        return s;
    }
};

struct InavPidSettings {
    Value<uint8_t> async_mode;
    Value<uint16_t> acc_task_frequency;
    Value<uint16_t> attitude_task_frequency;
    Value<uint8_t> heading_hold_rate_limit;
    Value<uint8_t> heading_hold_error_lpf_freq;
    Value<uint16_t> yaw_jump_prevention_limit;
    Value<uint8_t> gyro_lpf;
    Value<uint8_t> acc_soft_lpf_hz;
};

// MSP_INAV_PID: 6
struct InavPid : public InavPidSettings, public Message {
    InavPid(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_INAV_PID; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(async_mode);
        rc &= data.unpack(acc_task_frequency);
        rc &= data.unpack(attitude_task_frequency);
        rc &= data.unpack(heading_hold_rate_limit);
        rc &= data.unpack(heading_hold_error_lpf_freq);
        rc &= data.unpack(yaw_jump_prevention_limit);
        rc &= data.unpack(gyro_lpf);
        rc &= data.unpack(acc_soft_lpf_hz);
        // read the reserved bytes
        rc &= data.consume(4);
        return rc;
    }
};

// MSP_SET_INAV_PID: 7
struct SetInavPid : public InavPidSettings, public Message {
    SetInavPid(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_INAV_PID; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(async_mode);
        rc &= data->pack(acc_task_frequency);
        rc &= data->pack(attitude_task_frequency);
        rc &= data->pack(heading_hold_rate_limit);
        rc &= data->pack(heading_hold_error_lpf_freq);
        rc &= data->pack(yaw_jump_prevention_limit);
        rc &= data->pack(gyro_lpf);
        rc &= data->pack(acc_soft_lpf_hz);
        // write the reserved bytes
        rc &= data->pack(uint32_t(0));
        if(!rc) data.reset();
        return data;
    }
};

// MSP_NAME: 10
struct BoardName : public Message {
    BoardName(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_NAME; }

    Value<std::string> name;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(name);
    }
};

// MSP_SET_NAME: 11
struct SetBoardName : public Message {
    SetBoardName(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_NAME; }

    Value<std::string> name;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(name, MAX_NAME_LENGTH)) data.reset();
        return data;
    }
};

struct NavPosHoldSettings {
    Value<uint8_t> user_control_mode;
    Value<uint16_t> max_auto_speed;
    Value<uint16_t> max_auto_climb_rate;
    Value<uint16_t> max_manual_speed;
    Value<uint16_t> max_manual_climb_rate;
    Value<uint8_t> max_bank_angle;
    Value<uint8_t> use_thr_mid_for_althold;
    Value<uint16_t> hover_throttle;
};

// MSP_NAV_POSHOLD: 12
struct NavPosHold : public NavPosHoldSettings, public Message {
    NavPosHold(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_NAV_POSHOLD; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(user_control_mode);
        rc &= data.unpack(max_auto_speed);
        rc &= data.unpack(max_auto_climb_rate);
        rc &= data.unpack(max_manual_speed);
        rc &= data.unpack(max_manual_climb_rate);
        rc &= data.unpack(max_bank_angle);
        rc &= data.unpack(use_thr_mid_for_althold);
        rc &= data.unpack(hover_throttle);
        return rc;
    }
};

// MSP_SET_NAV_POSHOLD: 13
struct SetNavPosHold : public NavPosHoldSettings, public Message {
    SetNavPosHold(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_NAV_POSHOLD; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(user_control_mode);
        rc &= data->pack(max_auto_speed);
        rc &= data->pack(max_auto_climb_rate);
        rc &= data->pack(max_manual_speed);
        rc &= data->pack(max_manual_climb_rate);
        rc &= data->pack(max_bank_angle);
        rc &= data->pack(use_thr_mid_for_althold);
        rc &= data->pack(hover_throttle);
        if(!rc) data.reset();
        return data;
    }
};

struct CalibrationDataSettings {
    Value<uint16_t> acc_zero_x;
    Value<uint16_t> acc_zero_y;
    Value<uint16_t> acc_zero_z;
    Value<uint16_t> acc_gain_x;
    Value<uint16_t> acc_gain_y;
    Value<uint16_t> acc_gain_z;
};

// MSP_CALIBRATION_DATA: 14
struct CalibrationData : public CalibrationDataSettings, public Message {
    CalibrationData(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_CALIBRATION_DATA; }

    Value<uint8_t> axis_calibration_flags;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(axis_calibration_flags);
        rc &= data.unpack(acc_zero_x);
        rc &= data.unpack(acc_zero_y);
        rc &= data.unpack(acc_zero_z);
        rc &= data.unpack(acc_gain_x);
        rc &= data.unpack(acc_gain_y);
        rc &= data.unpack(acc_gain_z);
        return rc;
    }
};

// MSP_SET_CALIBRATION_DATA: 15
struct SetCalibrationData : public CalibrationDataSettings, public Message {
    SetCalibrationData(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_CALIBRATION_DATA; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(acc_zero_x);
        rc &= data->pack(acc_zero_y);
        rc &= data->pack(acc_zero_z);
        rc &= data->pack(acc_gain_x);
        rc &= data->pack(acc_gain_y);
        rc &= data->pack(acc_gain_z);
        if(!rc) data.reset();
        return data;
    }
};

struct PositionEstimationConfigSettings {
    Value<float> w_z_baro_p;
    Value<float> w_z_gps_p;
    Value<float> w_z_gps_v;
    Value<float> w_xy_gps_p;
    Value<float> w_xy_gps_v;
    Value<uint8_t> gps_min_sats;
    Value<bool> use_gps_vel_NED;
};

// MSP_POSITION_ESTIMATION_CONFIG: 16
struct PositionEstimationConfig : public PositionEstimationConfigSettings,
                                  public Message {
    PositionEstimationConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override {
        return ID::MSP_POSITION_ESTIMATION_CONFIG;
    }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack<uint16_t>(w_z_baro_p, 0.01);
        rc &= data.unpack<uint16_t>(w_z_gps_p, 0.01);
        rc &= data.unpack<uint16_t>(w_z_gps_v, 0.01);
        rc &= data.unpack<uint16_t>(w_xy_gps_p, 0.01);
        rc &= data.unpack<uint16_t>(w_xy_gps_v, 0.01);
        rc &= data.unpack(gps_min_sats);
        rc &= data.unpack(use_gps_vel_NED);
        return rc;
    }
};

// MSP_SET_POSITION_ESTIMATION_CONFIG: 17
struct SetPositionEstimationConfig : public PositionEstimationConfigSettings,
                                     public Message {
    SetPositionEstimationConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override {
        return ID::MSP_SET_POSITION_ESTIMATION_CONFIG;
    }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(static_cast<uint16_t>(w_z_baro_p() * 100));
        rc &= data->pack(static_cast<uint16_t>(w_z_gps_p() * 100));
        rc &= data->pack(static_cast<uint16_t>(w_z_gps_v() * 100));
        rc &= data->pack(static_cast<uint16_t>(w_xy_gps_p() * 100));
        rc &= data->pack(static_cast<uint16_t>(w_xy_gps_v() * 100));
        rc &= data->pack(gps_min_sats);
        rc &= data->pack(use_gps_vel_NED);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_WP_MISSION_LOAD: 18
struct WpMissionLoad : public Message {
    WpMissionLoad(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_WP_MISSION_LOAD; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(uint8_t(0))) data.reset();
        return data;
    }
};

// MSP_WP_MISSION_SAVE: 19
struct WpMissionSave : public Message {
    WpMissionSave(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_WP_MISSION_SAVE; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(uint8_t(0))) data.reset();
        return data;
    }
};

// MSP_WP_GETINFO: 20
struct WpGetInfo : public Message {
    WpGetInfo(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_WP_GETINFO; }

    Value<uint8_t> wp_capabilites;
    Value<uint8_t> max_waypoints;
    Value<bool> wp_list_valid;
    Value<uint8_t> wp_count;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(wp_capabilites);
        rc &= data.unpack(max_waypoints);
        rc &= data.unpack(wp_list_valid);
        rc &= data.unpack(wp_count);
        return rc;
    }
};

struct RthAndLandConfigSettings {
    Value<uint16_t> min_rth_distance;
    Value<uint8_t> rth_climb_first;
    Value<uint8_t> rth_climb_ignore_emerg;
    Value<uint8_t> rth_tail_first;
    Value<uint8_t> rth_allow_landing;
    Value<uint8_t> rth_alt_control_mode;
    Value<uint16_t> rth_abort_threshold;
    Value<uint16_t> rth_altitude;
    Value<uint16_t> land_descent_rate;
    Value<uint16_t> land_slowdown_minalt;
    Value<uint16_t> land_slowdown_maxalt;
    Value<uint16_t> emerg_descent_rate;
};

// MSP_RTH_AND_LAND_CONFIG: 21
struct RthAndLandConfig : public RthAndLandConfigSettings, public Message {
    RthAndLandConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RTH_AND_LAND_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(min_rth_distance);
        rc &= data.unpack(rth_climb_first);
        rc &= data.unpack(rth_climb_ignore_emerg);
        rc &= data.unpack(rth_tail_first);
        rc &= data.unpack(rth_allow_landing);
        rc &= data.unpack(rth_alt_control_mode);
        rc &= data.unpack(rth_abort_threshold);
        rc &= data.unpack(rth_altitude);
        rc &= data.unpack(land_descent_rate);
        rc &= data.unpack(land_slowdown_minalt);
        rc &= data.unpack(land_slowdown_maxalt);
        rc &= data.unpack(emerg_descent_rate);
        return rc;
    }
};

// MSP_SET_RTH_AND_LAND_CONFIG: 22
struct SetRthAndLandConfig : public RthAndLandConfigSettings, public Message {
    SetRthAndLandConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RTH_AND_LAND_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(min_rth_distance);
        rc &= data->pack(rth_climb_first);
        rc &= data->pack(rth_climb_ignore_emerg);
        rc &= data->pack(rth_tail_first);
        rc &= data->pack(rth_allow_landing);
        rc &= data->pack(rth_alt_control_mode);
        rc &= data->pack(rth_abort_threshold);
        rc &= data->pack(rth_altitude);
        rc &= data->pack(land_descent_rate);
        rc &= data->pack(land_slowdown_minalt);
        rc &= data->pack(land_slowdown_maxalt);
        rc &= data->pack(emerg_descent_rate);
        if(!rc) data.reset();
        return data;
    }
};

struct FwConfigSettings {
    Value<uint16_t> cruise_throttle;
    Value<uint16_t> min_throttle;
    Value<uint16_t> max_throttle;
    Value<uint8_t> max_bank_angle;
    Value<uint8_t> max_climb_angle;
    Value<uint8_t> max_dive_angle;
    Value<uint8_t> pitch_to_throttle;
    Value<uint16_t> loiter_radius;
};

// MSP_FW_CONFIG: 23
struct FwConfig : public FwConfigSettings, public Message {
    FwConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_FW_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(cruise_throttle);
        rc &= data.unpack(min_throttle);
        rc &= data.unpack(max_throttle);
        rc &= data.unpack(max_bank_angle);
        rc &= data.unpack(max_climb_angle);
        rc &= data.unpack(max_dive_angle);
        rc &= data.unpack(pitch_to_throttle);
        rc &= data.unpack(loiter_radius);
        return rc;
    }
};

// MSP_SET_FW_CONFIG: 24
struct SetFwConfig : public FwConfigSettings, public Message {
    SetFwConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_FW_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(cruise_throttle);
        rc &= data->pack(min_throttle);
        rc &= data->pack(max_throttle);
        rc &= data->pack(max_bank_angle);
        rc &= data->pack(max_climb_angle);
        rc &= data->pack(max_dive_angle);
        rc &= data->pack(pitch_to_throttle);
        rc &= data->pack(loiter_radius);
        if(!rc) data.reset();
        return data;
    }
};

struct BatteryConfigSettings {
    Value<uint8_t> vbatmincellvoltage;
    Value<uint8_t> vbatmaxcellvoltage;
    Value<uint8_t> vbatwarningcellvoltage;
    Value<uint16_t> batteryCapacity;
    Value<uint8_t> voltageMeterSource;
    Value<uint8_t> currentMeterSource;
};

// MSP_BATTERY_CONFIG: 32
struct BatteryConfig : public BatteryConfigSettings, public Message {
    BatteryConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BATTERY_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(vbatmincellvoltage);
        rc &= data.unpack(vbatmaxcellvoltage);
        rc &= data.unpack(vbatwarningcellvoltage);
        rc &= data.unpack(batteryCapacity);
        rc &= data.unpack(voltageMeterSource);
        rc &= data.unpack(currentMeterSource);
        return rc;
    }
};

// MSP_SET_BATTERY_CONFIG: 33
struct SetBatteryConfig : public BatteryConfigSettings, public Message {
    SetBatteryConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_BATTERY_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(vbatmincellvoltage);
        rc &= data->pack(vbatmaxcellvoltage);
        rc &= data->pack(vbatwarningcellvoltage);
        rc &= data->pack(batteryCapacity);
        rc &= data->pack(voltageMeterSource);
        rc &= data->pack(currentMeterSource);
        if(!rc) data.reset();
        return data;
    }
};

struct box_description {
    Value<uint8_t> id;
    Value<uint8_t> aux_channel_index;
    Value<uint8_t> startStep;
    Value<uint8_t> endStep;
};

// MSP_MODE_RANGES: 34
struct ModeRanges : public Message {
    ModeRanges(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_MODE_RANGES; }

    std::array<box_description, MAX_MODE_ACTIVATION_CONDITION_COUNT> boxes;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(size_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            rc &= data.unpack(boxes[i].id);
            rc &= data.unpack(boxes[i].aux_channel_index);
            rc &= data.unpack(boxes[i].startStep);
            rc &= data.unpack(boxes[i].endStep);
        }
        return rc;
    }
};

// MSP_SET_MODE_RANGE: 35
struct SetModeRange : public Message {
    SetModeRange(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_MODE_RANGE; }

    Value<uint8_t> mode_activation_condition_idx;
    box_description box;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(mode_activation_condition_idx);
        rc &= data->pack(box.id);
        rc &= data->pack(box.aux_channel_index);
        rc &= data->pack(box.startStep);
        rc &= data->pack(box.endStep);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_FEATURE: 36
struct Feature : public Message {
    Feature(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_FEATURE; }

    std::set<std::string> features;

    virtual bool decode(const ByteVector& data) override {
        uint32_t mask;
        bool rc = data.unpack(mask);
        if(!rc) return rc;
        features.clear();
        for(size_t ifeat(0); ifeat < FEATURES.size(); ifeat++) {
            if(mask & (1 << ifeat)) features.insert(FEATURES[ifeat]);
        }
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Features:" << std::endl;
        for(const std::string& f : features) {
            s << f << std::endl;
        }
        return s;
    }
};

// MSP_SET_FEATURE: 37
struct SetFeature : public Message {
    SetFeature(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_FEATURE; }

    std::set<std::string> features;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        uint32_t mask       = 0;
        for(size_t ifeat(0); ifeat < FEATURES.size(); ifeat++) {
            if(features.count(FEATURES[ifeat])) mask |= 1 << ifeat;
        }
        if(!data->pack(mask)) data.reset();
        return data;
    }
};

// iNav uses decidegrees, BF/CF use degrees
struct BoardAlignmentSettings {
    Value<uint16_t> roll;
    Value<uint16_t> pitch;
    Value<uint16_t> yaw;
};

// MSP_BOARD_ALIGNMENT: 38
struct BoardAlignment : public BoardAlignmentSettings, public Message {
    BoardAlignment(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BOARD_ALIGNMENT; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(roll);
        rc &= data.unpack(pitch);
        rc &= data.unpack(yaw);
        return rc;
    }
};

// MSP_SET_BOARD_ALIGNMENT: 39
struct SetBoardAlignment : public BoardAlignmentSettings, public Message {
    SetBoardAlignment(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BOARD_ALIGNMENT; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(roll);
        rc &= data->pack(pitch);
        rc &= data->pack(yaw);
        if(!rc) data.reset();
        return data;
    }
};

struct CurrentMeterConfigSettings {
    Value<uint16_t> currnet_scale;
    Value<uint16_t> current_offset;
    Value<uint8_t> current_type;
    Value<uint16_t> capacity;
};

// MSP_CURRENT_METER_CONFIG: 40 (differs from Cleanflight/BetaFlight)
struct CurrentMeterConfig : public CurrentMeterConfigSettings, public Message {
    CurrentMeterConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_CURRENT_METER_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(currnet_scale);
        rc &= data.unpack(current_offset);
        rc &= data.unpack(current_type);
        rc &= data.unpack(capacity);
        return rc;
    }
};

// MSP_SET_CURRENT_METER_CONFIG: 41 (differs from Cleanflight/BetaFlight)
struct SetCurrentMeterConfig : public CurrentMeterConfigSettings,
                               public Message {
    SetCurrentMeterConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_CURRENT_METER_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(currnet_scale);
        rc &= data->pack(current_offset);
        rc &= data->pack(current_type);
        rc &= data->pack(capacity);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_MIXER: 42
struct Mixer : public Message {
    Mixer(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_MIXER; }

    Value<uint8_t> mode;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(mode);
    }
};

// MSP_SET_MIXER: 43
struct SetMixer : public Message {
    SetMixer(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_MIXER; }

    Value<uint8_t> mode;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(mode)) data.reset();
        return data;
    }
};

struct RxConfigSettings {
    size_t valid_data_groups;
    // group1
    Value<uint8_t> serialrx_provider;
    Value<uint16_t> maxcheck;
    Value<uint16_t> midrc;
    Value<uint16_t> mincheck;
    Value<uint8_t> spektrum_sat_bind;
    // group 2
    Value<uint16_t> rx_min_usec;
    Value<uint16_t> rx_max_usec;
    // group 3
    Value<uint8_t> rcInterpolation;
    Value<uint8_t> rcInterpolationInterval;
    Value<uint16_t> airModeActivateThreshold;
    // group 4
    Value<uint8_t> rx_spi_protocol;
    Value<uint32_t> rx_spi_id;
    Value<uint8_t> rx_spi_rf_channel_count;
    // group 5
    Value<uint8_t> fpvCamAngleDegrees;
    // group 6 - iNav only
    Value<uint8_t> receiverType;

    std::ostream& rxConfigPrint(std::ostream& s) const {
        s << "#RX configuration:" << std::endl;
        s << " serialrx_provider: " << serialrx_provider << std::endl;
        s << " maxcheck: " << maxcheck << std::endl;
        s << " midrc: " << midrc << std::endl;
        s << " mincheck: " << mincheck << std::endl;
        s << " spektrum_sat_bind: " << spektrum_sat_bind << std::endl;
        s << " rx_min_usec: " << rx_min_usec << std::endl;
        s << " rx_max_usec: " << rx_max_usec << std::endl;
        s << " rcInterpolation: " << rcInterpolation << std::endl;
        s << " rcInterpolationInterval: " << rcInterpolationInterval
          << std::endl;
        s << " airModeActivateThreshold: " << airModeActivateThreshold
          << std::endl;
        s << " rx_spi_protocol: " << rx_spi_protocol << std::endl;
        s << " rx_spi_id: " << rx_spi_id << std::endl;
        s << " rx_spi_rf_channel_count: " << rx_spi_rf_channel_count
          << std::endl;
        s << " fpvCamAngleDegrees: " << fpvCamAngleDegrees << std::endl;
        s << " receiverType: " << receiverType << std::endl;
        return s;
    }
};

// MSP_RX_CONFIG: 44
struct RxConfig : public RxConfigSettings, public Message {
    RxConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RX_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc           = true;
        valid_data_groups = 1;
        rc &= data.unpack(serialrx_provider);
        rc &= data.unpack(maxcheck);
        rc &= data.unpack(midrc);
        rc &= data.unpack(mincheck);
        rc &= data.unpack(spektrum_sat_bind);
        if(data.unpacking_remaining() == 0) return rc;

        valid_data_groups += 1;
        rc &= data.unpack(rx_min_usec);
        rc &= data.unpack(rx_max_usec);
        if(data.unpacking_remaining() == 0) return rc;

        valid_data_groups += 1;
        rc &= data.unpack(rcInterpolation);
        rc &= data.unpack(rcInterpolationInterval);
        rc &= data.unpack(airModeActivateThreshold);
        if(data.unpacking_remaining() == 0) return rc;

        valid_data_groups += 1;
        rc &= data.unpack(rx_spi_protocol);
        rc &= data.unpack(rx_spi_id);
        rc &= data.unpack(rx_spi_rf_channel_count);
        if(data.unpacking_remaining() == 0) return rc;

        valid_data_groups += 1;
        rc &= data.unpack(fpvCamAngleDegrees);
        if(data.unpacking_remaining() == 0) return rc;

        valid_data_groups += 1;
        rc &= data.unpack(receiverType);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        return rxConfigPrint(s);
    }
};

// MSP_SET_RX_CONFIG: 45
struct SetRxConfig : public RxConfigSettings, public Message {
    SetRxConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RX_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(serialrx_provider);
        rc &= data->pack(maxcheck);
        rc &= data->pack(midrc);
        rc &= data->pack(mincheck);
        rc &= data->pack(spektrum_sat_bind);
        if(valid_data_groups == 1) goto packing_finished;
        rc &= data->pack(rx_min_usec);
        rc &= data->pack(rx_max_usec);
        if(valid_data_groups == 2) goto packing_finished;
        rc &= data->pack(rcInterpolation);
        rc &= data->pack(rcInterpolationInterval);
        rc &= data->pack(airModeActivateThreshold);
        if(valid_data_groups == 3) goto packing_finished;
        rc &= data->pack(rx_spi_protocol);
        rc &= data->pack(rx_spi_id);
        rc &= data->pack(rx_spi_rf_channel_count);
        if(valid_data_groups == 4) goto packing_finished;
        rc &= data->pack(fpvCamAngleDegrees);
        if(valid_data_groups == 5) goto packing_finished;
        rc &= data->pack(receiverType);
    packing_finished:
        if(!rc) data.reset();
        return data;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        return rxConfigPrint(s);
    }
};

struct HsvColor : public Packable {
    Value<uint16_t> h;
    Value<uint8_t> s;
    Value<uint8_t> v;

    bool unpack_from(const ByteVector& data) {
        bool rc = true;
        rc &= data.unpack(h);
        rc &= data.unpack(s);
        rc &= data.unpack(v);
        return rc;
    }

    bool pack_into(ByteVector& data) const {
        bool rc = true;
        rc &= data.pack(h);
        rc &= data.pack(s);
        rc &= data.pack(v);
        return rc;
    }
};

// MSP_LED_COLORS: 46
struct LedColors : public Message {
    LedColors(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_LED_COLORS; }

    std::array<HsvColor, LED_CONFIGURABLE_COLOR_COUNT> colors;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(auto& c : colors) {
            rc &= data.unpack(c);
        }
        return rc;
    }
};

// MSP_SET_LED_COLORS: 47
struct SetLedColors : public Message {
    SetLedColors(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_LED_COLORS; }

    std::array<HsvColor, LED_CONFIGURABLE_COLOR_COUNT> colors;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        for(size_t i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; ++i) {
            rc &= data->pack(colors[i].h);
            rc &= data->pack(colors[i].s);
            rc &= data->pack(colors[i].v);
        }
        if(!rc) data.reset();
        return data;
    }
};

// MSP_LED_STRIP_CONFIG: 48
struct LedStripConfigs : public Message {
    LedStripConfigs(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_LED_STRIP_CONFIG; }

    std::array<uint32_t, LED_MAX_STRIP_LENGTH> configs;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(size_t i = 0; i < LED_MAX_STRIP_LENGTH; ++i) {
            rc &= data.unpack(configs[i]);
        }
        return rc;
    }
};

// MSP_SET_LED_STRIP_CONFIG: 49
struct SetLedStripConfig : public Message {
    SetLedStripConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_LED_STRIP_CONFIG; }

    Value<uint8_t> cfg_index;
    Value<uint32_t> config;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(cfg_index);
        rc &= data->pack(config);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_RSSI_CONFIG: 50
struct RssiConfig : public Message {
    RssiConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RSSI_CONFIG; }

    Value<uint8_t> rssi_channel;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(rssi_channel);
    }
};

// MSP_SET_RSSI_CONFIG: 51
struct SetRssiConfig : public Message {
    SetRssiConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RSSI_CONFIG; }

    Value<uint8_t> rssi_channel;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(rssi_channel)) data.reset();
        return data;
    }
};

struct adjustmentRange {
    Value<uint8_t> adjustmentIndex;
    Value<uint8_t> auxChannelIndex;
    Value<uint8_t> range_startStep;
    Value<uint8_t> range_endStep;
    Value<uint8_t> adjustmentFunction;
    Value<uint8_t> auxSwitchChannelIndex;
};

// MSP_ADJUSTMENT_RANGES: 52
struct AdjustmentRanges : public Message {
    AdjustmentRanges(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ADJUSTMENT_RANGES; }

    std::array<adjustmentRange, MAX_ADJUSTMENT_RANGE_COUNT> ranges;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(size_t i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; ++i) {
            rc &= data.unpack(ranges[i].adjustmentIndex);
            rc &= data.unpack(ranges[i].auxChannelIndex);
            rc &= data.unpack(ranges[i].range_startStep);
            rc &= data.unpack(ranges[i].range_endStep);
            rc &= data.unpack(ranges[i].adjustmentFunction);
            rc &= data.unpack(ranges[i].auxSwitchChannelIndex);
        }
        return rc;
    }
};

// MSP_SET_ADJUSTMENT_RANGE: 53
struct SetAdjustmentRange : public Message {
    SetAdjustmentRange(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_ADJUSTMENT_RANGE; }

    Value<uint8_t> range_index;
    adjustmentRange range;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(range_index);
        rc &= data->pack(range.adjustmentIndex);
        rc &= data->pack(range.auxChannelIndex);
        rc &= data->pack(range.range_startStep);
        rc &= data->pack(range.range_endStep);
        rc &= data->pack(range.adjustmentFunction);
        rc &= data->pack(range.auxSwitchChannelIndex);
        if(!rc) data.reset();
        return data;
    }
};

struct CfSerialConfigSettings {
    Value<uint8_t> identifier;
    Value<uint16_t> functionMask;
    Value<uint8_t> mspBaudrateIndx;
    Value<uint8_t> gpsBaudrateIndx;
    Value<uint8_t> telemetryBaudrateIndx;
    Value<uint8_t> peripheralBaudrateIndx;
};

// MSP_CF_SERIAL_CONFIG: 54
struct CfSerialConfig : public Message {
    CfSerialConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_CF_SERIAL_CONFIG; }

    std::vector<CfSerialConfigSettings> configs;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        do {
            CfSerialConfigSettings tmp;
            rc &= data.unpack(tmp.identifier);
            rc &= data.unpack(tmp.functionMask);
            rc &= data.unpack(tmp.mspBaudrateIndx);
            rc &= data.unpack(tmp.gpsBaudrateIndx);
            rc &= data.unpack(tmp.telemetryBaudrateIndx);
            rc &= data.unpack(tmp.peripheralBaudrateIndx);
            if(rc) configs.push_back(tmp);
        } while(rc);
        return configs.size();
    }
};

// MSP_SET_CF_SERIAL_CONFIG: 55
struct SetCfSerialConfig : public Message {
    SetCfSerialConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_CF_SERIAL_CONFIG; }

    std::vector<CfSerialConfigSettings> configs;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        for(const auto& config : configs) {
            rc &= data->pack(config.identifier);
            rc &= data->pack(config.functionMask);
            rc &= data->pack(config.mspBaudrateIndx);
            rc &= data->pack(config.gpsBaudrateIndx);
            rc &= data->pack(config.telemetryBaudrateIndx);
            rc &= data->pack(config.peripheralBaudrateIndx);
        }
        if(!rc) data.reset();
        return data;
    }
};

struct VoltageMeterConfigSettings {
    Value<uint8_t> scale_dV;
    Value<uint8_t> cell_min_dV;
    Value<uint8_t> cell_max_dV;
    Value<uint8_t> cell_warning_dV;
};

// MSP_VOLTAGE_METER_CONFIG: 56 (differs from Cleanflight/BetaFlight)
struct VoltageMeterConfig : public VoltageMeterConfigSettings, public Message {
    VoltageMeterConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_VOLTAGE_METER_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(scale_dV);
        rc &= data.unpack(cell_min_dV);
        rc &= data.unpack(cell_max_dV);
        rc &= data.unpack(cell_warning_dV);
        return rc;
    }
};

// MSP_SET_VOLTAGE_METER_CONFIG: 57 (differs from Cleanflight/BetaFlight)
struct SetVoltageMeterConfig : public VoltageMeterConfigSettings,
                               public Message {
    SetVoltageMeterConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_VOLTAGE_METER_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(scale_dV);
        rc &= data->pack(cell_min_dV);
        rc &= data->pack(cell_max_dV);
        rc &= data->pack(cell_warning_dV);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SONAR_ALTITUDE: 58
struct SonarAltitude : public Message {
    SonarAltitude(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SONAR_ALTITUDE; }

    Value<uint32_t> altitude_cm;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(altitude_cm);
    }
};

// MSP_PID_CONTROLLER: 59
struct PidController : public Message {
    PidController(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_PID_CONTROLLER; }

    Value<uint8_t> controller_id;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(controller_id);
    }
};

// MSP_SET_PID_CONTROLLER: 60
struct SetPidController : public Message {
    SetPidController(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_PID_CONTROLLER; }
};

struct ArmingConfigSettings {
    Value<uint8_t> auto_disarm_delay;
    Value<uint8_t> disarm_kill_switch;
    bool imu_small_angle_valid;
    Value<uint8_t> imu_small_angle;
};

// MSP_ARMING_CONFIG: 61
struct ArmingConfig : public ArmingConfigSettings, public Message {
    ArmingConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ARMING_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(auto_disarm_delay);
        rc &= data.unpack(disarm_kill_switch);
        if(data.unpack(imu_small_angle)) imu_small_angle_valid = true;
        return rc;
    }
};

// MSP_SET_ARMING_CONFIG: 62
struct SetArmingConfig : public ArmingConfigSettings, public Message {
    SetArmingConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_ARMING_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(auto_disarm_delay);
        rc &= data->pack(disarm_kill_switch);
        if(imu_small_angle_valid) rc &= data->pack(imu_small_angle);
        if(!rc) data.reset();
        return data;
    }
};

struct RxMapSettings {
    std::array<uint8_t, MAX_MAPPABLE_RX_INPUTS> map;

    std::ostream& printRxMapSettings(std::ostream& s) const {
        s << "#Channel mapping:" << std::endl;
        for(size_t i(0); i < map.size(); i++) {
            s << " " << i << ": " << size_t(map[i]) << std::endl;
        }
        return s;
    }
};

// MSP_RX_MAP: 64
struct RxMap : public RxMapSettings, public Message {
    RxMap(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RX_MAP; }

    virtual bool decode(const ByteVector& data) override {
        if(data.size() < MAX_MAPPABLE_RX_INPUTS) return false;
        bool rc = true;
        for(size_t i = 0; i < MAX_MAPPABLE_RX_INPUTS; ++i) {
            rc &= data.unpack(map[i]);
        }
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        return printRxMapSettings(s);
    }
};

// MSP_SET_RX_MAP: 65
struct SetRxMap : public RxMapSettings, public Message {
    SetRxMap(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RX_MAP; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        for(const auto& channel : map) {
            rc &= data->pack(channel);
        }
        if(!rc) data.reset();
        return data;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        return printRxMapSettings(s);
    }
};

// iNav uses decidegrees, BF/CF use degrees
struct BfConfigSettings {
    Value<uint8_t> mixer_mode;
    Value<uint32_t> feature_mask;
    Value<uint8_t> serialrx_provider;
    Value<uint16_t> roll;
    Value<uint16_t> pitch;
    Value<uint16_t> yaw;
    Value<uint16_t> current_meter_scale;
    Value<uint16_t> current_meter_offset;
};

// MSP_BF_CONFIG: 66, //out message baseflight-specific settings that aren't
// covered elsewhere
struct BfConfig : public BfConfigSettings, public Message {
    BfConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BF_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(mixer_mode);
        rc &= data.unpack(feature_mask);
        rc &= data.unpack(serialrx_provider);
        rc &= data.unpack(roll);
        rc &= data.unpack(pitch);
        rc &= data.unpack(yaw);
        rc &= data.unpack(current_meter_scale);
        rc &= data.unpack(current_meter_offset);
        return rc;
    }
};

// MSP_SET_BF_CONFIG: 67, //in message baseflight-specific settings save
struct SetBfConfig : public BfConfigSettings, public Message {
    SetBfConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_BF_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(mixer_mode);
        rc &= data->pack(feature_mask);
        rc &= data->pack(serialrx_provider);
        rc &= data->pack(roll);
        rc &= data->pack(pitch);
        rc &= data->pack(yaw);
        rc &= data->pack(current_meter_scale);
        rc &= data->pack(current_meter_offset);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_REBOOT: 68
struct Reboot : public Message {
    Reboot(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_REBOOT; }
};

// MSP_BF_BUILD_INFO: 69
struct BfBuildInfo : public Message {
    BfBuildInfo(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BF_BUILD_INFO; }

    Value<std::string> build_date;
    Value<uint32_t> reserved1;
    Value<uint32_t> reserved2;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(build_date, 11);
        rc &= data.unpack(reserved1);
        rc &= data.unpack(reserved2);
        return rc;
    }
};

// MSP_DATAFLASH_SUMMARY: 70
struct DataflashSummary : public Message {
    DataflashSummary(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_DATAFLASH_SUMMARY; }

    bool flash_is_ready;
    Value<uint32_t> sectors;
    Value<uint32_t> total_size;
    Value<uint32_t> offset;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(flash_is_ready);
        rc &= data.unpack(sectors);
        rc &= data.unpack(total_size);
        rc &= data.unpack(offset);
        return rc;
    }
};

// message format differs between iNav and BF/CF
// MSP_DATAFLASH_READ: 71
struct DataflashRead : public Message {
    DataflashRead(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_DATAFLASH_READ; }

    Value<uint32_t> read_address;
    Value<uint16_t> read_size;
    bool allow_compression;
    ByteVector flash_data;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(read_address);
        rc &= data->pack(read_size);
        rc &= data->pack(allow_compression);
        if(!rc) data.reset();
        return data;
    }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(read_address);
        flash_data = ByteVector(data.unpacking_iterator(), data.end());
        rc &= data.consume(flash_data.size());
        return rc;
    }
};

// MSP_DATAFLASH_ERASE: 72
struct DataflashErase : public Message {
    DataflashErase(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_DATAFLASH_ERASE; }

    virtual bool decode(const ByteVector& /*data*/) override { return true; }
};

// MSP_LOOP_TIME: 73
struct LoopTime : public Message {
    LoopTime(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_LOOP_TIME; }

    Value<uint16_t> loop_time;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(loop_time);
    }
};

// MSP_SET_LOOP_TIME:74
struct SetLoopTime : public Message {
    SetLoopTime(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_LOOP_TIME; }

    Value<uint16_t> loop_time;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(loop_time)) data.reset();
        return data;
    }
};

struct FailsafeSettings {
    bool extended_contents;
    Value<uint8_t> delay;
    Value<uint8_t> off_delay;
    Value<uint16_t> throttle;
    Value<uint8_t> kill_switch;
    Value<uint16_t> throttle_low_delay;
    Value<uint8_t> procedure;
    Value<uint8_t> recovery_delay;
    Value<uint16_t> fw_roll_angle;
    Value<uint16_t> fw_pitch_angle;
    Value<uint16_t> fw_yaw_rate;
    Value<uint16_t> stick_motion_threshold;
    Value<uint16_t> min_distance;
    Value<uint8_t> min_distance_procedure;
};

// MSP_FAILSAFE_CONFIG: 75
struct FailsafeConfig : public FailsafeSettings, public Message {
    FailsafeConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_FAILSAFE_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc           = true;
        extended_contents = false;
        rc &= data.unpack(delay);
        rc &= data.unpack(off_delay);
        rc &= data.unpack(throttle);
        rc &= data.unpack(kill_switch);
        rc &= data.unpack(throttle_low_delay);
        rc &= data.unpack(procedure);
        if(data.unpacking_remaining() == 0) return rc;
        extended_contents = true;
        rc &= data.unpack(recovery_delay);
        rc &= data.unpack(fw_roll_angle);
        rc &= data.unpack(fw_pitch_angle);
        rc &= data.unpack(fw_yaw_rate);
        rc &= data.unpack(stick_motion_threshold);
        rc &= data.unpack(min_distance);
        rc &= data.unpack(min_distance_procedure);
        return rc;
    }
};

// MSP_SET_FAILSAFE_CONFIG: 76
struct SetFailsafeConfig : public FailsafeSettings, public Message {
    SetFailsafeConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_FAILSAFE_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(delay);
        rc &= data->pack(off_delay);
        rc &= data->pack(throttle);
        rc &= data->pack(kill_switch);
        rc &= data->pack(throttle_low_delay);
        rc &= data->pack(procedure);
        if(extended_contents) {
            rc &= data->pack(recovery_delay);
            rc &= data->pack(fw_roll_angle);
            rc &= data->pack(fw_pitch_angle);
            rc &= data->pack(fw_yaw_rate);
            rc &= data->pack(stick_motion_threshold);
            rc &= data->pack(min_distance);
            rc &= data->pack(min_distance_procedure);
        }
        if(!rc) data.reset();
        return data;
    }
};

struct RxFailChannelSettings {
    Value<uint8_t> mode;
    Value<uint16_t> val;
};

// MSP_RXFAIL_CONFIG: 77
struct RxFailConfigs : public Message {
    RxFailConfigs(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RXFAIL_CONFIG; }

    std::vector<RxFailChannelSettings> channels;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        channels.clear();
        while(rc && data.unpacking_remaining()) {
            RxFailChannelSettings tmp;
            rc &= data.unpack(tmp.mode);
            rc &= data.unpack(tmp.val);
            channels.push_back(tmp);
        }
        return rc;
    }
};

// MSP_SET_RXFAIL_CONFIG: 78
struct SetRxFailConfigs : public RxFailChannelSettings, public Message {
    SetRxFailConfigs(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RXFAIL_CONFIG; }

    Value<uint8_t> channel;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(channel);
        rc &= data.unpack(mode);
        rc &= data.unpack(val);
        return rc;
    }
};

// MSP_SDCARD_SUMMARY: 79
struct SdcardSummary : public Message {
    SdcardSummary(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SDCARD_SUMMARY; }

    Value<uint8_t> flags;
    Value<uint8_t> state;
    Value<uint8_t> last_error;
    Value<uint32_t> free_space_kb;
    Value<uint32_t> total_space_kb;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(flags);
        rc &= data.unpack(state);
        rc &= data.unpack(last_error);
        rc &= data.unpack(free_space_kb);
        rc &= data.unpack(total_space_kb);
        return rc;
    }
};

struct BlackboxConfigSettings {
    Value<uint8_t> device;
    Value<uint8_t> rate_num;
    Value<uint8_t> rate_denom;
    bool p_ratio_set;
    Value<uint16_t> p_ratio;
};

// MSP_BLACKBOX_CONFIG: 80
struct BlackboxConfig : public BlackboxConfigSettings, public Message {
    BlackboxConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BLACKBOX_CONFIG; }

    Value<uint8_t> supported;

    virtual bool decode(const ByteVector& data) override {
        bool rc     = true;
        p_ratio_set = false;
        rc &= data.unpack(supported);
        rc &= data.unpack(device);
        rc &= data.unpack(rate_num);
        rc &= data.unpack(rate_denom);
        if(data.unpacking_remaining()) {
            p_ratio_set = true;
            rc &= data.unpack(p_ratio);
        }
        return rc;
    }
};

// MSP_SET_BLACKBOX_CONFIG: 81
struct SetBlackboxConfig : public BlackboxConfigSettings, public Message {
    SetBlackboxConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_BLACKBOX_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(device);
        rc &= data->pack(rate_num);
        rc &= data->pack(rate_denom);
        if(p_ratio_set) rc &= data->pack(p_ratio);
        if(!rc) data.reset();
        return data;
    }
};

struct TransponderConfigSettings {
    Value<uint8_t> provider;
    Value<uint8_t> data_length;
};

// MSP_TRANSPONDER_CONFIG: 82
struct TransponderConfig : public Message {
    TransponderConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_TRANSPONDER_CONFIG; }

    Value<uint8_t> transponder_count;
    std::vector<TransponderConfigSettings> transponder_data;
    Value<uint8_t> provider;
    ByteVector provider_data;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(transponder_count);
        if(!transponder_count()) return rc;
        for(uint8_t i = 0; i < transponder_count(); ++i) {
            TransponderConfigSettings tmp;
            rc &= data.unpack(tmp.provider);
            rc &= data.unpack(tmp.data_length);
            transponder_data.push_back(tmp);
        }
        rc &= data.unpack(provider);
        if(!provider()) return rc;
        uint8_t data_len = transponder_data[provider() - 1].data_length();
        provider_data    = ByteVector(data.unpacking_iterator(),
                                   data.unpacking_iterator() + data_len);
        rc &= data.consume(data_len);
        return rc;
    }
};

// MSP_SET_TRANSPONDER_CONFIG: 83
struct SetTransponderConfig : public Message {
    SetTransponderConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_TRANSPONDER_CONFIG; }

    Value<uint8_t> provider;
    ByteVector provider_data;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(provider);
        rc &= data->pack(provider_data);
        if(!rc) data.reset();
        return data;
    }
};

// Differences between iNav and BF/CF
// MSP_OSD_CONFIG: 84
struct OsdConfig : public Message {
    OsdConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_OSD_CONFIG; }

    Value<uint8_t> osd_flags;
    Value<uint8_t> video_system;
    Value<uint8_t> units;
    Value<uint8_t> rssi_alarm;
    Value<uint16_t> battery_cap_warn;
    Value<uint16_t> time_alarm;
    Value<uint16_t> alt_alarm;
    Value<uint16_t> dist_alarm;
    Value<uint16_t> neg_alt_alarm;
    std::array<uint16_t, OSD_ITEM_COUNT> item_pos;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(osd_flags);
        if(rc && osd_flags()) {
            rc &= data.unpack(video_system);
            rc &= data.unpack(units);
            rc &= data.unpack(rssi_alarm);
            rc &= data.unpack(battery_cap_warn);
            rc &= data.unpack(time_alarm);
            rc &= data.unpack(alt_alarm);
            rc &= data.unpack(dist_alarm);
            rc &= data.unpack(neg_alt_alarm);
            for(size_t i = 0; i < OSD_ITEM_COUNT; ++i) {
                rc &= data.unpack(item_pos[i]);
            }
        }
        return rc;
    }
};

// MSP_SET_OSD_CONFIG: 85
struct SetOsdConfig : public Message {
    SetOsdConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_OSD_CONFIG; }

    int8_t param_idx;
    Value<uint16_t> item_pos;
    Value<uint8_t> video_system;
    Value<uint8_t> units;
    Value<uint8_t> rssi_alarm;
    Value<uint16_t> battery_cap_warn;
    Value<uint16_t> time_alarm;
    Value<uint16_t> alt_alarm;
    Value<uint16_t> dist_alarm;
    Value<uint16_t> neg_alt_alarm;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(param_idx);
        if(param_idx == -1) {
            rc &= data->pack(video_system);
            rc &= data->pack(units);
            rc &= data->pack(rssi_alarm);
            rc &= data->pack(battery_cap_warn);
            rc &= data->pack(time_alarm);
            rc &= data->pack(alt_alarm);
            rc &= data->pack(dist_alarm);
            rc &= data->pack(neg_alt_alarm);
        }
        else {
            rc &= data->pack(item_pos);
        }
        if(!rc) data.reset();
        return data;
    }
};

// MSP_OSD_CHAR_READ: 86 No reference implementation

// MSP_OSD_CHAR_WRITE: 87
struct OsdCharWrite : public Message {
    OsdCharWrite(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_OSD_CHAR_WRITE; }

    Value<uint8_t> addr;
    std::array<uint8_t, 54> font_data;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(addr);
        for(const auto& c : font_data) {
            rc &= data->pack(c);
        }
        if(!rc) data.reset();
        return data;
    }
};

// MSP_VTX_CONFIG: 88
struct VtxConfig : public Message {
    VtxConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_VTX_CONFIG; }

    Value<uint8_t> device_type;
    Value<uint8_t> band;
    Value<uint8_t> channel;
    Value<uint8_t> power_idx;
    Value<uint8_t> pit_mode;
    bool freq_set;
    Value<uint16_t> frequency;

    virtual bool decode(const ByteVector& data) override {
        bool rc  = true;
        freq_set = false;
        rc &= data.unpack(device_type);
        if(rc && (device_type() != 0xFF)) {
            rc &= data.unpack(band);
            rc &= data.unpack(channel);
            rc &= data.unpack(power_idx);
            rc &= data.unpack(pit_mode);
            if(data.unpacking_remaining()) {
                freq_set = true;
                rc &= data.unpack(frequency);
            }
        }
        return rc;
    }
};

// MSP_SET_VTX_CONFIG: 89
struct SetVtxConfig : public Message {
    SetVtxConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_VTX_CONFIG; }

    Value<uint16_t> frequency;
    Value<uint8_t> power;
    Value<uint8_t> pit_mode;

    bool set_freq(uint8_t band, uint8_t channel) {
        if(band & 0xF8 || channel & 0xF8) {
            return false;
        }
        frequency = uint16_t(band - 1) & uint16_t((channel - 1) << 3);
        return true;
    }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(frequency);
        rc &= data->pack(power);
        rc &= data->pack(pit_mode);
        if(!rc) data.reset();
        return data;
    }
};

// Differs between iNav and BF/CF
struct AdvancedConfigSettings {
    Value<uint8_t> gyro_sync_denom;
    Value<uint8_t> pid_process_denom;
    Value<uint8_t> use_unsynced_pwm;
    Value<uint8_t> motor_pwm_protocol;
    Value<uint16_t> motor_pwm_rate;
    Value<uint16_t> servo_pwm_rate;  // digitalIdleOffsetValue in BF/CF
    Value<uint8_t> gyro_sync;
    bool pwm_inversion_set;
    Value<uint8_t> pwm_inversion;
};

// Betaflight Additional Commands
// MSP_ADVANCED_CONFIG: 90
struct AdvancedConfig : public AdvancedConfigSettings, public Message {
    AdvancedConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ADVANCED_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc           = true;
        pwm_inversion_set = false;
        rc &= data.unpack(gyro_sync_denom);
        rc &= data.unpack(pid_process_denom);
        rc &= data.unpack(use_unsynced_pwm);
        rc &= data.unpack(motor_pwm_protocol);
        rc &= data.unpack(motor_pwm_rate);
        rc &= data.unpack(servo_pwm_rate);
        rc &= data.unpack(gyro_sync);
        if(rc && data.unpacking_remaining()) {
            pwm_inversion_set = true;
            rc &= data.unpack(pwm_inversion);
        }
        return rc;
    }
};

// MSP_SET_ADVANCED_CONFIG: 91
struct SetAdvancedConfig : public AdvancedConfigSettings, public Message {
    SetAdvancedConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_ADVANCED_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(gyro_sync_denom);
        rc &= data->pack(pid_process_denom);
        rc &= data->pack(use_unsynced_pwm);
        rc &= data->pack(motor_pwm_protocol);
        rc &= data->pack(motor_pwm_rate);
        rc &= data->pack(servo_pwm_rate);
        rc &= data->pack(gyro_sync);
        if(pwm_inversion_set) rc &= data->pack(pwm_inversion);
        if(!rc) data.reset();
        return data;
    }
};

struct FilterConfigSettings {
    Value<uint8_t> gyro_soft_lpf_hz;
    Value<uint16_t> dterm_lpf_hz;
    Value<uint16_t> yaw_lpf_hz;
    Value<uint16_t> gyro_soft_notch_hz_1;
    Value<uint16_t> gyro_soft_notch_cutoff_1;
    Value<uint16_t> dterm_soft_notch_hz;
    Value<uint16_t> dterm_soft_notch_cutoff;
    Value<uint16_t> gyro_soft_notch_hz_2;
    Value<uint16_t> gyro_soft_notch_cutoff_2;
    bool dterm_filter_type_set;
    Value<uint8_t> dterm_filter_type;
};

// MSP_FILTER_CONFIG: 92
struct FilterConfig : public FilterConfigSettings, public Message {
    FilterConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_FILTER_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc               = true;
        dterm_filter_type_set = false;
        rc &= data.unpack(gyro_soft_lpf_hz);
        rc &= data.unpack(dterm_lpf_hz);
        rc &= data.unpack(yaw_lpf_hz);
        rc &= data.unpack(gyro_soft_notch_hz_1);
        rc &= data.unpack(gyro_soft_notch_cutoff_1);
        rc &= data.unpack(dterm_soft_notch_hz);
        rc &= data.unpack(dterm_soft_notch_cutoff);
        rc &= data.unpack(gyro_soft_notch_hz_2);
        rc &= data.unpack(gyro_soft_notch_cutoff_2);
        if(rc && data.unpacking_remaining()) {
            dterm_filter_type_set = true;
            rc &= data.unpack(dterm_filter_type);
        }
        return rc;
    }
};

// MSP_SET_FILTER_CONFIG: 93
struct SetFilterConfig : public FilterConfigSettings, public Message {
    SetFilterConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_FILTER_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(gyro_soft_lpf_hz);
        rc &= data->pack(dterm_lpf_hz);
        rc &= data->pack(yaw_lpf_hz);
        rc &= data->pack(gyro_soft_notch_hz_1);
        rc &= data->pack(gyro_soft_notch_cutoff_1);
        rc &= data->pack(dterm_soft_notch_hz);
        rc &= data->pack(dterm_soft_notch_cutoff);
        rc &= data->pack(gyro_soft_notch_hz_2);
        rc &= data->pack(gyro_soft_notch_cutoff_2);
        if(dterm_filter_type_set) rc &= data->pack(dterm_filter_type);
        if(!rc) data.reset();
        return data;
    }
};

struct PidAdvancedSettings {
    Value<uint16_t> rollPitchItermIgnoreRate;
    Value<uint16_t> yawItermIgnoreRate;
    Value<uint16_t> yaw_p_limit;
    Value<uint8_t> deltaMethod;
    Value<uint8_t> vbatPidCompensation;
    Value<uint8_t> setpointRelaxRatio;
    Value<float> dterm_setpoint_weight;  // TODO scaled value
    Value<uint16_t> pidSumLimit;
    Value<uint8_t> itermThrottleGain;
    Value<uint32_t>
        axisAccelerationLimitRollPitch;        // TODO scaled and clamped value
    Value<uint32_t> axisAccelerationLimitYaw;  // TODO scaled and clamped value
};

// Difference between iNav and BF/CF
// MSP_PID_ADVANCED: 94
struct PidAdvanced : public PidAdvancedSettings, public Message {
    PidAdvanced(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_PID_ADVANCED; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(rollPitchItermIgnoreRate);
        rc &= data.unpack(yawItermIgnoreRate);
        rc &= data.unpack(yaw_p_limit);
        rc &= data.unpack(deltaMethod);
        rc &= data.unpack(vbatPidCompensation);
        rc &= data.unpack(setpointRelaxRatio);
        rc &= data.unpack<uint8_t>(dterm_setpoint_weight, 0.01);
        rc &= data.unpack(pidSumLimit);
        rc &= data.unpack(itermThrottleGain);
        Value<uint16_t> tmp16;
        rc &= data.unpack(tmp16);
        axisAccelerationLimitRollPitch = tmp16() * 10;
        rc &= data.unpack(tmp16);
        axisAccelerationLimitYaw = tmp16() * 10;
        return rc;
    }
};

// MSP_SET_PID_ADVANCED: 95
struct SetPidAdvanced : public PidAdvancedSettings, public Message {
    SetPidAdvanced(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_PID_ADVANCED; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(rollPitchItermIgnoreRate);
        rc &= data->pack(yawItermIgnoreRate);
        rc &= data->pack(yaw_p_limit);
        rc &= data->pack(deltaMethod);
        rc &= data->pack(vbatPidCompensation);
        rc &= data->pack(setpointRelaxRatio);
        rc &= data->pack(uint8_t(dterm_setpoint_weight() * 100));
        rc &= data->pack(pidSumLimit);
        rc &= data->pack(itermThrottleGain);
        rc &= data->pack(axisAccelerationLimitRollPitch() / 10);
        rc &= data->pack(axisAccelerationLimitYaw() / 10);
        if(!rc) data.reset();
        return data;
    }
};

struct SensorConfigSettings {
    Value<uint8_t> acc_hardware;
    Value<uint8_t> baro_hardware;
    Value<uint8_t> mag_hardware;
    bool extended_contents;
    Value<uint8_t> pitot_hardware;
    Value<uint8_t> rangefinder_hardware;
    Value<uint8_t> opflow_hardware;
};

// MSP_SENSOR_CONFIG: 96
struct SensorConfig : public SensorConfigSettings, public Message {
    SensorConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SENSOR_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc           = true;
        extended_contents = false;
        rc &= data.unpack(acc_hardware);
        rc &= data.unpack(baro_hardware);
        rc &= data.unpack(mag_hardware);
        if(data.unpacking_remaining()) {
            extended_contents = true;
            rc &= data.unpack(pitot_hardware);
            rc &= data.unpack(rangefinder_hardware);
            rc &= data.unpack(opflow_hardware);
        }
        return rc;
    }
};

// MSP_SET_SENSOR_CONFIG: 97
struct SetSensorConfig : public SensorConfigSettings, public Message {
    SetSensorConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_SENSOR_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(acc_hardware);
        rc &= data->pack(baro_hardware);
        rc &= data->pack(mag_hardware);
        if(!extended_contents) {
            rc &= data->pack(pitot_hardware);
            rc &= data->pack(rangefinder_hardware);
            rc &= data->pack(opflow_hardware);
        }
        if(!rc) data.reset();
        return data;
    }
};

// MSP_CAMERA_CONTROL: 98
struct CameraControl : public Message {
    CameraControl(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_CAMERA_CONTROL; }

    Value<uint8_t> key;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(key)) data.reset();
        return data;
    }
};

// MSP_SET_ARMING_DISABLED: 99
struct SetArmingDisabled : public Message {
    SetArmingDisabled(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_ARMING_DISABLED; }

    Value<uint8_t> command;
    Value<uint8_t> disableRunawayTakeoff;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(command);
        rc &= data->pack(disableRunawayTakeoff);
        if(!rc) data.reset();
        return data;
    }
};

/////////////////////////////////////////////////////////////////////
/// Requests (1xx)

// MSP_IDENT: 100
struct Ident : public Message {
    Ident(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_IDENT; }

    Value<uint8_t> version;
    MultiType type;
    Value<uint8_t> msp_version;
    std::set<Capability> capabilities;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;

        rc &= data.unpack(version);
        rc &= data.unpack((uint8_t&)type);
        rc &= data.unpack(msp_version);
        uint32_t capability;
        rc &= data.unpack(capability);
        if(!rc) return false;
        capabilities.clear();
        if(capability & (1 << 0)) capabilities.insert(Capability::BIND);
        if(capability & (1 << 2)) capabilities.insert(Capability::DYNBAL);
        if(capability & (1 << 3)) capabilities.insert(Capability::FLAP);
        if(capability & (1 << 4)) capabilities.insert(Capability::NAVCAP);
        if(capability & (1 << 5)) capabilities.insert(Capability::EXTAUX);

        return true;
    }

    bool has(const Capability& cap) const { return capabilities.count(cap); }

    bool hasBind() const { return has(Capability::BIND); }

    bool hasDynBal() const { return has(Capability::DYNBAL); }

    bool hasFlap() const { return has(Capability::FLAP); }

    virtual std::ostream& print(std::ostream& s) const override {
        std::string s_type;
        switch(type) {
        case msp::msg::MultiType::TRI:
            s_type = "Tricopter";
            break;
        case msp::msg::MultiType::QUADP:
            s_type = "Quadrocopter Plus";
            break;
        case msp::msg::MultiType::QUADX:
            s_type = "Quadrocopter X";
            break;
        case msp::msg::MultiType::BI:
            s_type = "BI-copter";
            break;
        default:
            s_type = "UNDEFINED";
            break;
        }

        s << "#Ident:" << std::endl;

        s << " MultiWii Version: " << version << std::endl
          << " MSP Version: " << msp_version << std::endl
          << " Type: " << s_type << std::endl
          << " Capabilities:" << std::endl;

        s << "    Bind:   ";
        hasBind() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    DynBal: ";
        hasDynBal() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    Flap:   ";
        hasFlap() ? s << "ON" : s << "OFF";
        s << std::endl;

        return s;
    }
};

struct StatusBase : public Packable {
    Value<uint16_t> cycle_time;  // in us
    Value<uint16_t> i2c_errors;
    std::set<Sensor> sensors;
    std::set<size_t> box_mode_flags;
    Value<uint8_t> current_profile;

    bool unpack_from(const ByteVector& data) {
        bool rc = true;
        rc &= data.unpack(cycle_time);
        rc &= data.unpack(i2c_errors);

        // get sensors
        sensors.clear();
        uint16_t sensor = 0;
        rc &= data.unpack(sensor);
        if(sensor & (1 << 0)) sensors.insert(Sensor::Accelerometer);
        if(sensor & (1 << 1)) sensors.insert(Sensor::Barometer);
        if(sensor & (1 << 2)) sensors.insert(Sensor::Magnetometer);
        if(sensor & (1 << 3)) sensors.insert(Sensor::GPS);
        if(sensor & (1 << 4)) sensors.insert(Sensor::Sonar);
        if(sensor & (1 << 5)) sensors.insert(Sensor::OpticalFlow);
        if(sensor & (1 << 6)) sensors.insert(Sensor::Pitot);
        if(sensor & (1 << 15)) sensors.insert(Sensor::GeneralHealth);

        // check active boxes
        box_mode_flags.clear();
        uint32_t flag = 0;
        rc &= data.unpack(flag);
        for(size_t ibox(0); ibox < sizeof(flag) * CHAR_BIT; ibox++) {
            if(flag & (1 << ibox)) box_mode_flags.insert(ibox);
        }

        rc &= data.unpack(current_profile);
        return rc;
    }

    virtual bool pack_into(ByteVector& data) const {
        bool rc = true;
        rc &= data.pack(cycle_time);
        rc &= data.pack(i2c_errors);

        Value<uint16_t> sensor_pack;
        sensor_pack = 0;
        for(const auto& s : sensors) {
            switch(s) {
            case Sensor::Accelerometer:
                sensor_pack() |= (1 << 0);
                break;
            case Sensor::Barometer:
                sensor_pack() |= (1 << 1);
                break;
            case Sensor::Magnetometer:
                sensor_pack() |= (1 << 2);
                break;
            case Sensor::GPS:
                sensor_pack() |= (1 << 3);
                break;
            case Sensor::Sonar:
                sensor_pack() |= (1 << 4);
                break;
            case Sensor::OpticalFlow:
                sensor_pack() |= (1 << 5);
                break;
            case Sensor::Pitot:
                sensor_pack() |= (1 << 6);
                break;
            case Sensor::GeneralHealth:
                sensor_pack() |= (1 << 15);
                break;
            }
        }
        rc &= data.pack(sensor_pack);

        Value<uint32_t> box_pack;
        box_pack = 0;
        for(const auto& b : box_mode_flags) {
            box_pack() |= (1 << b);
        }
        rc &= data.pack(box_pack);
        return rc;
    }
};

// MSP_STATUS: 101
struct Status : public StatusBase, public Message {
    Status(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_STATUS; }

    Value<uint16_t> avg_system_load_pct;
    Value<uint16_t> gyro_cycle_time;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= StatusBase::unpack_from(data);

        if(fw_variant != FirmwareVariant::INAV) {
            rc &= data.unpack(avg_system_load_pct);
            rc &= data.unpack(gyro_cycle_time);
        }
        return rc;
    }

    bool hasAccelerometer() const {
        return sensors.count(Sensor::Accelerometer);
    }

    bool hasBarometer() const { return sensors.count(Sensor::Barometer); }

    bool hasMagnetometer() const { return sensors.count(Sensor::Magnetometer); }

    bool hasGPS() const { return sensors.count(Sensor::GPS); }

    bool hasSonar() const { return sensors.count(Sensor::Sonar); }

    bool hasOpticalFlow() const { return sensors.count(Sensor::OpticalFlow); }

    bool hasPitot() const { return sensors.count(Sensor::Pitot); }

    bool isHealthy() const { return sensors.count(Sensor::GeneralHealth); }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Status:" << std::endl;
        s << " Cycle time: " << cycle_time << " us" << std::endl;
        s << " I2C errors: " << i2c_errors << std::endl;
        s << " Sensors:" << std::endl;

        s << "    Accelerometer: ";
        hasAccelerometer() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    Barometer: ";
        hasBarometer() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    Magnetometer: ";
        hasMagnetometer() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    GPS: ";
        hasGPS() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    Sonar: ";
        hasSonar() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << " Active Boxes (by ID):";
        for(const size_t box_id : box_mode_flags) {
            s << " " << box_id;
        }
        s << std::endl;

        return s;
    }
};

// MSP_RAW_IMU: 102
struct RawImu : public Message {
    RawImu(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RAW_IMU; }

    std::array<Value<int16_t>, 3> acc;
    std::array<Value<int16_t>, 3> gyro;
    std::array<Value<int16_t>, 3> mag;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(auto& a : acc) {
            rc &= data.unpack(a);
        }
        for(auto& g : gyro) {
            rc &= data.unpack(g);
        }
        for(auto& m : mag) {
            rc &= data.unpack(m);
        }
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Imu:" << std::endl;
        s << " Linear acceleration: " << acc[0] << ", " << acc[1] << ", "
          << acc[2] << std::endl;
        s << " Angular velocity: " << gyro[0] << ", " << gyro[1] << ", "
          << gyro[2] << std::endl;
        s << " Magnetometer: " << mag[0] << ", " << mag[1] << ", " << mag[2]
          << std::endl;
        return s;
    }
};

// helper class to convert raw imu readigs into standard physic units
// custom scaling factors have to be derived from the sensor documentation
struct ImuSI {
    std::array<Value<float>, 3> acc;   // m/s^2
    std::array<Value<float>, 3> gyro;  // deg/s
    std::array<Value<float>, 3> mag;   // uT

    ImuSI(RawImu raw,
          const float acc_1g,     // sensor value at 1g
          const float gyro_unit,  // resolution in 1/(deg/s)
          const float magn_gain,  // scale magnetic value to uT (micro Tesla)
          const float si_unit_1g  // acceleration at 1g (in m/s^2))
    ) {
        for(int i = 0; i < 3; ++i) {
            acc[i] = raw.acc[i]() / acc_1g * si_unit_1g;
        }
        for(int i = 0; i < 3; ++i) {
            gyro[i] = raw.gyro[i]() * gyro_unit;
        }
        for(int i = 0; i < 3; ++i) {
            mag[i] = raw.mag[i]() * magn_gain;
        }
    }

    std::ostream& print(std::ostream& s) const {
        s << "#Imu:" << std::endl;
        s << " Linear acceleration: " << acc[0] << ", " << acc[1] << ", "
          << acc[2] << " m/s²" << std::endl;
        s << " Angular velocity: " << gyro[0] << ", " << gyro[1] << ", "
          << gyro[2] << " deg/s" << std::endl;
        s << " Magnetometer: " << mag[0] << ", " << mag[1] << ", " << mag[2]
          << " uT" << std::endl;
        return s;
    }
};

// MSP_SERVO: 103
struct Servo : public Message {
    Servo(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SERVO; }

    std::array<uint16_t, N_SERVO> servo;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(auto& s : servo) rc &= data.unpack(s);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Servo:" << std::endl;
        s << " " << servo[0] << " " << servo[1] << " " << servo[2] << " "
          << servo[3] << std::endl;
        s << " " << servo[4] << " " << servo[5] << " " << servo[6] << " "
          << servo[7] << std::endl;
        return s;
    }
};

// MSP_MOTOR: 104
struct Motor : public Message {
    Motor(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_MOTOR; }

    std::array<uint16_t, N_MOTOR> motor;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(auto& m : motor) rc &= data.unpack(m);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Motor:" << std::endl;
        s << " " << motor[0] << " " << motor[1] << " " << motor[2] << " "
          << motor[3] << std::endl;
        s << " " << motor[4] << " " << motor[5] << " " << motor[6] << " "
          << motor[7] << std::endl;
        return s;
    }
};

// MSP_RC: 105
struct Rc : public Message {
    Rc(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RC; }

    std::vector<uint16_t> channels;

    virtual bool decode(const ByteVector& data) override {
        channels.clear();
        bool rc = true;
        while(rc) {
            uint16_t rc_data;
            rc &= data.unpack(rc_data);
            if(rc) channels.push_back(rc_data);
        }
        return !channels.empty();
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Rc channels (" << channels.size() << ") :" << std::endl;
        for(const uint16_t c : channels) {
            s << c << " ";
        }
        s << " " << std::endl;
        return s;
    }
};

// MSP_RAW_GPS: 106
struct RawGPS : public Message {
    RawGPS(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RAW_GPS; }

    Value<uint8_t> fix;
    Value<uint8_t> numSat;
    Value<float> lat;
    Value<float> lon;
    Value<float> altitude;
    Value<float> ground_speed;
    Value<float> ground_course;
    bool hdop_set;
    Value<float> hdop;

    virtual bool decode(const ByteVector& data) override {
        bool rc  = true;
        hdop_set = false;
        rc &= data.unpack(fix);
        rc &= data.unpack(numSat);
        rc &= data.unpack<int32_t>(lat, 1.f / 1e-7);
        rc &= data.unpack<int32_t>(lon, 1.f / 1e-7);
        rc &= data.unpack<int16_t>(altitude, 1.f);
        rc &= data.unpack<uint16_t>(ground_speed, 100.f);
        rc &= data.unpack<uint16_t>(ground_course, 10.f);
        if(data.unpacking_remaining()) {
            hdop_set = true;
            rc &= data.unpack<uint16_t>(hdop, 100.f);
        }
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#RawGPS:" << std::endl;
        s << " Fix: " << fix << std::endl;
        s << " Num Sat: " << numSat << std::endl;
        s << " Location: " << lat << " " << lon << std::endl;
        s << " Altitude: " << altitude << " m" << std::endl;
        s << " Ground speed: " << ground_speed << " m/s" << std::endl;
        s << " Ground course: " << ground_course << " deg" << std::endl;
        if(hdop_set) s << " HDOP: " << hdop << std::endl;
        return s;
    }
};

// MSP_COMP_GPS: 107
struct CompGPS : public Message {
    CompGPS(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_COMP_GPS; }

    Value<uint16_t> distanceToHome;   // meter
    Value<uint16_t> directionToHome;  // degree
    Value<uint8_t> update;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(distanceToHome);
        rc &= data.unpack(directionToHome);
        rc &= data.unpack(update);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#CompGPS:" << std::endl;
        s << " Distance: " << distanceToHome << std::endl;
        s << " Direction: " << directionToHome << std::endl;
        s << " Update: " << update << std::endl;

        return s;
    }
};

// TODO validate units
// MSP_ATTITUDE: 108
struct Attitude : public Message {
    Attitude(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ATTITUDE; }

    Value<int16_t> roll;   // degree
    Value<int16_t> pitch;  // degree
    Value<int16_t> yaw;    // degree

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(roll);
        rc &= data.unpack(pitch);
        rc &= data.unpack(yaw);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Attitude:" << std::endl;
        s << " Roll : " << roll << " deg" << std::endl;
        s << " Pitch : " << pitch << " deg" << std::endl;
        s << " Heading: " << yaw << " deg" << std::endl;
        return s;
    }
};

// TODO validate units
// MSP_ALTITUDE: 109
struct Altitude : public Message {
    Altitude(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ALTITUDE; }

    Value<float> altitude;  // m
    Value<float> vario;     // m/s
    bool baro_altitude_set;
    Value<float> baro_altitude;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack<int32_t>(altitude, 100.f);
        rc &= data.unpack<int16_t>(vario, 100.f);
        if(data.unpacking_remaining()) {
            baro_altitude_set = true;
            rc &= data.unpack<int32_t>(baro_altitude, 100.f);
        }
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Altitude:" << std::endl;
        s << " Altitude: " << altitude << " m, var: " << vario << " m/s"
          << std::endl;
        s << " Barometer: ";
        if(baro_altitude_set)
            s << baro_altitude;
        else
            s << "<unset>";
        s << std::endl;
        return s;
    }
};

// TODO check amperage units
// MSP_ANALOG: 110
struct Analog : public Message {
    Analog(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ANALOG; }

    Value<float> vbat;           // Volt
    Value<float> powerMeterSum;  // Ah
    Value<uint16_t> rssi;   // Received Signal Strength Indication [0; 1023]
    Value<float> amperage;  // Ampere

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack<uint8_t>(vbat, 0.1);
        rc &= data.unpack<uint16_t>(powerMeterSum, 0.001);
        rc &= data.unpack(rssi);
        rc &= data.unpack<int8_t>(amperage, 0.1);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Analog:" << std::endl;
        s << " Battery Voltage: " << vbat << " V" << std::endl;
        s << " Current: " << amperage << " A" << std::endl;
        s << " Power consumption: " << powerMeterSum << " Ah" << std::endl;
        s << " RSSI: " << rssi << std::endl;
        return s;
    }
};

struct RcTuningSettings {
    // RPY sequence
    std::array<Value<uint8_t>, 3> rates;
    std::array<Value<uint8_t>, 3> rcRates;
    std::array<Value<uint8_t>, 3> rcExpo;

    Value<uint8_t> dynamic_throttle_pid;
    Value<uint8_t> throttle_mid;
    Value<uint8_t> throttle_expo;
    Value<uint16_t> tpa_breakpoint;

    std::ostream& print(std::ostream& s) const {
        s << "#Rc Tuning:" << std::endl;
        s << " Rc Rate: " << rcRates[0] << " " << rcRates[1] << " "
          << rcRates[2] << std::endl;
        s << " Rc Expo: " << rcExpo[0] << " " << rcExpo[1] << " " << rcExpo[2]
          << std::endl;

        s << " Dynamic Throttle PID: " << dynamic_throttle_pid << std::endl;
        s << " Throttle MID: " << throttle_mid << std::endl;
        s << " Throttle Expo: " << throttle_expo << std::endl;
        return s;
    }
};

// Differences between iNav and BF/CF
// MSP_RC_TUNING: 111
struct RcTuning : public RcTuningSettings, public Message {
    RcTuning(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RC_TUNING; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(rcRates[0]);
        rc &= data.unpack(rcExpo[0]);
        for(size_t i = 0; i < 3; ++i) {
            rc &= data.unpack(rates[0]);
        }
        rc &= data.unpack(dynamic_throttle_pid);
        rc &= data.unpack(throttle_mid);
        rc &= data.unpack(throttle_expo);
        rc &= data.unpack(tpa_breakpoint);
        rc &= data.unpack(rcExpo[2]);
        if(fw_variant == FirmwareVariant::INAV) return rc;
        rc &= data.unpack(rcRates[2]);
        rc &= data.unpack(rcRates[1]);
        rc &= data.unpack(rcExpo[1]);
        return rc;
    }
};

// PID struct for messages 112 and 202
struct PidTerms : public Packable {
    uint8_t P;
    uint8_t I;
    uint8_t D;

    bool unpack_from(const ByteVector& data) {
        bool rc = true;
        rc &= data.unpack(P);
        rc &= data.unpack(I);
        rc &= data.unpack(D);
        return rc;
    }

    bool pack_into(ByteVector& data) const {
        bool rc = true;
        rc &= data.pack(P);
        rc &= data.pack(I);
        rc &= data.pack(D);
        return rc;
    }
};

struct PidSettings {
    std::array<Value<PidTerms>,
               static_cast<uint8_t>(PID_Element::PID_ITEM_COUNT)>
        entry;

    std::ostream& print(std::ostream& s) const {
        uint8_t PID_ROLL =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_ROLL);
        uint8_t PID_PITCH =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_PITCH);
        uint8_t PID_YAW = static_cast<uint8_t>(msp::msg::PID_Element::PID_YAW);
        uint8_t PID_POS_Z =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_POS_Z);
        uint8_t PID_POS_XY =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_POS_XY);
        uint8_t PID_VEL_XY =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_VEL_XY);
        uint8_t PID_SURFACE =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_SURFACE);
        uint8_t PID_LEVEL =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_LEVEL);
        uint8_t PID_HEADING =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_HEADING);
        uint8_t PID_VEL_Z =
            static_cast<uint8_t>(msp::msg::PID_Element::PID_VEL_Z);

        s << std::setprecision(3);
        s << "#PID:" << std::endl;
        s << " Name      P     | I     | D     |" << std::endl;
        s << " ----------------|-------|-------|" << std::endl;
        s << " Roll:      " << entry[PID_ROLL]().P << "\t| "
          << entry[PID_ROLL]().I << "\t| " << entry[PID_ROLL]().D << std::endl;
        s << " Pitch:     " << entry[PID_PITCH]().P << "\t| "
          << entry[PID_PITCH]().I << "\t| " << entry[PID_PITCH]().D
          << std::endl;
        s << " Yaw:       " << entry[PID_YAW]().P << "\t| "
          << entry[PID_YAW]().I << "\t| " << entry[PID_YAW]().D << std::endl;
        s << " Altitude:  " << entry[PID_POS_Z]().P << "\t| "
          << entry[PID_POS_Z]().I << "\t| " << entry[PID_POS_Z]().D
          << std::endl;

        s << " Position:  " << entry[PID_POS_XY]().P << "\t| "
          << entry[PID_POS_XY]().I << "\t| " << entry[PID_POS_XY]().D
          << std::endl;
        s << " PositionR: " << entry[PID_VEL_XY]().P << "\t| "
          << entry[PID_VEL_XY]().I << "\t| " << entry[PID_VEL_XY]().D
          << std::endl;
        s << " NavR:      " << entry[PID_SURFACE]().P << "\t| "
          << entry[PID_SURFACE]().I << "\t| " << entry[PID_SURFACE]().D
          << std::endl;
        s << " Level:     " << entry[PID_LEVEL]().P << "\t| "
          << entry[PID_LEVEL]().I << "\t| " << entry[PID_LEVEL]().D
          << std::endl;
        s << " Magn:      " << entry[PID_HEADING]().P << "\t| "
          << entry[PID_HEADING]().I << "\t| " << entry[PID_HEADING]().D
          << std::endl;
        s << " Vel:       " << entry[PID_VEL_Z]().P << "\t| "
          << entry[PID_VEL_Z]().I << "\t| " << entry[PID_VEL_Z]().D
          << std::endl;

        return s;
    }
};

// TODO: revisit
// MSP_PID: 112
struct Pid : public PidSettings, public Message {
    Pid(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_PID; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(uint8_t i = 0;
            i < static_cast<uint8_t>(PID_Element::PID_ITEM_COUNT);
            ++i) {
            rc &= data.unpack(entry[i]);
        }
        return rc;
    }
};

// MSP_ACTIVEBOXES: 113
struct ActiveBoxes : public Message {
    ActiveBoxes(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ACTIVEBOXES; }

    // box activation pattern
    std::vector<std::array<std::set<SwitchPosition>, NAUX>> box_pattern;

    virtual bool decode(const ByteVector& data) override {
        box_pattern.clear();
        bool rc = true;
        while(rc && data.unpacking_remaining() > 1) {
            Value<uint16_t> box_conf;
            rc &= data.unpack(box_conf);
            std::array<std::set<SwitchPosition>, NAUX> aux_sp;
            for(size_t iaux(0); iaux < NAUX; iaux++) {
                for(size_t ip(0); ip < 3; ip++) {
                    if(box_conf() & (1 << (iaux * 3 + ip)))
                        aux_sp[iaux].insert(SwitchPosition(ip));
                }  // each position (L,M,H)
            }      // each aux switch
            box_pattern.push_back(aux_sp);
        }  // each box
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Box:" << std::endl;
        for(size_t ibox(0); ibox < box_pattern.size(); ibox++) {
            s << ibox << " ";
            for(size_t iaux(0); iaux < box_pattern[ibox].size(); iaux++) {
                s << "aux" << iaux + 1 << ": ";
                if(box_pattern[ibox][iaux].count(msp::msg::SwitchPosition::LOW))
                    s << "L";
                else
                    s << "_";
                if(box_pattern[ibox][iaux].count(msp::msg::SwitchPosition::MID))
                    s << "M";
                else
                    s << "_";
                if(box_pattern[ibox][iaux].count(
                       msp::msg::SwitchPosition::HIGH))
                    s << "H";
                else
                    s << "_";
                s << ", ";
            }
            s << std::endl;
        }

        return s;
    }
};

struct MiscSettings {
    Value<uint16_t> mid_rc;
    Value<uint16_t> min_throttle;
    Value<uint16_t> max_throttle;
    Value<uint16_t> min_command;
    Value<uint16_t> failsafe_throttle;
    Value<uint8_t> gps_provider;
    Value<uint8_t> gps_baudrate;
    Value<uint8_t> gps_ubx_sbas;
    Value<uint8_t> multiwii_current_meter_output;
    Value<uint8_t> rssi_channel;
    Value<uint8_t> reserved;
    Value<float> mag_declination;  // degree
    Value<float> voltage_scale, cell_min, cell_max, cell_warning;

    std::ostream& print(std::ostream& s) const {
        s << "#Miscellaneous:" << std::endl;
        s << " Mid rc: " << mid_rc << std::endl;
        s << " Min Throttle: " << min_throttle << std::endl;
        s << " Max Throttle: " << max_throttle << std::endl;
        s << " Failsafe Throttle: " << failsafe_throttle << std::endl;

        s << " Magnetic Declination: " << mag_declination << " deg"
          << std::endl;
        s << " Battery Voltage Scale: " << voltage_scale << " V" << std::endl;
        s << " Battery Warning Level 1: " << cell_min << " V" << std::endl;
        s << " Battery Warning Level 2: " << cell_max << " V" << std::endl;
        s << " Battery Critical Level: " << cell_warning << " V" << std::endl;

        return s;
    }
};

// MSP_MISC: 114
struct Misc : public MiscSettings, public Message {
    Misc(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_MISC; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(mid_rc);
        rc &= data.unpack(min_throttle);
        rc &= data.unpack(max_throttle);
        rc &= data.unpack(min_command);
        rc &= data.unpack(failsafe_throttle);
        rc &= data.unpack(gps_provider);
        rc &= data.unpack(gps_baudrate);
        rc &= data.unpack(gps_ubx_sbas);
        rc &= data.unpack(multiwii_current_meter_output);
        rc &= data.unpack(rssi_channel);
        rc &= data.unpack(reserved);

        rc &= data.unpack<uint16_t>(mag_declination, 0.1);
        rc &= data.unpack<uint8_t>(voltage_scale, 0.1);
        rc &= data.unpack<uint8_t>(cell_min, 0.1);
        rc &= data.unpack<uint8_t>(cell_max, 0.1);
        rc &= data.unpack<uint8_t>(cell_warning, 0.1);
        return rc;
    }
};

// MSP_MOTOR_PINS: 115
struct MotorPins : public Message {
    MotorPins(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_MOTOR_PINS; }

    Value<uint8_t> pwm_pin[N_MOTOR];

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(auto& pin : pwm_pin) rc &= data.unpack(pin);
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Motor pins:" << std::endl;
        for(size_t imotor(0); imotor < msp::msg::N_MOTOR; imotor++) {
            s << " Motor " << imotor << ": pin " << size_t(pwm_pin[imotor]())
              << std::endl;
        }

        return s;
    }
};

// MSP_BOXNAMES: 116
struct BoxNames : public Message {
    BoxNames(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BOXNAMES; }

    std::vector<std::string> box_names;

    virtual bool decode(const ByteVector& data) override {
        box_names.clear();
        bool rc = true;
        std::string str;
        rc &= data.unpack(str);
        std::stringstream ss(str);
        std::string bname;
        while(getline(ss, bname, ';')) {
            box_names.push_back(bname);
        }
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "# Box names:" << std::endl;
        for(size_t ibox(0); ibox < box_names.size(); ibox++) {
            s << " " << ibox << ": " << box_names[ibox] << std::endl;
        }
        return s;
    }
};

// MSP_PIDNAMES: 117
struct PidNames : public Message {
    PidNames(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_PIDNAMES; }

    std::vector<std::string> pid_names;

    virtual bool decode(const ByteVector& data) override {
        pid_names.clear();
        bool rc = true;
        std::string str;
        rc &= data.unpack(str);
        std::stringstream ss(str);
        std::string pname;
        while(getline(ss, pname, ';')) {
            pid_names.push_back(pname);
        }
        return rc;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#PID names:" << std::endl;
        for(size_t ipid(0); ipid < pid_names.size(); ipid++) {
            s << " " << ipid << ": " << pid_names[ipid] << std::endl;
        }
        return s;
    }
};

// MSP_WP: 118
struct WayPoint : public Message {
    WayPoint(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_WP; }

    Value<uint8_t> wp_no;
    Value<uint32_t> lat;
    Value<uint32_t> lon;
    Value<uint32_t> altHold;
    Value<uint16_t> heading;
    Value<uint16_t> staytime;
    Value<uint8_t> navflag;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(wp_no);
        rc &= data.unpack(lat);
        rc &= data.unpack(lon);
        rc &= data.unpack(altHold);
        rc &= data.unpack(heading);
        rc &= data.unpack(staytime);
        rc &= data.unpack(navflag);
        return rc;
    }
};

// MSP_BOXIDS: 119
struct BoxIds : public Message {
    BoxIds(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BOXIDS; }

    ByteVector box_ids;

    virtual bool decode(const ByteVector& data) override {
        box_ids.clear();

        for(uint8_t bi : data) box_ids.push_back(bi);
        ;

        return true;
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Box IDs:" << std::endl;
        for(size_t ibox(0); ibox < box_ids.size(); ibox++) {
            s << " " << ibox << ": " << size_t(box_ids[ibox]) << std::endl;
        }
        return s;
    }
};

struct ServoConfRange {
    Value<uint16_t> min;
    Value<uint16_t> max;
    Value<uint16_t> middle;
    Value<uint8_t> rate;
};

// MSP_SERVO_CONF: 120
struct ServoConf : public Message {
    ServoConf(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SERVO_CONF; }

    ServoConfRange servo_conf[N_SERVO];

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(size_t i(0); i < N_SERVO; i++) {
            rc &= data.unpack(servo_conf[i].min);
            rc &= data.unpack(servo_conf[i].max);
            rc &= data.unpack(servo_conf[i].middle);
            rc &= data.unpack(servo_conf[i].rate);
        }
        return rc;
    }
};

// MSP_NAV_STATUS: 121
struct NavStatus : public Message {
    NavStatus(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_NAV_STATUS; }

    Value<uint8_t> GPS_mode;
    Value<uint8_t> NAV_state;
    Value<uint8_t> mission_action;
    Value<uint8_t> mission_number;
    Value<uint8_t> NAV_error;
    int16_t target_bearing;  // degrees

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(GPS_mode);
        rc &= data.unpack(NAV_state);
        rc &= data.unpack(mission_action);
        rc &= data.unpack(mission_number);
        rc &= data.unpack(NAV_error);
        rc &= data.unpack(target_bearing);
        return rc;
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

    uint16_t wp_radius;         // in cm
    uint16_t safe_wp_distance;  // in meter
    uint16_t nav_max_altitude;  // in meter
    uint16_t nav_speed_max;     // in cm/s
    uint16_t nav_speed_min;     // in cm/s

    uint8_t crosstrack_gain;  // * 100 (0-2.56)
    uint16_t nav_bank_max;    // degree * 100; (3000 default)
    uint16_t rth_altitude;    // in meter
    uint8_t land_speed;       // between 50 and 255 (100 approx = 50cm/sec)
    uint16_t fence;           // fence control in meters

    uint8_t max_wp_number;

    uint8_t checksum;
};

// MSP_NAV_CONFIG: 122
struct NavConfig : public Message, public GpsConf {
    NavConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_NAV_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(filtering);
        rc &= data.unpack(lead_filter);
        rc &= data.unpack(dont_reset_home_at_arm);
        rc &= data.unpack(nav_controls_heading);

        rc &= data.unpack(nav_tail_first);
        rc &= data.unpack(nav_rth_takeoff_heading);
        rc &= data.unpack(slow_nav);
        rc &= data.unpack(wait_for_rth_alt);

        rc &= data.unpack(ignore_throttle);
        rc &= data.unpack(takeover_baro);

        rc &= data.unpack(wp_radius);
        rc &= data.unpack(safe_wp_distance);
        rc &= data.unpack(nav_max_altitude);
        rc &= data.unpack(nav_speed_max);
        rc &= data.unpack(nav_speed_min);

        rc &= data.unpack(crosstrack_gain);
        rc &= data.unpack(nav_bank_max);
        rc &= data.unpack(rth_altitude);
        rc &= data.unpack(land_speed);
        rc &= data.unpack(fence);

        rc &= data.unpack(max_wp_number);
        rc &= data.unpack(checksum);

        return rc;
    }
};

struct Motor3dConfig : public Message {
    Motor3dConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_MOTOR_3D_CONFIG; }

    Value<uint16_t> deadband3d_low;
    Value<uint16_t> deadband3d_high;
    Value<uint16_t> neutral_3d;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(deadband3d_low);
        rc &= data.unpack(deadband3d_high);
        rc &= data.unpack(neutral_3d);
        return rc;
    }
};

struct RcDeadbandSettings {
    Value<uint8_t> deadband;
    Value<uint8_t> yaw_deadband;
    Value<uint8_t> alt_hold_deadband;
    Value<uint16_t> deadband3d_throttle;
};

struct RcDeadband : public RcDeadbandSettings, public Message {
    RcDeadband(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RC_DEADBAND; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(deadband);
        rc &= data.unpack(yaw_deadband);
        rc &= data.unpack(alt_hold_deadband);
        rc &= data.unpack(deadband3d_throttle);
        return rc;
    }
};

struct SensorAlignmentSettings {
    Value<uint8_t> gyro_align;
    Value<uint8_t> acc_align;
    Value<uint8_t> mag_align;
};

struct SensorAlignment : public SensorAlignmentSettings, public Message {
    SensorAlignment(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SENSOR_ALIGNMENT; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(gyro_align);
        rc &= data.unpack(acc_align);
        rc &= data.unpack(mag_align);
        return rc;
    }
};

struct LedStripModecolor : public Message {
    LedStripModecolor(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_LED_STRIP_MODECOLOR; }

    std::array<std::array<uint8_t, LED_DIRECTION_COUNT>, LED_MODE_COUNT>
        mode_colors;
    std::array<uint8_t, LED_SPECIAL_COLOR_COUNT> special_colors;
    Value<uint8_t> led_aux_channel;
    Value<uint8_t> reserved;
    Value<uint8_t> led_strip_aux_channel;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(auto& mode : mode_colors) {
            for(auto& color : mode) {
                rc &= data.unpack(color);
            }
        }
        for(auto& special : special_colors) {
            rc &= data.unpack(special);
        }

        rc &= data.unpack(led_aux_channel);
        rc &= data.unpack(reserved);
        rc &= data.unpack(led_strip_aux_channel);
        return rc;
    }
};

struct VoltageMeter {
    Value<uint8_t> id;
    Value<uint8_t> val;
};

struct VoltageMeters : public Message {
    VoltageMeters(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_VOLTAGE_METERS; }

    std::vector<VoltageMeter> meters;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(auto& meter : meters) {
            rc &= data.unpack(meter.id);
            rc &= data.unpack(meter.val);
        }
        return rc;
    }
};

struct CurrentMeter {
    Value<uint8_t> id;
    Value<uint16_t> mAh_drawn;
    Value<uint16_t> mA;
};

struct CurrentMeters : public Message {
    CurrentMeters(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_CURRENT_METERS; }

    std::vector<CurrentMeter> meters;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        for(auto& meter : meters) {
            rc &= data.unpack(meter.id);
            rc &= data.unpack(meter.mAh_drawn);
            rc &= data.unpack(meter.mA);
        }
        return rc;
    }
};

struct BatteryState : public Message {
    BatteryState(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BATTERY_STATE; }

    Value<uint8_t> cell_count;
    Value<uint16_t> capacity_mAh;
    Value<uint8_t> voltage;
    Value<uint16_t> mAh_drawn;
    Value<uint16_t> current;
    Value<uint8_t> state;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(cell_count);
        rc &= data.unpack(capacity_mAh);
        rc &= data.unpack(voltage);
        rc &= data.unpack(mAh_drawn);
        rc &= data.unpack(current);
        rc &= data.unpack(state);
        return rc;
    }
};

struct MotorConfigSettings {
    Value<uint16_t> min_throttle;
    Value<uint16_t> max_throttle;
    Value<uint16_t> min_command;
};

struct MotorConfig : public MotorConfigSettings, public Message {
    MotorConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_MOTOR_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(min_throttle);
        rc &= data.unpack(max_throttle);
        rc &= data.unpack(min_command);
        return rc;
    }
};

struct GpsConfigSettings {
    Value<uint8_t> provider;
    Value<uint8_t> sbas_mode;
    Value<uint8_t> auto_config;
    Value<uint8_t> auto_baud;
};

struct GpsConfig : public GpsConfigSettings, public Message {
    GpsConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_GPS_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(provider);
        rc &= data.unpack(sbas_mode);
        rc &= data.unpack(auto_config);
        rc &= data.unpack(auto_baud);
        return rc;
    }
};

struct CompassConfig : public Message {
    CompassConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_COMPASS_CONFIG; }

    Value<uint16_t> mag_declination;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(mag_declination);
    }
};

struct EscData {
    Value<uint8_t> temperature;
    Value<uint16_t> rpm;
};

struct EscSensorData : public Message {
    EscSensorData(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ESC_SENSOR_DATA; }

    Value<uint8_t> motor_count;
    std::vector<EscData> esc_data;

    virtual bool decode(const ByteVector& data) override {
        if(data.empty()) {
            motor_count = 0;
            return true;
        }
        bool rc = true;
        rc &= data.unpack(motor_count);
        for(int i = 0; i < motor_count(); ++i) {
            EscData esc;
            rc &= data.unpack(esc.temperature);
            rc &= data.unpack(esc.rpm);
            esc_data.push_back(esc);
        }
        return rc;
    }
};

struct StatusEx : public StatusBase, public Message {
    StatusEx(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_STATUS_EX; }

    // bf/cf fields
    Value<uint8_t> max_profiles;
    Value<uint8_t> control_rate_profile;
    // iNav fields
    Value<uint16_t> avg_system_load_pct;
    Value<uint16_t> arming_flags;
    Value<uint8_t> acc_calibration_axis_flags;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= StatusBase::unpack_from(data);
        if(fw_variant == FirmwareVariant::INAV) {
            rc &= data.unpack(avg_system_load_pct);
            rc &= data.unpack(arming_flags);
            rc &= data.unpack(acc_calibration_axis_flags);
        }
        else {
            rc &= data.unpack(max_profiles);
            rc &= data.unpack(control_rate_profile);
        }
        return rc;
    }
};

struct SensorStatus : public Message {
    SensorStatus(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SENSOR_STATUS; }

    Value<uint8_t> hardware_healthy;
    Value<uint8_t> hw_gyro_status;
    Value<uint8_t> hw_acc_status;
    Value<uint8_t> hw_compass_status;
    Value<uint8_t> hw_baro_status;
    Value<uint8_t> hw_gps_status;
    Value<uint8_t> hw_rangefinder_status;
    Value<uint8_t> hw_pitometer_status;
    Value<uint8_t> hw_optical_flow_status;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(hardware_healthy);
        rc &= data.unpack(hw_gyro_status);
        rc &= data.unpack(hw_acc_status);
        rc &= data.unpack(hw_compass_status);
        rc &= data.unpack(hw_baro_status);
        rc &= data.unpack(hw_gps_status);
        rc &= data.unpack(hw_rangefinder_status);
        rc &= data.unpack(hw_pitometer_status);
        rc &= data.unpack(hw_optical_flow_status);
        return rc;
    }
};

// MSP_UID                         = 160,
struct Uid : public Message {
    Uid(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_UID; }

    Value<uint32_t> u_id_0;
    Value<uint32_t> u_id_1;
    Value<uint32_t> u_id_2;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(u_id_0);
        rc &= data.unpack(u_id_1);
        rc &= data.unpack(u_id_2);
        return rc;
    }
};

struct GpsSvInfoSettings {
    uint8_t channel;
    uint8_t sv_id;
    uint8_t quality;
    uint8_t cno;
};

// MSP_GPSSVINFO                   = 164,
struct GpsSvInfo : public Message {
    GpsSvInfo(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_GPSSVINFO; }

    Value<uint8_t> hdop;

    Value<uint8_t> channel_count;
    std::vector<GpsSvInfoSettings> sv_info;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        if(fw_variant == FirmwareVariant::INAV) {
            rc &= data.consume(4);
            rc &= data.unpack(hdop);
        }
        else {
            rc &= data.unpack(channel_count);
            for(uint8_t i = 0; i < channel_count(); ++i) {
                GpsSvInfoSettings tmp;
                rc &= data.unpack(tmp.channel);
                rc &= data.unpack(tmp.sv_id);
                rc &= data.unpack(tmp.quality);
                rc &= data.unpack(tmp.cno);
            }
        }
        return rc;
    }
};

// MSP_GPSSTATISTICS               = 166,
struct GpsStatistics : public Message {
    GpsStatistics(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_GPSSTATISTICS; }

    Value<uint16_t> last_msg_dt;
    Value<uint32_t> errors;
    Value<uint32_t> timeouts;
    Value<uint32_t> packet_count;
    Value<uint16_t> hdop;
    Value<uint16_t> eph;
    Value<uint16_t> epv;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(last_msg_dt);
        rc &= data.unpack(errors);
        rc &= data.unpack(timeouts);
        rc &= data.unpack(packet_count);
        rc &= data.unpack(hdop);
        rc &= data.unpack(eph);
        rc &= data.unpack(epv);
        return rc;
    }
};

// no actual implementations
// MSP_OSD_VIDEO_CONFIG            = 180,
struct OsdVideoConfig : public Message {
    OsdVideoConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_OSD_VIDEO_CONFIG; }

    virtual bool decode(const ByteVector& /*data*/) override { return false; }
};

// MSP_SET_OSD_VIDEO_CONFIG        = 181,
struct SetOsdVideoConfig : public Message {
    SetOsdVideoConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_OSD_VIDEO_CONFIG; }
};

// MSP_DISPLAYPORT                 = 182,
struct Displayport : public Message {
    Displayport(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_DISPLAYPORT; }

    Value<uint8_t> sub_cmd;
    Value<uint8_t> row;
    Value<uint8_t> col;
    Value<std::string> str;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(sub_cmd);
        if(sub_cmd() == 3) {
            rc &= data->pack(row);
            rc &= data->pack(col);
            rc &= data->pack(uint8_t(0));
            rc &= data->pack(uint8_t(str().size()));
            rc &= data->pack(str);
        }
        if(!rc) data.reset();
        return data;
    }
};

// Not available in iNav (183-185)
// MSP_COPY_PROFILE                = 183,
struct CopyProfile : public Message {
    CopyProfile(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_COPY_PROFILE; }

    Value<uint8_t> profile_type;
    Value<uint8_t> dest_profile_idx;
    Value<uint8_t> src_profile_idx;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(profile_type);
        rc &= data->pack(dest_profile_idx);
        rc &= data->pack(src_profile_idx);
        if(!rc) data.reset();
        return data;
    }
};

struct BeeperConfigSettings {
    Value<uint32_t> beeper_off_mask;
    Value<uint8_t> beacon_tone;
};

// MSP_BEEPER_CONFIG               = 184,
struct BeeperConfig : public BeeperConfigSettings, public Message {
    BeeperConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_BEEPER_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(beeper_off_mask);
        rc &= data.unpack(beacon_tone);
        return rc;
    }
};

// MSP_SET_BEEPER_CONFIG           = 185,
struct SetBeeperConfig : public BeeperConfigSettings, public Message {
    SetBeeperConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_BEEPER_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(beeper_off_mask);
        if(beacon_tone.set()) {
            rc &= data->pack(beacon_tone);
        }
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_TX_INFO                 = 186, // in message           Used to send
// runtime information from TX lua scripts to the firmware
struct SetTxInfo : public Message {
    SetTxInfo(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_TX_INFO; }

    Value<uint8_t> rssi;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(rssi)) data.reset();
        return data;
    }
};

// MSP_TX_INFO                     = 187, // out message          Used by TX lua
// scripts to read information from the firmware
struct TxInfo : public Message {
    TxInfo(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_TX_INFO; }

    Value<uint8_t> rssi_source;
    Value<uint8_t> rtc_date_time_status;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(rssi_source);
        rc &= data.unpack(rtc_date_time_status);
        return rc;
    }
};

/////////////////////////////////////////////////////////////////////
/// Response (2xx)

// MSP_SET_RAW_RC: 200
// This message is accepted but ignored on betaflight 3.0.1 onwards
// if "USE_RX_MSP" is not defined for the target. In this case, you can manually
// add "#define USE_RX_MSP" to your 'target.h'.
struct SetRawRc : public Message {
    SetRawRc(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RAW_RC; }

    std::vector<uint16_t> channels;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        for(const uint16_t& c : channels) {
            rc &= data->pack(c);
        }
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_RAW_GPS: 201
struct SetRawGPS : public Message {
    SetRawGPS(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RAW_GPS; }

    Value<uint8_t> fix;
    Value<uint8_t> numSat;
    Value<uint32_t> lat;
    Value<uint32_t> lon;
    Value<uint16_t> altitude;
    Value<uint16_t> speed;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(fix);
        rc &= data->pack(numSat);
        rc &= data->pack(lat);
        rc &= data->pack(lon);
        rc &= data->pack(altitude);
        rc &= data->pack(speed);
        assert(data->size() == 14);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_PID: 202,
struct SetPid : public PidSettings, public Message {
    SetPid(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_PID; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        for(uint8_t i = 0;
            i < static_cast<uint8_t>(PID_Element::PID_ITEM_COUNT);
            ++i) {
            rc &= data->pack(entry[i]);
        }
        if(!rc) data.reset();
        return data;
    }
};

// Depricated - no examples
// MSP_SET_BOX: 203,
struct SetBox : public Message {
    SetBox(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_BOX; }
};

// Differences between iNav and BF/CF - this is BF/CF variant
// MSP_SET_RC_TUNING: 204
struct SetRcTuning : public RcTuningSettings, public Message {
    SetRcTuning(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RC_TUNING; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(rcRates[0]);
        rc &= data->pack(rcExpo[0]);
        for(const auto& r : rates) {
            rc &= data->pack(r);
        }
        rc &= data->pack(dynamic_throttle_pid);
        rc &= data->pack(throttle_mid);
        rc &= data->pack(throttle_expo);
        rc &= data->pack(tpa_breakpoint);
        // this field is optional in all firmwares

        if(!rcExpo[2].set()) goto packing_finished;
        rc &= data->pack(rcExpo[2]);
        // INAV quits her

        if(fw_variant == FirmwareVariant::INAV) goto packing_finished;
        // these fields are optional in BF/CF

        if(!rcRates[2].set()) goto packing_finished;
        rc &= data->pack(rcRates[2]);

        if(!rcRates[1].set()) goto packing_finished;
        rc &= data->pack(rcRates[1]);

        if(!rcExpo[1].set()) goto packing_finished;
        rc &= data->pack(rcExpo[1]);

    packing_finished:
        if(!rc) data.reset();
        return data;
    }
};

// MSP_ACC_CALIBRATION: 205
struct AccCalibration : public Message {
    AccCalibration(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ACC_CALIBRATION; }
};

// MSP_MAG_CALIBRATION: 206
struct MagCalibration : public Message {
    MagCalibration(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_MAG_CALIBRATION; }
};

// MSP_SET_MISC: 207
struct SetMisc : public MiscSettings, public Message {
    SetMisc(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_MISC; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(mid_rc);
        rc &= data->pack(min_throttle);
        rc &= data->pack(max_throttle);
        rc &= data->pack(failsafe_throttle);
        rc &= data->pack(gps_provider);
        rc &= data->pack(gps_baudrate);
        rc &= data->pack(gps_ubx_sbas);
        rc &= data->pack(multiwii_current_meter_output);
        rc &= data->pack(rssi_channel);
        rc &= data->pack(reserved);
        rc &= data->pack<uint16_t>(mag_declination, 10.f);
        rc &= data->pack<uint8_t>(voltage_scale, 10.f);
        rc &= data->pack<uint8_t>(cell_min, 10.f);
        rc &= data->pack<uint8_t>(cell_max, 10.f);
        rc &= data->pack<uint8_t>(cell_warning, 10.f);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_RESET_CONF: 208
struct ResetConfig : public Message {
    ResetConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RESET_CONF; }
};

// MSP_SET_WP: 209
struct SetWp : public Message {
    SetWp(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_WP; }

    Value<uint8_t> wp_no;
    Value<uint32_t> lat;
    Value<uint32_t> lon;
    Value<uint32_t> alt;

    Value<uint16_t> p1;
    Value<uint16_t> p2;
    Value<uint16_t> p3;
    Value<uint8_t> nav_flag;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(wp_no);
        rc &= data->pack(lat);
        rc &= data->pack(lon);
        rc &= data->pack(alt);
        rc &= data->pack(p1);
        rc &= data->pack(p2);
        if(fw_variant == FirmwareVariant::INAV) {
            rc &= data->pack(p3);
        }
        rc &= data->pack(nav_flag);
        return data;
    }
};

// MSP_SELECT_SETTING: 210
struct SelectSetting : public Message {
    SelectSetting(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SELECT_SETTING; }

    uint8_t current_setting;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(current_setting)) data.reset();
        return data;
    }
};

// MSP_SET_HEADING: 211
struct SetHeading : public Message {
    SetHeading(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_HEADING; }

    int16_t heading;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(heading)) data.reset();
        assert(data->size() == 2);
        return data;
    }
};

// MSP_SET_SERVO_CONF: 212
struct SetServoConf : public Message {
    SetServoConf(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_SERVO_CONF; }

    Value<uint8_t> servo_idx;
    Value<uint16_t> min;
    Value<uint16_t> max;
    Value<uint16_t> middle;
    Value<uint8_t> rate;

    Value<uint8_t> forward_from_channel;
    Value<uint32_t> reversed_sources;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(servo_idx);
        rc &= data->pack(min);
        rc &= data->pack(max);
        rc &= data->pack(middle);
        rc &= data->pack(rate);
        if(fw_variant == FirmwareVariant::INAV) {
            uint8_t tmp = 0;
            rc &= data->pack(tmp);
            rc &= data->pack(tmp);
        }
        rc &= data->pack(forward_from_channel);
        rc &= data->pack(reversed_sources);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_MOTOR: 214
struct SetMotor : public Message {
    SetMotor(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_MOTOR; }

    std::array<uint16_t, N_MOTOR> motor;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        for(size_t i(0); i < N_MOTOR; i++) rc &= data->pack(motor[i]);
        assert(data->size() == N_MOTOR * 2);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_NAV_CONFIG              = 215
struct SetNavConfig : public Message {
    SetNavConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_NAV_CONFIG; }
};

// MSP_SET_MOTOR_3D_CONF           = 217
struct SetMotor3dConf : public Message {
    SetMotor3dConf(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_MOTOR_3D_CONF; }

    Value<uint16_t> deadband3d_low;
    Value<uint16_t> deadband3d_high;
    Value<uint16_t> neutral_3d;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(deadband3d_low);
        rc &= data->pack(deadband3d_high);
        rc &= data->pack(neutral_3d);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_RC_DEADBAND             = 218
struct SetRcDeadband : public RcDeadbandSettings, public Message {
    SetRcDeadband(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RC_DEADBAND; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(deadband);
        rc &= data->pack(yaw_deadband);
        rc &= data->pack(alt_hold_deadband);
        rc &= data->pack(deadband3d_throttle);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_RESET_CURR_PID          = 219
struct SetResetCurrPid : public Message {
    SetResetCurrPid(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RESET_CURR_PID; }
};

// MSP_SET_SENSOR_ALIGNMENT        = 220
struct SetSensorAlignment : public SensorAlignmentSettings, public Message {
    SetSensorAlignment(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_SENSOR_ALIGNMENT; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(gyro_align);
        rc &= data->pack(acc_align);
        rc &= data->pack(mag_align);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_LED_STRIP_MODECOLOR     = 221
struct SetLedStripModecolor : public SensorAlignmentSettings, public Message {
    SetLedStripModecolor(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_LED_STRIP_MODECOLOR; }

    Value<uint8_t> mode_idx;
    Value<uint8_t> fun_idx;
    Value<uint8_t> color;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(mode_idx);
        rc &= data->pack(fun_idx);
        rc &= data->pack(color);
        if(!rc) data.reset();
        return data;
    }
};

// Not available in iNav (222-224)
// MSP_SET_MOTOR_CONFIG            = 222    //out message         Motor
// configuration (min/max throttle, etc)
struct SetMotorConfig : public MotorConfigSettings, public Message {
    SetMotorConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_MOTOR_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(min_throttle);
        rc &= data->pack(max_throttle);
        rc &= data->pack(min_command);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_GPS_CONFIG              = 223    //out message         GPS
// configuration
struct SetGpsConfig : public GpsConfigSettings, public Message {
    SetGpsConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_GPS_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(provider);
        rc &= data->pack(sbas_mode);
        rc &= data->pack(auto_config);
        rc &= data->pack(auto_baud);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_SET_COMPASS_CONFIG          = 224    //out message         Compass
// configuration
struct SetCompassConfig : public Message {
    SetCompassConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_GPS_CONFIG; }

    Value<float> mag_declination;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack<uint16_t>(mag_declination, 10)) data.reset();
        return data;
    }
};

struct AccTrimSettings {
    Value<uint16_t> pitch;
    Value<uint16_t> roll;
};

// MSP_SET_ACC_TRIM                = 239    //in message          set acc angle
// trim values
struct SetAccTrim : public AccTrimSettings, public Message {
    SetAccTrim(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_ACC_TRIM; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(pitch);
        rc &= data->pack(roll);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_ACC_TRIM                    = 240    //out message         get acc angle
// trim values
struct AccTrim : public AccTrimSettings, public Message {
    AccTrim(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_ACC_TRIM; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(pitch);
        rc &= data.unpack(roll);
        return rc;
    }
};

struct ServoMixRule : public Packable {
    uint8_t target_channel;
    uint8_t input_source;
    uint8_t rate;
    uint8_t speed;
    uint8_t min;
    uint8_t max;
    uint8_t box;

    bool unpack_from(const ByteVector& data) {
        bool rc = true;
        rc &= data.unpack(target_channel);
        rc &= data.unpack(input_source);
        rc &= data.unpack(rate);
        rc &= data.unpack(speed);
        rc &= data.unpack(min);
        rc &= data.unpack(max);
        rc &= data.unpack(box);
        return rc;
    }

    bool pack_into(ByteVector& data) const {
        bool rc = true;
        rc &= data.pack(target_channel);
        rc &= data.pack(input_source);
        rc &= data.pack(rate);
        rc &= data.pack(speed);
        rc &= data.pack(min);
        rc &= data.pack(max);
        rc &= data.pack(box);
        return rc;
    }
};

// MSP_SERVO_MIX_RULES             = 241    //out message         Returns servo
// mixer configuration
struct ServoMixRules : public Message {
    ServoMixRules(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SERVO_MIX_RULES; }

    std::vector<Value<ServoMixRule>> rules;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        while(data.unpacking_remaining()) {
            Value<ServoMixRule> rule;
            rc &= data.unpack(rule);
            if(rc)
                rules.push_back(rule);
            else
                break;
        }
        return rc;
    }
};

// MSP_SET_SERVO_MIX_RULE          = 242    //in message          Sets servo
// mixer configuration
struct SetServoMixRule : public Message {
    SetServoMixRule(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SERVO_MIX_RULES; }

    Value<ServoMixRule> rule;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(rule)) data.reset();
        return data;
    }
};

// not used in CF, BF, iNav
// MSP_PASSTHROUGH_SERIAL          = 244
struct PassthroughSerial : public Message {
    PassthroughSerial(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_PASSTHROUGH_SERIAL; }
};

// MSP_SET_4WAY_IF                 = 245    //in message          Sets 4way
// interface
struct Set4WayIF : public Message {
    Set4WayIF(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_4WAY_IF; }

    Value<uint8_t> esc_mode;
    Value<uint8_t> esc_port_index;

    Value<uint8_t> esc_count;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        if(esc_mode.set()) {
            rc &= data->pack(esc_mode);
            rc &= data->pack(esc_port_index);
        }
        if(!rc) data.reset();
        return data;
    }

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(esc_count);
    }
};

struct RtcVals {
    Value<uint32_t> secs;
    Value<uint16_t> millis;
};

// MSP_SET_RTC                     = 246    //in message          Sets the RTC
// clock
struct SetRtc : public RtcVals, public Message {
    SetRtc(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_SET_RTC; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(secs);
        rc &= data->pack(millis);
        if(!rc) data.reset();
        return data;
    }
};

// MSP_RTC                         = 247    //out message         Gets the RTC
// clock
struct Rtc : public RtcVals, public Message {
    Rtc(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RTC; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(secs);
        rc &= data.unpack(millis);
        return rc;
    }
};

// MSP_EEPROM_WRITE: 250
struct WriteEEPROM : public Message {
    WriteEEPROM(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_EEPROM_WRITE; }
};

// MSP_RESERVE_1: 251,    //reserved for system usage
struct Reserve1 : public Message {
    Reserve1(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RESERVE_1; }
};

// MSP_RESERVE_2: 252,    //reserved for system usage
struct Reserve2 : public Message {
    Reserve2(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_RESERVE_2; }
};

// MSP_DEBUGMSG: 253
struct DebugMessage : public Message {
    DebugMessage(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_DEBUGMSG; }

    Value<std::string> debug_msg;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(debug_msg);
    }
};

// MSP_DEBUG: 254
struct Debug : public Message {
    Debug(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_DEBUG; }

    Value<uint16_t> debug1;
    Value<uint16_t> debug2;
    Value<uint16_t> debug3;
    Value<uint16_t> debug4;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(debug1);
        rc &= data.unpack(debug2);
        rc &= data.unpack(debug3);
        rc &= data.unpack(debug4);
        return rc;
    }
};

// MSP_V2_FRAME: 255
struct V2Frame : public Message {
    V2Frame(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP_V2_FRAME; }
};

// MSP2_COMMON_TZ                  = 0x1001,  //out message   Gets the TZ offset
// for the local time (returns: minutes(i16))
struct CommonTz : public Message {
    CommonTz(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_COMMON_TZ; }

    Value<uint16_t> tz_offset;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(tz_offset);
    }
};

// MSP2_COMMON_SET_TZ              = 0x1002,  //in message    Sets the TZ offset
// for the local time (args: minutes(i16))
struct CommonSetTz : public Message {
    CommonSetTz(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_COMMON_SET_TZ; }

    Value<uint16_t> tz_offset;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        if(!data->pack(tz_offset)) data.reset();
        return data;
    }
};

enum class DATA_TYPE : uint8_t {
    UNSET,
    UINT8,
    INT8,
    UINT16,
    INT16,
    UINT32,
    FLOAT,
    STRING
};

// MSP2_COMMON_SETTING             = 0x1003,  //in/out message   Returns the
// value for a setting
struct CommonSetting : public Message {
    CommonSetting(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_COMMON_SETTING; }

    Value<std::string> setting_name;
    Value<uint8_t> uint8_val;
    Value<int8_t> int8_val;
    Value<uint16_t> uint16_val;
    Value<int16_t> int16_val;
    Value<uint32_t> uint32_val;
    Value<float> float_val;
    Value<std::string> string_val;

    DATA_TYPE expected_data_type;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(setting_name);
        if(!rc) data.reset();
        return data;
    }

    // TODO
    virtual bool decode(const ByteVector& data) override {
        std::cout << "decoding " << data;
        switch(expected_data_type) {
        case DATA_TYPE::UINT8:
            return data.unpack(uint8_val);
        case DATA_TYPE::INT8:
            return data.unpack(int8_val);
        case DATA_TYPE::UINT16:
            return data.unpack(uint16_val);
        case DATA_TYPE::INT16:
            return data.unpack(int16_val);
        case DATA_TYPE::UINT32:
            return data.unpack(uint32_val);
        case DATA_TYPE::FLOAT:
            return data.unpack(float_val);
        case DATA_TYPE::STRING:
            return data.unpack(string_val);
        default:
            return false;
        }
    }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Setting:" << std::endl;
        s << " " << setting_name << ": ";

        switch(expected_data_type) {
        case DATA_TYPE::UINT8:
            s << uint8_val;
            break;
        case DATA_TYPE::INT8:
            s << int8_val;
            break;
        case DATA_TYPE::UINT16:
            s << uint16_val;
            break;
        case DATA_TYPE::INT16:
            s << int16_val;
            break;
        case DATA_TYPE::UINT32:
            s << uint32_val;
            break;
        case DATA_TYPE::FLOAT:
            s << float_val;
            break;
        case DATA_TYPE::STRING:
            s << string_val;
            break;
        default:
            s << "<invalid>";
        }

        s << std::endl;

        return s;
    }
};

// MSP2_COMMON_SET_SETTING         = 0x1004,  //in message    Sets the value for
// a setting
struct CommonSetSetting : public Message {
    CommonSetSetting(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_COMMON_SET_SETTING; }

    Value<std::string> setting_name;
    Value<uint8_t> uint8_val;
    Value<int8_t> int8_val;
    Value<uint16_t> uint16_val;
    Value<int16_t> int16_val;
    Value<uint32_t> uint32_val;
    Value<float> float_val;
    Value<std::string> string_val;

    DATA_TYPE expected_data_type;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(setting_name);
        if(uint8_val.set())
            rc &= data->pack(uint8_val);
        else if(int8_val.set())
            rc &= data->pack(int8_val);
        else if(uint16_val.set())
            rc &= data->pack(uint16_val);
        else if(int16_val.set())
            rc &= data->pack(int16_val);
        else if(uint32_val.set())
            rc &= data->pack(uint32_val);
        else if(float_val.set())
            rc &= data->pack(float_val);
        else if(string_val.set())
            rc &= data->pack(string_val);
        if(!rc) data.reset();
        return data;
    }
};

struct MotorMixer : public Packable {
    Value<float> throttle;
    Value<float> roll;
    Value<float> pitch;
    Value<float> yaw;

    bool unpack_from(const ByteVector& data) {
        bool rc = true;
        rc &= data.unpack<uint16_t>(throttle, 1000.0);
        rc &= data.unpack<uint16_t>(roll, 1000.0, 1.0);
        rc &= data.unpack<uint16_t>(pitch, 1000.0, 1.0);
        rc &= data.unpack<uint16_t>(yaw, 1000.0, 1.0);
        return rc;
    }

    bool pack_into(ByteVector& data) const {
        bool rc = true;
        rc &= data.pack<uint16_t>(throttle, 1000.0, 1.0);
        rc &= data.pack<uint16_t>(roll, 1000.0, 1.0);
        rc &= data.pack<uint16_t>(pitch, 1000.0, 1.0);
        rc &= data.pack<uint16_t>(yaw, 1000.0, 1.0);
        return rc;
    }
};

// MSP2_COMMON_MOTOR_MIXER         = 0x1005,
struct CommonMotorMixer : public Message {
    CommonMotorMixer(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_COMMON_MOTOR_MIXER; }

    std::vector<MotorMixer> mixer;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        while(data.unpacking_remaining()) {
            MotorMixer m;
            rc &= data.unpack(m);
            mixer.push_back(m);
        }
        return rc;
    }
};

// MSP2_COMMON_SET_MOTOR_MIXER     = 0x1006,
struct CommonSetMotorMixer : public Message {
    CommonSetMotorMixer(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_COMMON_SET_MOTOR_MIXER; }

    Value<uint8_t> index;
    MotorMixer mixer;

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(index);
        rc &= data->pack(mixer);
        if(!rc) data.reset();
        return data;
    }
};

// MSP2_INAV_STATUS                = 0x2000,
struct InavStatus : public StatusBase, public Message {
    InavStatus(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_STATUS; }

    Value<uint16_t> avg_system_load_pct;
    Value<uint8_t> config_profile;
    Value<uint32_t> arming_flags;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(cycle_time);
        rc &= data.unpack(i2c_errors);

        // get sensors
        sensors.clear();
        uint16_t sensor = 0;
        rc &= data.unpack(sensor);
        if(sensor & (1 << 0)) sensors.insert(Sensor::Accelerometer);
        if(sensor & (1 << 1)) sensors.insert(Sensor::Barometer);
        if(sensor & (1 << 2)) sensors.insert(Sensor::Magnetometer);
        if(sensor & (1 << 3)) sensors.insert(Sensor::GPS);
        if(sensor & (1 << 4)) sensors.insert(Sensor::Sonar);
        if(sensor & (1 << 5)) sensors.insert(Sensor::OpticalFlow);
        if(sensor & (1 << 6)) sensors.insert(Sensor::Pitot);
        if(sensor & (1 << 15)) sensors.insert(Sensor::GeneralHealth);

        rc &= data.unpack(avg_system_load_pct);
        rc &= data.unpack(config_profile);

        rc &= data.unpack(arming_flags);

        // check active boxes
        box_mode_flags.clear();
        uint32_t flag = 0;
        rc &= data.unpack(flag);
        for(size_t ibox(0); ibox < sizeof(flag) * CHAR_BIT; ibox++) {
            if(flag & (1 << ibox)) box_mode_flags.insert(ibox);
        }

        return rc;
    }

    bool hasAccelerometer() const {
        return sensors.count(Sensor::Accelerometer);
    }

    bool hasBarometer() const { return sensors.count(Sensor::Barometer); }

    bool hasMagnetometer() const { return sensors.count(Sensor::Magnetometer); }

    bool hasGPS() const { return sensors.count(Sensor::GPS); }

    bool hasSonar() const { return sensors.count(Sensor::Sonar); }

    bool hasOpticalFlow() const { return sensors.count(Sensor::OpticalFlow); }

    bool hasPitot() const { return sensors.count(Sensor::Pitot); }

    bool isHealthy() const { return sensors.count(Sensor::GeneralHealth); }

    virtual std::ostream& print(std::ostream& s) const override {
        s << "#Status:" << std::endl;
        s << " Cycle time: " << cycle_time << " us" << std::endl;
        s << " Average system load: " << avg_system_load_pct << "%"
          << std::endl;
        s << " Arming flags: " << armingFlagToString(arming_flags())
          << std::endl;
        s << " I2C errors: " << i2c_errors << std::endl;
        s << " Sensors:" << std::endl;

        s << "    Accelerometer: ";
        hasAccelerometer() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    Barometer: ";
        hasBarometer() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    Magnetometer: ";
        hasMagnetometer() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    GPS: ";
        hasGPS() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << "    Sonar: ";
        hasSonar() ? s << "ON" : s << "OFF";
        s << std::endl;

        s << " Active Boxes (by ID):";
        for(const size_t box_id : box_mode_flags) {
            s << " " << box_id;
        }
        s << std::endl;

        return s;
    }
};

// MSP2_INAV_OPTICAL_FLOW          = 0x2001,
struct InavOpticalFlow : public Message {
    InavOpticalFlow(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_OPTICAL_FLOW; }

    Value<uint8_t> raw_quality;
    Value<uint16_t> flow_rate_x;
    Value<uint16_t> flow_rate_y;
    Value<uint16_t> body_rate_x;
    Value<uint16_t> body_rate_y;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(raw_quality);
        rc &= data.unpack(flow_rate_x);
        rc &= data.unpack(flow_rate_y);
        rc &= data.unpack(body_rate_x);
        rc &= data.unpack(body_rate_y);
        return rc;
    }
};

// MSP2_INAV_ANALOG                = 0x2002,
struct InavAnalog : public Message {
    InavAnalog(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_ANALOG; }

    Value<uint8_t> battery_voltage;
    Value<uint16_t> mAh_drawn;
    Value<uint16_t> rssi;
    Value<uint16_t> amperage;

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(battery_voltage);
        rc &= data.unpack(mAh_drawn);
        rc &= data.unpack(rssi);
        rc &= data.unpack(amperage);
        return rc;
    }
};

struct InavMiscSettings {
    Value<uint16_t> mid_rc;
    Value<uint16_t> min_throttle;
    Value<uint16_t> max_throttle;
    Value<uint16_t> min_command;
    Value<uint16_t> failsafe_throttle;
    Value<uint8_t> gps_provider;
    Value<uint8_t> gps_baudrate;
    Value<uint8_t> gps_ubx_sbas;
    Value<uint8_t> rssi_channel;
    Value<uint16_t> mag_declination;
    Value<uint16_t> voltage_scale;
    Value<uint16_t> cell_min;
    Value<uint16_t> cell_max;
    Value<uint16_t> cell_warning;
    Value<uint32_t> capacity;
    Value<uint32_t> capacity_warning;
    Value<uint32_t> capacity_critical;
    Value<uint8_t> capacity_units;
};

// MSP2_INAV_MISC                  = 0x2003,
struct InavMisc : public InavMiscSettings, public Message {
    InavMisc(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_MISC; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(mid_rc);
        rc &= data.unpack(min_throttle);
        rc &= data.unpack(max_throttle);
        rc &= data.unpack(min_command);
        rc &= data.unpack(failsafe_throttle);
        rc &= data.unpack(gps_provider);
        rc &= data.unpack(gps_baudrate);
        rc &= data.unpack(gps_ubx_sbas);
        rc &= data.unpack(rssi_channel);
        rc &= data.unpack(mag_declination);
        rc &= data.unpack(voltage_scale);
        rc &= data.unpack(cell_min);
        rc &= data.unpack(cell_max);
        rc &= data.unpack(cell_warning);
        rc &= data.unpack(capacity);
        rc &= data.unpack(capacity_warning);
        rc &= data.unpack(capacity_critical);
        rc &= data.unpack(capacity_units);
        return rc;
    }
};

// MSP2_INAV_SET_MISC              = 0x2004,
struct InavSetMisc : public InavMiscSettings, public Message {
    InavSetMisc(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_SET_MISC; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(mid_rc);
        rc &= data->pack(min_throttle);
        rc &= data->pack(mid_rc);
        rc &= data->pack(max_throttle);
        rc &= data->pack(min_command);
        rc &= data->pack(failsafe_throttle);
        rc &= data->pack(gps_provider);
        rc &= data->pack(gps_baudrate);
        rc &= data->pack(gps_ubx_sbas);
        rc &= data->pack(rssi_channel);
        rc &= data->pack(mag_declination);
        rc &= data->pack(voltage_scale);
        rc &= data->pack(cell_min);
        rc &= data->pack(cell_max);
        rc &= data->pack(cell_warning);
        rc &= data->pack(capacity);
        rc &= data->pack(capacity_warning);
        rc &= data->pack(capacity_critical);
        rc &= data->pack(capacity_units);
        if(!rc) data.reset();
        return data;
    }
};

struct InavBatteryConfigSettings {
    Value<uint16_t> voltage_scale;
    Value<uint16_t> cell_min;
    Value<uint16_t> cell_max;
    Value<uint16_t> cell_warning;
    Value<uint16_t> current_offset;
    Value<uint16_t> current_scale;
    Value<uint32_t> capacity;
    Value<uint32_t> capacity_warning;
    Value<uint32_t> capacity_critical;
    Value<uint8_t> capacity_units;
};

// MSP2_INAV_BATTERY_CONFIG        = 0x2005,
struct InavBatteryConfig : public InavBatteryConfigSettings, public Message {
    InavBatteryConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_BATTERY_CONFIG; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(voltage_scale);
        rc &= data.unpack(cell_min);
        rc &= data.unpack(cell_max);
        rc &= data.unpack(cell_warning);
        rc &= data.unpack(current_offset);
        rc &= data.unpack(current_scale);
        rc &= data.unpack(capacity);
        rc &= data.unpack(capacity_warning);
        rc &= data.unpack(capacity_critical);
        rc &= data.unpack(capacity_units);
        return rc;
    }
};

// MSP2_INAV_SET_BATTERY_CONFIG    = 0x2006,
struct InavSetBatteryConfig : public InavBatteryConfigSettings, public Message {
    InavSetBatteryConfig(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_SET_BATTERY_CONFIG; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(voltage_scale);
        rc &= data->pack(cell_min);
        rc &= data->pack(cell_max);
        rc &= data->pack(cell_warning);
        rc &= data->pack(current_offset);
        rc &= data->pack(current_scale);
        rc &= data->pack(capacity);
        rc &= data->pack(capacity_warning);
        rc &= data->pack(capacity_critical);
        rc &= data->pack(capacity_units);
        if(!rc) data.reset();
        return data;
    }
};

struct InavRateProfileSettings {
    Value<uint8_t> throttle_rc_mid;
    Value<uint8_t> throttle_rc_expo;
    Value<uint8_t> throttle_dyn_pid;
    Value<uint16_t> throttle_pa_breakpoint;

    Value<uint8_t> stabilized_rc_expo;
    Value<uint8_t> stabilized_rc_yaw_expo;
    Value<uint8_t> stabilized_rate_r;
    Value<uint8_t> stabilized_rate_p;
    Value<uint8_t> stabilized_rate_y;

    Value<uint8_t> manual_rc_expo;
    Value<uint8_t> manual_rc_yaw_expo;
    Value<uint8_t> manual_rate_r;
    Value<uint8_t> manual_rate_p;
    Value<uint8_t> manual_rate_y;
};

// MSP2_INAV_RATE_PROFILE          = 0x2007,
struct InavRateProfile : public InavRateProfileSettings, public Message {
    InavRateProfile(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_RATE_PROFILE; }

    virtual bool decode(const ByteVector& data) override {
        bool rc = true;
        rc &= data.unpack(throttle_rc_mid);
        rc &= data.unpack(throttle_rc_expo);
        rc &= data.unpack(throttle_dyn_pid);
        rc &= data.unpack(throttle_pa_breakpoint);

        rc &= data.unpack(stabilized_rc_expo);
        rc &= data.unpack(stabilized_rc_yaw_expo);
        rc &= data.unpack(stabilized_rate_r);
        rc &= data.unpack(stabilized_rate_p);
        rc &= data.unpack(stabilized_rate_y);

        rc &= data.unpack(manual_rc_expo);
        rc &= data.unpack(manual_rc_yaw_expo);
        rc &= data.unpack(manual_rate_r);
        rc &= data.unpack(manual_rate_p);
        rc &= data.unpack(manual_rate_y);
        return rc;
    }
};

// MSP2_INAV_SET_RATE_PROFILE      = 0x2008,
struct InavSetRateProfile : public InavRateProfileSettings, public Message {
    InavSetRateProfile(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_SET_RATE_PROFILE; }

    virtual ByteVectorUptr encode() const override {
        ByteVectorUptr data = std::make_unique<ByteVector>();
        bool rc             = true;
        rc &= data->pack(throttle_rc_mid);
        rc &= data->pack(throttle_rc_expo);
        rc &= data->pack(throttle_dyn_pid);
        rc &= data->pack(throttle_pa_breakpoint);

        rc &= data->pack(stabilized_rc_expo);
        rc &= data->pack(stabilized_rc_yaw_expo);
        rc &= data->pack(stabilized_rate_r);
        rc &= data->pack(stabilized_rate_p);
        rc &= data->pack(stabilized_rate_y);

        rc &= data->pack(manual_rc_expo);
        rc &= data->pack(manual_rc_yaw_expo);
        rc &= data->pack(manual_rate_r);
        rc &= data->pack(manual_rate_p);
        rc &= data->pack(manual_rate_y);
        if(!rc) data.reset();
        return data;
    }
};

// MSP2_INAV_AIR_SPEED             = 0x2009
struct InavAirSpeed : public InavMiscSettings, public Message {
    InavAirSpeed(FirmwareVariant v) : Message(v) {}

    virtual ID id() const override { return ID::MSP2_INAV_RATE_PROFILE; }

    Value<uint32_t> speed;

    virtual bool decode(const ByteVector& data) override {
        return data.unpack(speed);
    }
};

}  // namespace msg
}  // namespace msp

inline std::ostream& operator<<(std::ostream& s, const msp::msg::ImuSI& val) {
    return val.print(s);
}

#endif  // MSP_MSG_HPP
