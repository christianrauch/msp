#ifndef MSP_ID_HPP
#define MSP_ID_HPP

#include <cstdint>
#include <string>
#include <iostream>

namespace msp {

enum class ID : uint16_t {
    // Cleanflight
    MSP_API_VERSION                 = 1,
    MSP_FC_VARIANT                  = 2,
    MSP_FC_VERSION                  = 3,
    MSP_BOARD_INFO                  = 4,
    MSP_BUILD_INFO                  = 5,
    
    MSP_INAV_PID                    = 6,
    MSP_SET_INAV_PID                = 7,

    MSP_NAME                        = 10,   //out message          Returns user set board name - betaflight
    MSP_SET_NAME                    = 11,   //in message           Sets board name - betaflight

    //only in iNav (12-24)
    MSP_NAV_POSHOLD                 = 12,
    MSP_SET_NAV_POSHOLD             = 13,

    MSP_CALIBRATION_DATA            = 14,
    MSP_SET_CALIBRATION_DATA        = 15,

    MSP_POSITION_ESTIMATION_CONFIG  = 16,
    MSP_SET_POSITION_ESTIMATION_CONFIG  = 17,

    MSP_WP_MISSION_LOAD             = 18,      // Load mission from NVRAM
    MSP_WP_MISSION_SAVE             = 19,      // Save mission to NVRAM
    MSP_WP_GETINFO                  = 20,

    MSP_RTH_AND_LAND_CONFIG         = 21,
    MSP_SET_RTH_AND_LAND_CONFIG     = 22,
    
    MSP_FW_CONFIG                   = 23,
    MSP_SET_FW_CONFIG               = 24,

    //Not avaialable in iNav (32,33)
    MSP_BATTERY_CONFIG              = 32,
    MSP_SET_BATTERY_CONFIG          = 33,
    
    MSP_MODE_RANGES                 = 34,
    MSP_SET_MODE_RANGE              = 35,
    
    MSP_FEATURE                     = 36,
    MSP_SET_FEATURE                 = 37,
    
    MSP_BOARD_ALIGNMENT             = 38,
    MSP_SET_BOARD_ALIGNMENT         = 39,
    
    MSP_CURRENT_METER_CONFIG        = 40,
    MSP_SET_CURRENT_METER_CONFIG    = 41,
    
    MSP_MIXER                       = 42,
    MSP_SET_MIXER                   = 43,
    
    MSP_RX_CONFIG                   = 44,
    MSP_SET_RX_CONFIG               = 45,
    
    MSP_LED_COLORS                  = 46,
    MSP_SET_LED_COLORS              = 47,
    
    MSP_LED_STRIP_CONFIG            = 48,
    MSP_SET_LED_STRIP_CONFIG        = 49,
    
    MSP_RSSI_CONFIG                 = 50,
    MSP_SET_RSSI_CONFIG             = 51,
    
    MSP_ADJUSTMENT_RANGES           = 52,
    MSP_SET_ADJUSTMENT_RANGE        = 53,

    MSP_CF_SERIAL_CONFIG            = 54,
    MSP_SET_CF_SERIAL_CONFIG        = 55,

    MSP_VOLTAGE_METER_CONFIG        = 56,
    MSP_SET_VOLTAGE_METER_CONFIG    = 57,

    MSP_SONAR_ALTITUDE              = 58,
    
    MSP_PID_CONTROLLER              = 59,
    MSP_SET_PID_CONTROLLER          = 60,

    MSP_ARMING_CONFIG               = 61,
    MSP_SET_ARMING_CONFIG           = 62,

    MSP_RX_MAP                      = 64,
    MSP_SET_RX_MAP                  = 65,
    
    //Depricated - still in BF and iNav (66,67)
    MSP_BF_CONFIG                   = 66, //out message baseflight-specific settings that aren't covered elsewhere
    MSP_SET_BF_CONFIG               = 67, //in message baseflight-specific settings save

    MSP_REBOOT                      = 68,

    //Depricated - still available in iNav (69)
    MSP_BF_BUILD_INFO               = 69,

    MSP_DATAFLASH_SUMMARY           = 70,
    MSP_DATAFLASH_READ              = 71,
    MSP_DATAFLASH_ERASE             = 72,
    
    //Depricated - still available in iNav(73,74)
    MSP_LOOP_TIME                   = 73,
    MSP_SET_LOOP_TIME               = 74,

    MSP_FAILSAFE_CONFIG             = 75,
    MSP_SET_FAILSAFE_CONFIG         = 76,
    
    //Depricated in iNav (77,78)
    MSP_RXFAIL_CONFIG               = 77,
    MSP_SET_RXFAIL_CONFIG           = 78,

    MSP_SDCARD_SUMMARY              = 79,

    MSP_BLACKBOX_CONFIG             = 80,
    MSP_SET_BLACKBOX_CONFIG         = 81,

    MSP_TRANSPONDER_CONFIG          = 82,
    MSP_SET_TRANSPONDER_CONFIG      = 83,

    MSP_OSD_CONFIG                  = 84, //out message         Get osd settings - betaflight
    MSP_SET_OSD_CONFIG              = 85, //in message          Set osd settings - betaflight

    MSP_OSD_CHAR_READ               = 86, //out message         Get osd settings - betaflight
    MSP_OSD_CHAR_WRITE              = 87,

    MSP_VTX_CONFIG                  = 88,
    MSP_SET_VTX_CONFIG              = 89,
    
    MSP_ADVANCED_CONFIG             = 90,
    MSP_SET_ADVANCED_CONFIG         = 91,

    MSP_FILTER_CONFIG               = 92,
    MSP_SET_FILTER_CONFIG           = 93,

    MSP_PID_ADVANCED                = 94,
    MSP_SET_PID_ADVANCED            = 95,

    MSP_SENSOR_CONFIG               = 96,
    MSP_SET_SENSOR_CONFIG           = 97,

    //Depricated - IDs have been reassigned (98,99)
    //MSP_SPECIAL_PARAMETERS        = 98, // Temporary betaflight parameters before cleanup and k$
    //MSP_SET_SPECIAL_PARAMETERS    = 99, // Temporary betaflight parameters before cleanup and k$

    MSP_CAMERA_CONTROL              = 98,
    
    MSP_SET_ARMING_DISABLED         = 99,


    // MultiWii
    //Depricated - still available in iNav (100)
    MSP_IDENT                       = 100,
    MSP_STATUS                      = 101,
    MSP_RAW_IMU                     = 102,
    MSP_SERVO                       = 103,
    MSP_MOTOR                       = 104,
    MSP_RC                          = 105,
    MSP_RAW_GPS                     = 106,
    MSP_COMP_GPS                    = 107,
    MSP_ATTITUDE                    = 108,
    MSP_ALTITUDE                    = 109,
    MSP_ANALOG                      = 110,
    MSP_RC_TUNING                   = 111,
    MSP_PID                         = 112,
    //Depricated - still available in iNav (113-115)
    MSP_ACTIVEBOXES                 = 113,
    MSP_MISC                        = 114,
    MSP_MOTOR_PINS                  = 115,
    MSP_BOXNAMES                    = 116,
    MSP_PIDNAMES                    = 117,
    MSP_WP                          = 118,
    MSP_BOXIDS                      = 119,
    MSP_SERVO_CONF                  = 120,
    MSP_NAV_STATUS                  = 121,
    MSP_NAV_CONFIG                  = 122,
    MSP_MOTOR_3D_CONFIG             = 124,
    MSP_RC_DEADBAND                 = 125,
    MSP_SENSOR_ALIGNMENT            = 126,
    MSP_LED_STRIP_MODECOLOR         = 127,
    //Not present in iNav (128-134)
    MSP_VOLTAGE_METERS              = 128,
    MSP_CURRENT_METERS              = 129,
    MSP_BATTERY_STATE               = 130,
    MSP_MOTOR_CONFIG                = 131,    //out message         Motor configuration (min/max throttle, etc)
    MSP_GPS_CONFIG                  = 132,    //out message         GPS configuration
    MSP_COMPASS_CONFIG              = 133,    //out message         Compass configuration
    MSP_ESC_SENSOR_DATA             = 134,    //out message         Extra ESC data from 32-Bit ESCs (Temperature, RPM)

    MSP_STATUS_EX                   = 150,
    //Only in iNav (151)
    MSP_SENSOR_STATUS               = 151,
    MSP_UID                         = 160,
    MSP_GPSSVINFO                   = 164,
    MSP_GPSSTATISTICS               = 166,

    MSP_OSD_VIDEO_CONFIG            = 180,
    MSP_SET_OSD_VIDEO_CONFIG        = 181,
    
    MSP_DISPLAYPORT                 = 182,

    // Not available in iNav (183-185)
    MSP_COPY_PROFILE                = 183,

    MSP_BEEPER_CONFIG               = 184,
    MSP_SET_BEEPER_CONFIG           = 185,

    MSP_SET_TX_INFO                 = 186, // in message           Used to send runtime information from TX lua scripts to the firmware
    MSP_TX_INFO                     = 187, // out message          Used by TX lua scripts to read information from the firmware

    
    MSP_SET_RAW_RC                  = 200,
    MSP_SET_RAW_GPS                 = 201,
    MSP_SET_PID                     = 202,
    //Depricated - still available in iNav
    MSP_SET_BOX                     = 203,
    MSP_SET_RC_TUNING               = 204,
    MSP_ACC_CALIBRATION             = 205,
    MSP_MAG_CALIBRATION             = 206,
    //Depricated - still available in iNav
    MSP_SET_MISC                    = 207,
    MSP_RESET_CONF                  = 208,
    MSP_SET_WP                      = 209,
    MSP_SELECT_SETTING              = 210,
    MSP_SET_HEADING                 = 211,
    MSP_SET_SERVO_CONF              = 212,
    
    MSP_SET_MOTOR                   = 214,
    MSP_SET_NAV_CONFIG              = 215,

    MSP_SET_MOTOR_3D_CONF           = 217,
    MSP_SET_RC_DEADBAND             = 218,
    MSP_SET_RESET_CURR_PID          = 219,
    MSP_SET_SENSOR_ALIGNMENT        = 220,
    MSP_SET_LED_STRIP_MODECOLOR     = 221,
    //Not available in iNav (222-224)
    MSP_SET_MOTOR_CONFIG            = 222,    //out message         Motor configuration (min/max throttle, etc)
    MSP_SET_GPS_CONFIG              = 223,    //out message         GPS configuration
    MSP_SET_COMPASS_CONFIG          = 224,    //out message         Compass configuration
    
    //Depricated - conflicting ID assignment
    //MSP_BIND                      = 240,    //in message          no param
    
    MSP_SET_ACC_TRIM                = 239,    //in message          set acc angle trim values
    MSP_ACC_TRIM                    = 240,    //out message         get acc angle trim values
    MSP_SERVO_MIX_RULES             = 241,    //out message         Returns servo mixer configuration
    
    //Depricated - conflicting ID assignment
    //MSP_ALARMS                    = 242,

    MSP_SET_SERVO_MIX_RULE          = 242,    //in message          Sets servo mixer configuration
    //not used in CF, BF, iNav
    MSP_PASSTHROUGH_SERIAL          = 244,
    MSP_SET_4WAY_IF                 = 245,    //in message          Sets 4way interface
    MSP_SET_RTC                     = 246,    //in message          Sets the RTC clock
    MSP_RTC                         = 247,    //out message         Gets the RTC clock

    MSP_EEPROM_WRITE                = 250,    //in message          no param
    MSP_RESERVE_1                   = 251,    //reserved for system usage
    MSP_RESERVE_2                   = 252,    //reserved for system usage
    MSP_DEBUGMSG                    = 253,    //out message         debug string buffer
    MSP_DEBUG                       = 254,    //out message         debug1,debug2,debug3,debug4
    //reserved in Cleanflight, used in Betaflight and iNav for MSPv2 over MSPv1
    MSP_V2_FRAME                    = 255,    //reserved for system usage

    
    MSP2_COMMON_TZ                  = 0x1001,  //out message   Gets the TZ offset for the local time (returns: minutes(i16))
    MSP2_COMMON_SET_TZ              = 0x1002,  //in message    Sets the TZ offset for the local time (args: minutes(i16))
    MSP2_COMMON_SETTING             = 0x1003,  //in/out message   Returns the value for a setting
    MSP2_COMMON_SET_SETTING         = 0x1004,  //in message    Sets the value for a setting

    MSP2_COMMON_MOTOR_MIXER         = 0x1005,
    MSP2_COMMON_SET_MOTOR_MIXER     = 0x1006,
    
    MSP2_INAV_STATUS                = 0x2000,
    MSP2_INAV_OPTICAL_FLOW          = 0x2001,
    MSP2_INAV_ANALOG                = 0x2002,
    MSP2_INAV_MISC                  = 0x2003,
    MSP2_INAV_SET_MISC              = 0x2004,
    MSP2_INAV_BATTERY_CONFIG        = 0x2005,
    MSP2_INAV_SET_BATTERY_CONFIG    = 0x2006,
    MSP2_INAV_RATE_PROFILE          = 0x2007,
    MSP2_INAV_SET_RATE_PROFILE      = 0x2008,
    MSP2_INAV_AIR_SPEED             = 0x2009
};

enum class ArmingFlags : uint32_t {
    ARMED                                           = (1 << 2),
    WAS_EVER_ARMED                                  = (1 << 3),

    ARMING_DISABLED_FAILSAFE_SYSTEM                 = (1 << 7),

    ARMING_DISABLED_NOT_LEVEL                       = (1 << 8),
    ARMING_DISABLED_SENSORS_CALIBRATING             = (1 << 9),
    ARMING_DISABLED_SYSTEM_OVERLOADED               = (1 << 10),
    ARMING_DISABLED_NAVIGATION_UNSAFE               = (1 << 11),
    ARMING_DISABLED_COMPASS_NOT_CALIBRATED          = (1 << 12),
    ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED    = (1 << 13),
    ARMING_DISABLED_ARM_SWITCH                      = (1 << 14),
    ARMING_DISABLED_HARDWARE_FAILURE                = (1 << 15),
    ARMING_DISABLED_BOXFAILSAFE                     = (1 << 16),
    ARMING_DISABLED_BOXKILLSWITCH                   = (1 << 17),
    ARMING_DISABLED_RC_LINK                         = (1 << 18),
    ARMING_DISABLED_THROTTLE                        = (1 << 19),
    ARMING_DISABLED_CLI                             = (1 << 20),
    ARMING_DISABLED_CMS_MENU                        = (1 << 21),
    ARMING_DISABLED_OSD_MENU                        = (1 << 22),
    ARMING_DISABLED_ROLLPITCH_NOT_CENTERED	        = (1 << 23),
    ARMING_DISABLED_SERVO_AUTOTRIM                  = (1 << 24),
    ARMING_DISABLED_OOM                             = (1 << 25),
    ARMING_DISABLED_INVALID_SETTING                 = (1 << 26),

    ARMING_DISABLED_ALL_FLAGS                       = (ARMING_DISABLED_FAILSAFE_SYSTEM | ARMING_DISABLED_NOT_LEVEL | ARMING_DISABLED_SENSORS_CALIBRATING | ARMING_DISABLED_SYSTEM_OVERLOADED |
                                                       ARMING_DISABLED_NAVIGATION_UNSAFE | ARMING_DISABLED_COMPASS_NOT_CALIBRATED | ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED |
                                                       ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_HARDWARE_FAILURE | ARMING_DISABLED_BOXFAILSAFE | ARMING_DISABLED_BOXKILLSWITCH |
                                                       ARMING_DISABLED_RC_LINK | ARMING_DISABLED_THROTTLE | ARMING_DISABLED_CLI | ARMING_DISABLED_CMS_MENU | ARMING_DISABLED_OSD_MENU |
                                                       ARMING_DISABLED_ROLLPITCH_NOT_CENTERED | ARMING_DISABLED_SERVO_AUTOTRIM | ARMING_DISABLED_OOM | ARMING_DISABLED_INVALID_SETTING)
} ;

std::string armingFlagToString(uint32_t flag)
{
    std::string val;
    if (flag & (1<<2)) val += "ARMED ";
    if (flag & (1<<3)) val += "WAS_EVER_ARMED ";
    if (flag & (1<<7)) val += "ARMING_DISABLED_FAILSAFE_SYSTEM ";
    if (flag & (1<<8)) val += "ARMING_DISABLED_NOT_LEVEL ";
    if (flag & (1<<9)) val += "ARMING_DISABLED_SENSORS_CALIBRATING ";
    if (flag & (1<<10)) val += "ARMING_DISABLED_SYSTEM_OVERLOADED ";
    if (flag & (1<<11)) val += "ARMING_DISABLED_NAVIGATION_UNSAFE ";
    if (flag & (1<<12)) val += "ARMING_DISABLED_COMPASS_NOT_CALIBRATED ";
    if (flag & (1<<13)) val += "ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED ";
    if (flag & (1<<14)) val += "ARMING_DISABLED_ARM_SWITCH ";
    if (flag & (1<<15)) val += "ARMING_DISABLED_HARDWARE_FAILURE ";
    if (flag & (1<<16)) val += "ARMING_DISABLED_BOXFAILSAFE ";
    if (flag & (1<<17)) val += "ARMING_DISABLED_BOXKILLSWITCH ";
    if (flag & (1<<18)) val += "ARMING_DISABLED_RC_LINK ";
    if (flag & (1<<19)) val += "ARMING_DISABLED_THROTTLE ";
    if (flag & (1<<20)) val += "ARMING_DISABLED_CLI ";
    if (flag & (1<<21)) val += "ARMING_DISABLED_CMS_MENU ";
    if (flag & (1<<22)) val += "ARMING_DISABLED_OSD_MENU ";
    if (flag & (1<<23)) val += "ARMING_DISABLED_ROLLPITCH_NOT_CENTERED ";
    if (flag & (1<<24)) val += "ARMING_DISABLED_SERVO_AUTOTRIM ";
    if (flag & (1<<25)) val += "ARMING_DISABLED_OOM ";
    if (flag & (1<<26)) val += "ARMING_DISABLED_INVALID_SETTING ";    
    return val;
}
    
} // namespace msp

std::ostream& operator<<(std::ostream& s, const msp::ID& id) {
    s << (int)id;
    return s;
}

#endif // MSP_ID_HPP
