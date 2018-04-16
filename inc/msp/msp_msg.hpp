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
namespace msg {

const static size_t N_SERVO = 8;
const static size_t N_MOTOR = 8;

const static size_t BOARD_IDENTIFIER_LENGTH = 4;

const static size_t BUILD_DATE_LENGTH = 11;
const static size_t BUILD_TIME_LENGTH = 8;
const static size_t GIT_SHORT_REVISION_LENGTH = 7;

const static size_t MAX_NAME_LENGTH = 16;
const static size_t MAX_MODE_ACTIVATION_CONDITION_COUNT = 20;

const static size_t LED_CONFIGURABLE_COLOR_COUNT = 16;
const static size_t LED_MAX_STRIP_LENGTH = 32;

const static size_t MAX_ADJUSTMENT_RANGE_COUNT = 12;
const static size_t MAX_SIMULTANEOUS_ADJUSTMENT_COUNT = 6;

const static size_t OSD_ITEM_COUNT = 41; //manual count from iNav io/osd.h

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

const static size_t NAUX = 4;

enum class SwitchPosition : size_t {
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

	uint8_t protocol;
	uint8_t major;
	uint8_t minor;

    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(protocol);
        rc &= data.unpack(major);
        rc &= data.unpack(minor);
        return rc;
    }
};

// MSP_FC_VARIANT: 2
struct FcVariant : public Request {
    ID id() const { return ID::MSP_FC_VARIANT; }

    std::string identifier;

    bool decode(ByteVector &data) {
        return data.unpack(identifier,data.size());
    }
};

// MSP_FC_VERSION: 3
struct FcVersion : public Request {
    ID id() const { return ID::MSP_FC_VERSION; }

	uint8_t major;
	uint8_t minor;
	uint8_t patch_level;

    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(major);
        rc &= data.unpack(minor);
        rc &= data.unpack(patch_level);
        return rc;
    }
};

// MSP_BOARD_INFO: 4
struct BoardInfo : public Request {
    ID id() const { return ID::MSP_BOARD_INFO; }

    std::string identifier;
    uint16_t version;
    uint8_t osd_support;
    uint8_t comms_capabilites;
    std::string name;

    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(identifier,BOARD_IDENTIFIER_LENGTH);
        rc &= data.unpack(version);
        rc &= data.unpack(osd_support);
        rc &= data.unpack(comms_capabilites);
        uint8_t name_len;
        rc &= data.unpack(name_len);
        rc &= data.unpack(name,name_len);
        return rc;
    }
};

// MSP_BUILD_INFO: 5
struct BuildInfo : public Request {
    ID id() const { return ID::MSP_BUILD_INFO; }

    std::string buildDate;
    std::string buildTime;
    std::string shortGitRevision;

    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(buildDate,BUILD_DATE_LENGTH);
        rc &= data.unpack(buildTime,BUILD_TIME_LENGTH);
        rc &= data.unpack(shortGitRevision,GIT_SHORT_REVISION_LENGTH);
        return rc;
    }
};

// MSP_NAME: 10
struct BoardName : public Request {
    ID id() const { return ID::MSP_NAME; }
    
    std::string name;
    
    bool decode(ByteVector & data) {
        return data.unpack(name,data.size());
    }
};


// MSP_SET_NAME: 11
struct SetBoardName : public Response {
    ID id() const { return ID::MSP_SET_NAME; }
    
    std::string name;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(name,MAX_NAME_LENGTH);
        return data;
    }
};


// MSP_NAV_POSHOLD: 12
struct NavPosHold : public Request {
    ID id() const { return ID::MSP_NAV_POSHOLD; }
    
    uint8_t user_control_mode;
    uint16_t max_auto_speed;
    uint16_t max_auto_climb_rate;
    uint16_t max_manual_speed;
    uint16_t max_manual_climb_rate;
    uint8_t max_bank_angle;
    uint8_t use_thr_mid_for_althold;
    uint16_t hover_throttle;

    bool decode(ByteVector &data) {
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
struct SetNavPosHold : public Response {
    ID id() const { return ID::MSP_SET_NAV_POSHOLD; }
    
    uint8_t user_control_mode;
    uint16_t max_auto_speed;
    uint16_t max_auto_climb_rate;
    uint16_t max_manual_speed;
    uint16_t max_manual_climb_rate;
    uint8_t max_bank_angle;
    uint8_t use_thr_mid_for_althold;
    uint16_t hover_throttle;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(user_control_mode);
        data.pack(max_auto_speed);
        data.pack(max_auto_climb_rate);
        data.pack(max_manual_speed);
        data.pack(max_manual_climb_rate);
        data.pack(max_bank_angle);
        data.pack(use_thr_mid_for_althold);
        data.pack(hover_throttle);
        return data;
    }
};

        
// MSP_CALIBRATION_DATA: 14
struct CalibrationData : public Request {
    ID id() const { return ID::MSP_CALIBRATION_DATA; }
    
    uint8_t axis_flags;
    uint16_t acc_zero_x;
    uint16_t acc_zero_y;
    uint16_t acc_zero_z;
    uint16_t acc_gain_x;
    uint16_t acc_gain_y;
    uint16_t acc_gain_z;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(axis_flags);
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
struct SetCalibrationData : public Response {
    ID id() const { return ID::MSP_SET_CALIBRATION_DATA; }
    
    uint16_t acc_zero_x;
    uint16_t acc_zero_y;
    uint16_t acc_zero_z;
    uint16_t acc_gain_x;
    uint16_t acc_gain_y;
    uint16_t acc_gain_z;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(acc_zero_x);
        data.pack(acc_zero_y);
        data.pack(acc_zero_z);
        data.pack(acc_gain_x);
        data.pack(acc_gain_y);
        data.pack(acc_gain_z);
        return data;
    }
};
        
// MSP_POSITION_ESTIMATION_CONFIG: 16
struct PositionEstimationConfig : public Request {
    ID id() const { return ID::MSP_POSITION_ESTIMATION_CONFIG; }
    
    float w_z_baro_p;
    float w_z_gps_p;
    float w_z_gps_v;
    float w_xy_gps_p;
    float w_xy_gps_v;
    uint8_t gps_min_sats;
    bool use_gps_vel_NED;

    bool decode(ByteVector &data) {
        bool rc = true;
        uint16_t raw;
        rc &= data.unpack(raw);
        w_z_baro_p = raw/100.0;
        rc &= data.unpack(raw);
        w_z_gps_p = raw/100.0;
        rc &= data.unpack(raw);
        w_z_gps_v = raw/100.0;
        rc &= data.unpack(raw);
        w_xy_gps_p = raw/100.0;
        rc &= data.unpack(raw);
        w_xy_gps_v = raw/100.0;
        rc &= data.unpack(gps_min_sats);
        rc &= data.unpack(use_gps_vel_NED);
    }
};

// MSP_SET_POSITION_ESTIMATION_CONFIG: 17
struct SetPositionEstimationConfig : public Response {
    ID id() const { return ID::MSP_SET_POSITION_ESTIMATION_CONFIG; }
    
    float w_z_baro_p;
    float w_z_gps_p;
    float w_z_gps_v;
    float w_xy_gps_p;
    float w_xy_gps_v;
    uint8_t gps_min_sats;
    bool use_gps_vel_NED;
    
    ByteVector encode() const {
        ByteVector data;
        uint16_t val;
        val = static_cast<uint16_t>(w_z_baro_p*100);
        data.pack(val);
        val = static_cast<uint16_t>(w_z_gps_p*100);
        data.pack(val);
        val = static_cast<uint16_t>(w_z_gps_v*100);
        data.pack(val);
        val = static_cast<uint16_t>(w_xy_gps_p*100);
        data.pack(val);
        val = static_cast<uint16_t>(w_xy_gps_v*100);
        data.pack(val);
        data.pack(gps_min_sats);
        data.pack(use_gps_vel_NED);
        return data;
    }
};
    
// MSP_WP_MISSION_LOAD: 18
struct WpMissionLoad : public Response {
    ID id() const { return ID::MSP_WP_MISSION_LOAD; }
    
    ByteVector encode() const {
        return ByteVector(1,0);
    }
};
    
// MSP_WP_MISSION_SAVE: 19
struct WpMissionSave : public Response {
    ID id() const { return ID::MSP_WP_MISSION_SAVE; }
    
    ByteVector encode() const {
        return ByteVector(1,0);
    }
};

// MSP_WP_GETINFO: 20
struct WpGetInfo : public Request {
    ID id() const { return ID::MSP_WP_GETINFO; }
    
    uint8_t wp_capabilites;
    uint8_t max_waypoints;
    bool wp_list_valid;
    uint8_t wp_count;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(wp_capabilites);
        rc &= data.unpack(max_waypoints);
        rc &= data.unpack(wp_list_valid);
        rc &= data.unpack(wp_count);
        return rc;
    }
};

// MSP_RTH_AND_LAND_CONFIG: 21
struct RthAndLandConfig : public Request {
    ID id() const { return ID::MSP_RTH_AND_LAND_CONFIG; }
    
    uint16_t min_rth_distance;
    uint8_t rth_climb_first;
    uint8_t rth_climb_ignore_emerg;
    uint8_t rth_tail_first;
    uint8_t rth_allow_landing;
    uint8_t rth_alt_control_mode;
    uint16_t rth_abort_threshold;
    uint16_t rth_altitude;
    uint16_t land_descent_rate;
    uint16_t land_slowdown_minalt;
    uint16_t land_slowdown_maxalt;
    uint16_t emerg_descent_rate;
    
    bool decode(ByteVector &data) {
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
struct SetRthAndLandConfig : public Response {
    ID id() const { return ID::MSP_SET_RTH_AND_LAND_CONFIG; }
    
    uint16_t min_rth_distance;
    uint8_t rth_climb_first;
    uint8_t rth_climb_ignore_emerg;
    uint8_t rth_tail_first;
    uint8_t rth_allow_landing;
    uint8_t rth_alt_control_mode;
    uint16_t rth_abort_threshold;
    uint16_t rth_altitude;
    uint16_t land_descent_rate;
    uint16_t land_slowdown_minalt;
    uint16_t land_slowdown_maxalt;
    uint16_t emerg_descent_rate;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(min_rth_distance);
        data.pack(rth_climb_first);
        data.pack(rth_climb_ignore_emerg);
        data.pack(rth_tail_first);
        data.pack(rth_allow_landing);
        data.pack(rth_alt_control_mode);
        data.pack(rth_abort_threshold);
        data.pack(rth_altitude);
        data.pack(land_descent_rate);
        data.pack(land_slowdown_minalt);
        data.pack(land_slowdown_maxalt);
        data.pack(emerg_descent_rate);
        return data;
    }
};

// MSP_FW_CONFIG: 23
struct FwConfig : public Request {
    ID id() const { return ID::MSP_FW_CONFIG; }
    
    uint16_t cruise_throttle;
    uint16_t min_throttle;
    uint16_t max_throttle;
    uint8_t max_bank_angle;
    uint8_t max_climb_angle;
    uint8_t max_dive_angle;
    uint8_t pitch_to_throttle;
    uint16_t loiter_radius;
    
    bool decode(ByteVector &data) {
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
struct SetFwConfig : public Response {
    ID id() const { return ID::MSP_SET_FW_CONFIG; }

    uint16_t cruise_throttle;
    uint16_t min_throttle;
    uint16_t max_throttle;
    uint8_t max_bank_angle;
    uint8_t max_climb_angle;
    uint8_t max_dive_angle;
    uint8_t pitch_to_throttle;
    uint16_t loiter_radius;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(cruise_throttle);
        data.pack(min_throttle);
        data.pack(max_throttle);
        data.pack(max_bank_angle);
        data.pack(max_climb_angle);
        data.pack(max_dive_angle);
        data.pack(pitch_to_throttle);
        data.pack(loiter_radius);
        return data;
    }
};

struct box_description
{
    uint8_t id;
    uint8_t aux_channel_index;
    uint8_t startStep;
    uint8_t endStep;
};

// MSP_MODE_RANGES: 34
struct ModeRange : public Request {
    ID id() const { return ID::MSP_MODE_RANGES; }
    
    std::array<box_description,MAX_MODE_ACTIVATION_CONDITION_COUNT> boxes;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        for (uint i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            rc &= data.unpack(boxes[i].id);
            rc &= data.unpack(boxes[i].aux_channel_index);
            rc &= data.unpack(boxes[i].startStep);
            rc &= data.unpack(boxes[i].endStep);
        }
        return rc;
    }
};


// MSP_SET_MODE_RANGE: 35
struct SetModeRange : public Response {
    ID id() const { return ID::MSP_SET_MODE_RANGE; }
    
    uint8_t mode_activation_condition;
    box_description box;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(mode_activation_condition);
        data.pack(box.id);
        data.pack(box.aux_channel_index);
        data.pack(box.startStep);
        data.pack(box.endStep);
        return data;
    }
};



// MSP_FEATURE: 36
struct Feature : public Request {
    ID id() const { return ID::MSP_FEATURE; }

    std::set<std::string> features;
    
    bool decode(ByteVector &data) {
        uint32_t mask;
        bool rc = data.unpack(mask);
        if (!rc) return rc;
        for(size_t ifeat(0); ifeat<FEATURES.size(); ifeat++) {
            if(mask & (1<<ifeat))
                features.insert(FEATURES[ifeat]);
        }
    }
};

// MSP_SET_FEATURE: 37
struct SetFeature : public Response {
    ID id() const { return ID::MSP_SET_FEATURE; }

    std::set<std::string> features;

    ByteVector encode() const {
        ByteVector data;
        uint32_t mask = 0;
        for(size_t ifeat(0); ifeat<FEATURES.size(); ifeat++) {
            if(features.count(FEATURES[ifeat]))
                mask |= 1<<ifeat;
        }
        data.pack(mask);
        return data;
    }
};


// MSP_BOARD_ALIGNMENT: 38
struct BoardAlignment : public Request {
    ID id() const { return ID::MSP_BOARD_ALIGNMENT; }

    uint16_t rollDeciDegrees;
    uint16_t pitchDeciDegrees;
    uint16_t yawDeciDegrees;

    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(rollDeciDegrees);
        rc &= data.unpack(pitchDeciDegrees);
        rc &= data.unpack(yawDeciDegrees);
        return rc;
    }
};


// MSP_SET_BOARD_ALIGNMENT: 39
struct SetBoardAlignment : public Response {
    ID id() const { return ID::MSP_BOARD_ALIGNMENT; }

    uint16_t rollDeciDegrees;
    uint16_t pitchDeciDegrees;
    uint16_t yawDeciDegrees;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(rollDeciDegrees);
        data.pack(pitchDeciDegrees);
        data.pack(yawDeciDegrees);
        return data;
    }
};


// MSP_AMPERAGE_METER_CONFIG: 40 (AKA MSP_CURRENT_METER_CONFIG)
struct CurrentMeterConfig : public Request {
    ID id() const { return ID::MSP_AMPERAGE_METER_CONFIG; }

    uint16_t currnet_scale;
    uint16_t current_offset;
    uint8_t current_type;
    uint16_t capacity;

    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(currnet_scale);
        rc &= data.unpack(current_offset);
        rc &= data.unpack(current_type);
        rc &= data.unpack(capacity);
        return rc;
    }
};


// MSP_SET_AMPERAGE_METER_CONFIG: 41 (AKA MSP_SET_CURRENT_METER_CONFIG)
struct SetCurrentMeterConfig : public Response {
    ID id() const { return ID::MSP_SET_AMPERAGE_METER_CONFIG; }

    uint16_t currnet_scale;
    uint16_t current_offset;
    uint8_t current_type;
    uint16_t capacity;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(currnet_scale);
        data.pack(current_offset);
        data.pack(current_type);
        data.pack(capacity);
        return data;
    }
};


// MSP_MIXER: 42
struct Mixer : public Request {
    ID id() const { return ID::MSP_MIXER; }

    uint8_t mode;

    bool decode(ByteVector &data) {
        return data.unpack(mode);
    }
};



// MSP_SET_MIXER: 43
struct SetMixer : public Response {
    ID id() const { return ID::MSP_SET_MIXER; }

    uint8_t mode;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(mode);
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

    bool decode(ByteVector &data) {
        serialrx_provider = data[0];
        maxcheck = deserialise_uint16(data, 1);
        midrc = deserialise_uint16(data, 3);
        mincheck = deserialise_uint16(data, 5);
        spektrum_sat_bind = data[7];
        rx_min_usec = deserialise_uint16(data, 8);
        rx_max_usec = deserialise_uint16(data, 10);
    }
};


// MSP_SET_RX_CONFIG: 45
struct SetRxConfig : public Response {
    ID id() const { return ID::MSP_SET_RX_CONFIG; }

    uint8_t serialrx_provider;
    uint16_t maxcheck;
    uint16_t midrc;
    uint16_t mincheck;
    uint8_t spektrum_sat_bind;
    uint16_t rx_min_usec;
    uint16_t rx_max_usec;
    uint8_t rcInterpolation;
    uint8_t rcInterpolationInterval;
    uint16_t airModeActivateThreshold;
    uint8_t rx_spi_protocol;
    uint32_t rx_spi_id;
    uint8_t rx_spi_rf_channel_count;
    uint8_t fpvCamAngleDegrees;
    uint8_t receiverType;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(serialrx_provider);
        data.pack(maxcheck);
        data.pack(midrc);
        data.pack(mincheck);
        data.pack(spektrum_sat_bind);
        data.pack(rx_min_usec);
        data.pack(rx_max_usec);
        data.pack(rcInterpolation);
        data.pack(rcInterpolationInterval);
        data.pack(airModeActivateThreshold);
        data.pack(rx_spi_protocol);
        data.pack(rx_spi_id);
        data.pack(rx_spi_rf_channel_count);
        data.pack(fpvCamAngleDegrees);
        data.pack(receiverType);
        return data;
    }
};

struct HsvColor {
    uint16_t h;
    uint8_t s;
    uint8_t v;
};

// MSP_LED_COLORS: 46
struct LedColors : public Request {
    ID id() const { return ID::MSP_LED_COLORS; }

    std::array<HsvColor,LED_CONFIGURABLE_COLOR_COUNT> colors;

    bool decode(ByteVector &data) {
        bool rc = true;
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            rc &= data.unpack(colors[i].h);
            rc &= data.unpack(colors[i].s);
            rc &= data.unpack(colors[i].v);
        }
        return rc;
    }
};


// MSP_SET_LED_COLORS: 47
struct SetLedColors : public Response {
    ID id() const { return ID::MSP_SET_LED_COLORS; }

    std::array<HsvColor,LED_CONFIGURABLE_COLOR_COUNT> colors;

    ByteVector encode() const {
        ByteVector data;
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            data.pack(colors[i].h);
            data.pack(colors[i].s);
            data.pack(colors[i].v);
        }
        return data;
    }
};

// MSP_LED_STRIP_CONFIG: 48
struct LedStripConfig : public Request {
    ID id() const { return ID::MSP_LED_STRIP_CONFIG; }

    std::array<uint32_t,LED_MAX_STRIP_LENGTH> configs;

    bool decode(ByteVector &data) {
        bool rc = true;
        for (int i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
            rc &= data.unpack(configs[i]);
        }
        return rc;
    }
};

// MSP_SET_LED_STRIP_CONFIG: 49
struct SetLedStripConfig : public Response {
    ID id() const { return ID::MSP_SET_LED_STRIP_CONFIG; }

    uint8_t cfg_index;
    uint32_t config;

    ByteVector encode() const {
        ByteVector data;
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            data.pack(cfg_index);
            data.pack(config);
        }
        return data;
    }
};

// MSP_RSSI_CONFIG: 50
struct RssiConfig : public Request {
    ID id() const { return ID::MSP_RSSI_CONFIG; }

    uint8_t rssi_channel;

    bool decode(ByteVector &data) {
        return data.unpack(rssi_channel);
    }
};

// MSP_SET_RSSI_CONFIG: 51
struct SetRssiConfig : public Response {
    ID id() const { return ID::MSP_SET_RSSI_CONFIG; }

    uint8_t rssi_channel;

    ByteVector encode() const {
        ByteVector data;
        data.pack(rssi_channel);
        return data;
    }
};

struct adjustmentRange {
    uint8_t adjustmentIndex;
    uint8_t auxChannelIndex;
    uint8_t range_startStep;
    uint8_t range_endStep;
    uint8_t adjustmentFunction;
    uint8_t auxSwitchChannelIndex;
};

// MSP_ADJUSTMENT_RANGES: 52
struct AdjustmentRanges : public Request {
    ID id() const { return ID::MSP_ADJUSTMENT_RANGES; }

    std::array<adjustmentRange,MAX_ADJUSTMENT_RANGE_COUNT> ranges;

    bool decode(ByteVector &data) {
        bool rc = true;
        for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
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
struct SetAdjustmentRanges : public Response {
    ID id() const { return ID::MSP_SET_ADJUSTMENT_RANGE; }

    uint8_t range_index;
    adjustmentRange range;

    ByteVector encode() const {
        ByteVector data;
        data.pack(range_index);
        data.pack(range.adjustmentIndex);
        data.pack(range.auxChannelIndex);
        data.pack(range.range_startStep);
        data.pack(range.range_endStep);
        data.pack(range.adjustmentFunction);
        data.pack(range.auxSwitchChannelIndex);
        return data;
    }
};

// MSP_CF_SERIAL_CONFIG: 54
// MSP_SET_CF_SERIAL_CONFIG: 55

// MSP_VOLTAGE_METER_CONFIG: 56 (differs from BetaFlight)
struct VoltageMeterConfig : public Request {
    ID id() const { return ID::MSP_VOLTAGE_METER_CONFIG; }

    uint8_t scale_dV;
    uint8_t cell_min_dV;
    uint8_t cell_max_dV;
    uint8_t cell_warning_dV;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(scale_dV);
        rc &= data.unpack(cell_min_dV);
        rc &= data.unpack(cell_max_dV);
        rc &= data.unpack(cell_warning_dV);
        return rc;
    }
};



// MSP_SET_VOLTAGE_METER_CONFIG: 57 (differs from BetaFlight)
struct SetVoltageMeterConfig : public Response {
    ID id() const { return ID::MSP_SET_VOLTAGE_METER_CONFIG; }

    uint8_t scale_dV;
    uint8_t cell_min_dV;
    uint8_t cell_max_dV;
    uint8_t cell_warning_dV;

    ByteVector encode() const {
        ByteVector data;
        data.pack(scale_dV);
        data.pack(cell_min_dV);
        data.pack(cell_max_dV);
        data.pack(cell_warning_dV);
        return data;
    }
};


// MSP_SONAR_ALTITUDE: 58
struct SonarAltitude : public Request {
    ID id() const { return ID::MSP_SONAR_ALTITUDE; }

    float altitude; // meters

    bool decode(ByteVector &data) {
        int32_t cm;
        bool rc = data.unpack(cm);
        altitude = cm/100.0f;
        return rc;
    }
};

//MSP_ARMING_CONFIG: 61
struct ArmingConfig : public Request {
    ID id() const { return ID::MSP_ARMING_CONFIG; }

    uint8_t auto_disarm_delay;
    uint8_t disarm_kill_switch;

    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(auto_disarm_delay);
        rc &= data.unpack(disarm_kill_switch);
        return rc;
    }
};

//MSP_SET_ARMING_CONFIG: 62
struct SetArmingConfig : public Response {
    ID id() const { return ID::MSP_SET_ARMING_CONFIG; }

    uint8_t auto_disarm_delay;
    uint8_t disarm_kill_switch;

    ByteVector encode() const {
        ByteVector data;
        data.pack(auto_disarm_delay);
        data.pack(disarm_kill_switch);
        return data;
    }
};

// MSP_RX_MAP: 64
struct RxMap : public Request {
    ID id() const { return ID::MSP_RX_MAP; }

    ByteVector map;

    bool decode(ByteVector &data) {
        map = data;
    }
};

// MSP_SET_RX_MAP: 65
struct SetRxMap : public Response {
    ID id() const { return ID::MSP_SET_RX_MAP; }

    ByteVector map;

    ByteVector encode() const {
        return map;
    }
};

// MSP_REBOOT: 68
struct Reboot : public Response {
    ID id() const { return ID::MSP_REBOOT; }
    
    ByteVector encode() const {
        return ByteVector();
    }
};


// MSP_BF_BUILD_INFO: 69
struct BfBuildInfo : public Request {
    ID id() const { return ID::MSP_BF_BUILD_INFO; }

    std::string build_date;
    uint32_t reserved1;
    uint32_t reserved2;

    bool decode(ByteVector &data) {
        data.unpack(build_date,11);
        data.unpack(reserved1);
        data.unpack(reserved2);
    }
};

// MSP_DATAFLASH_SUMMARY: 70
struct DataflashSummary : public Request {
    ID id() const { return ID::MSP_DATAFLASH_SUMMARY; }

    bool flash_is_ready;
    uint32_t sectors;
    uint32_t total_size;
    uint32_t offset;

    bool decode(ByteVector &data) {
        data.unpack(flash_is_ready);
        data.unpack(sectors);
        data.unpack(total_size);
        data.unpack(offset);
    }
};

// MSP_DATAFLASH_READ: 71
// This seems like a special command that actually encodes data into the 
// outbound buffer. I'm too lazy to do this now. TODO

// MSP_DATAFLASH_ERASE: 72
struct DataflashErase : public Request {
    ID id() const { return ID::MSP_DATAFLASH_ERASE; }

    bool decode(ByteVector &data) {
        return true;
    }
};

// MSP_LOOP_TIME: 73
struct LoopTime : public Request {
    ID id() const { return ID::MSP_LOOP_TIME; }

    uint16_t loop_time;
    
    bool decode(ByteVector &data) {
        return data.unpack(loop_time);
    }
};

// MSP_SET_LOOP_TIME:74
struct SetLoopTime : public Response {
    ID id() const { return ID::MSP_SET_LOOP_TIME; }
    
    uint16_t loop_time;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(loop_time);
        return data;
    }
};

struct Failsafe {
    uint8_t delay;
    uint8_t off_delay;
    uint16_t throttle;
    uint8_t kill_switch;
    uint16_t throttle_low_delay;
    uint8_t procedure;
    uint8_t recovery_delay;
    uint16_t fw_roll_angle;
    uint16_t fw_pitch_angle;
    uint16_t fw_yaw_rate;
    uint16_t stick_motion_threshold;
    uint16_t min_distance;
    uint8_t min_distance_procedure;
};

// MSP_FAILSAFE_CONFIG: 75
struct FailsafeConfig : public Request {
    ID id() const { return ID::MSP_FAILSAFE_CONFIG; }

    Failsafe failsafe;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(failsafe.delay);
        rc &= data.unpack(failsafe.off_delay);
        rc &= data.unpack(failsafe.throttle);
        rc &= data.unpack(failsafe.kill_switch);
        rc &= data.unpack(failsafe.throttle_low_delay);
        rc &= data.unpack(failsafe.procedure);
        rc &= data.unpack(failsafe.recovery_delay);
        rc &= data.unpack(failsafe.fw_roll_angle);
        rc &= data.unpack(failsafe.fw_pitch_angle);
        rc &= data.unpack(failsafe.fw_yaw_rate);
        rc &= data.unpack(failsafe.stick_motion_threshold);
        rc &= data.unpack(failsafe.min_distance);
        rc &= data.unpack(failsafe.min_distance_procedure);
        return rc;
    }
};

// MSP_SET_FAILSAFE_CONFIG: 76
struct SetFailsafeConfig : public Response {
    ID id() const { return ID::MSP_SET_FAILSAFE_CONFIG; }
    
    Failsafe failsafe;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(failsafe.delay);
        data.pack(failsafe.off_delay);
        data.pack(failsafe.throttle);
        data.pack(failsafe.kill_switch);
        data.pack(failsafe.throttle_low_delay);
        data.pack(failsafe.procedure);
        data.pack(failsafe.recovery_delay);
        data.pack(failsafe.fw_roll_angle);
        data.pack(failsafe.fw_pitch_angle);
        data.pack(failsafe.fw_yaw_rate);
        data.pack(failsafe.stick_motion_threshold);
        data.pack(failsafe.min_distance);
        data.pack(failsafe.min_distance_procedure);
        return data;
    }
};

//Depricated
// MSP_RXFAIL_CONFIG: 77
// MSP_SET_RXFAIL_CONFIG: 78

// MSP_SDCARD_SUMMARY: 79
struct SdcardSummary : public Request {
    ID id() const { return ID::MSP_SDCARD_SUMMARY; }

    uint8_t flags;
    uint8_t state;
    uint8_t last_error;
    uint32_t free_space_kb;
    uint32_t total_space_kb;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(flags);
        rc &= data.unpack(state);
        rc &= data.unpack(last_error);
        rc &= data.unpack(free_space_kb);
        rc &= data.unpack(total_space_kb);
        return rc;
    }
};

// MSP_BLACKBOX_CONFIG: 80
struct BlackboxConfig : public Request {
    ID id() const { return ID::MSP_BLACKBOX_CONFIG; }

    uint8_t supported;
    uint8_t device;
    uint8_t rate_num;
    uint8_t rate_denom;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(supported);
        rc &= data.unpack(device);
        rc &= data.unpack(rate_num);
        rc &= data.unpack(rate_denom);
        return rc;
    }
};

// MSP_SET_BLACKBOX_CONFIG: 81
struct SetBlackboxConfig : public Response {
    ID id() const { return ID::MSP_SET_BLACKBOX_CONFIG; }
    
    uint8_t device;
    uint8_t rate_num;
    uint8_t rate_denom;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(device);
        data.pack(rate_num);
        data.pack(rate_denom);
        return data;
    }
};

// No reference available TODO
// MSP_TRANSPONDER_CONFIG: 82
// MSP_SET_TRANSPONDER_CONFIG: 83

// MSP_OSD_CONFIG: 84
struct OsdConfig : public Request {
    ID id() const { return ID::MSP_OSD_CONFIG; }

    uint8_t supported;
    uint8_t video_system;
    uint8_t units;
    uint8_t rssi_alarm;
    uint16_t battery_cap_warn;
    uint16_t time_alarm;
    uint16_t alt_alarm;
    uint16_t dist_alarm;
    uint16_t neg_alt_alarm;
    std::array<uint16_t,OSD_ITEM_COUNT> items;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(supported);
        if (rc && supported) {
            rc &= data.unpack(video_system);
            rc &= data.unpack(units);
            rc &= data.unpack(rssi_alarm);
            rc &= data.unpack(battery_cap_warn);
            rc &= data.unpack(time_alarm);
            rc &= data.unpack(alt_alarm);
            rc &= data.unpack(dist_alarm);
            rc &= data.unpack(neg_alt_alarm);
            for (int i = 0; i < OSD_ITEM_COUNT; i++) {
                rc &= data.unpack(items[i]);
            }
        }
        return rc;
    }
};

// MSP_SET_OSD_CONFIG: 85
struct SetOsdConfig : public Response {
    ID id() const { return ID::MSP_SET_OSD_CONFIG; }
    
    int8_t param_idx;
    uint16_t param_val;
    uint8_t video_system;
    uint8_t units;
    uint8_t rssi_alarm;
    uint16_t battery_cap_warn;
    uint16_t time_alarm;
    uint16_t alt_alarm;
    uint16_t dist_alarm;
    uint16_t neg_alt_alarm;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(param_idx);
        if (param_idx == -1) {
            data.pack(video_system);
            data.pack(units);
            data.pack(rssi_alarm);
            data.pack(battery_cap_warn);
            data.pack(time_alarm);
            data.pack(alt_alarm);
            data.pack(dist_alarm);
            data.pack(neg_alt_alarm);
        } else {
            data.pack(param_val);
        }
        return data;
    }
};

// MSP_OSD_CHAR_READ: 86 No reference implementation


// MSP_OSD_CHAR_WRITE: 87
struct OsdCharWrite : public Response {
    ID id() const { return ID::MSP_OSD_CHAR_WRITE; }
    
    uint8_t addr;
    std::array<uint8_t,54> data;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(addr);
        for (auto c : data) {
            data.pack(c);
        }
        return data;
    }
};


// MSP_VTX_CONFIG: 88
struct VtxConfig : public Request {
    ID id() const { return ID::MSP_VTX_CONFIG; }

    uint8_t device_type;
    uint8_t band;
    uint8_t channel;
    uint8_t power_idx;
    uint8_t pit_mode;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(device_type);
        if (device_type != 0xFF) {
            rc &= data.unpack(band);
            rc &= data.unpack(channel);
            rc &= data.unpack(power_idx);
            rc &= data.unpack(pit_mode);
        }
        return rc;
    }
};

// MSP_SET_VTX_CONFIG: 89
struct SetVtxConfig : public Response {
    ID id() const { return ID::MSP_SET_VTX_CONFIG; }
    
    uint8_t band;
    uint8_t channel;
    uint8_t power;
    uint8_t pit_mode;
    
    
    ByteVector encode() const {
        ByteVector data;
        uint16_t tmp = (band-1)*8 + (channel-1);
        data.pack(tmp);
        data.pack(power);
        data.pack(pit_mode);
        return data;
    }
};




// Betaflight Additional Commands
// MSP_ADVANCED_CONFIG: 90
struct AdvancedConfig : public Request {
    ID id() const { return ID::MSP_ADVANCED_CONFIG; }

    uint8_t gyro_sync_denom;
    uint8_t pid_process_denom;
    uint8_t use_unsynced_pwm;
    uint8_t pwm_protocol;
    uint16_t motor_pwm_rate;
    uint16_t servo_pwm_rate;
    uint8_t gyro_sync;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(gyro_sync_denom);
        rc &= data.unpack(pid_process_denom);
        rc &= data.unpack(use_unsynced_pwm);
        rc &= data.unpack(pwm_protocol);
        rc &= data.unpack(motor_pwm_rate);
        rc &= data.unpack(servo_pwm_rate);
        rc &= data.unpack(gyro_sync);
        return rc;
    }
};


// MSP_SET_ADVANCED_CONFIG: 91
struct SetAdvancedConfig : public Response {
    ID id() const { return ID::MSP_SET_ADVANCED_CONFIG; }
    
    uint8_t gyro_sync_denom;
    uint8_t pid_process_denom;
    uint8_t use_unsynced_pwm;
    uint8_t pwm_protocol;
    uint16_t motor_pwm_rate;
    uint16_t servo_pwm_rate;
    uint8_t gyro_sync;
    
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(gyro_sync_denom);
        data.pack(pid_process_denom);
        data.pack(use_unsynced_pwm);
        data.pack(pwm_protocol);
        data.pack(motor_pwm_rate);
        data.pack(servo_pwm_rate);
        data.pack(gyro_sync);
        return data;
    }
};


// MSP_FILTER_CONFIG: 92
struct FilterConfig : public Request {
    ID id() const { return ID::MSP_FILTER_CONFIG; }

    uint8_t gyro_soft_lpf_hz;
    uint16_t dterm_lpf_hz;
    uint16_t yaw_lpf_hz;
    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t dterm_soft_notch_hz;
    uint16_t dterm_soft_notch_cutoff;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(gyro_soft_lpf_hz);
        rc &= data.unpack(dterm_lpf_hz);
        rc &= data.unpack(yaw_lpf_hz);
        rc &= data.unpack(gyro_soft_notch_hz_1);
        rc &= data.unpack(gyro_soft_notch_cutoff_1);
        rc &= data.unpack(dterm_soft_notch_hz);
        rc &= data.unpack(dterm_soft_notch_cutoff);
        rc &= data.unpack(gyro_soft_notch_hz_2);
        rc &= data.unpack(gyro_soft_notch_cutoff_2);
        return rc;
    }
};



// MSP_SET_FILTER_CONFIG: 93 TODO this is an unstable message, decoding depends on the FC build flags
struct SetFilterConfig : public Response {
    ID id() const { return ID::MSP_SET_FILTER_CONFIG; }
    
    uint8_t gyro_soft_lpf_hz;
    uint16_t dterm_lpf_hz;
    uint16_t yaw_lpf_hz;
    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t dterm_soft_notch_hz;
    uint16_t dterm_soft_notch_cutoff;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(gyro_soft_lpf_hz);
        data.pack(dterm_lpf_hz);
        data.pack(yaw_lpf_hz);
        data.pack(gyro_soft_notch_hz_1);
        data.pack(gyro_soft_notch_cutoff_1);
        data.pack(dterm_soft_notch_hz);
        data.pack(dterm_soft_notch_cutoff);
        data.pack(gyro_soft_notch_hz_2);
        data.pack(gyro_soft_notch_cutoff_2);
        return data;
    }
};


// MSP_PID_ADVANCED: 94
struct PidAdvanced : public Request {
    ID id() const { return ID::MSP_PID_ADVANCED; }

    uint16_t rollPitchItermIgnoreRate;
    uint16_t yawItermIgnoreRate;
    uint16_t yaw_p_limit;
    uint8_t deltaMethod;
    uint8_t vbatPidCompensation;
    uint8_t setpointRelaxRatio;
    uint8_t dterm_setpoint_weight; //TODO scaled value
    uint16_t pidSumLimit;
    uint8_t itermThrottleGain;
    uint16_t axisAccelerationLimitRollPitch; //TODO scaled and clamped value
    uint16_t axisAccelerationLimitYaw; //TODO scaled and clamped value
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(rollPitchItermIgnoreRate);
        rc &= data.unpack(yawItermIgnoreRate);
        rc &= data.unpack(yaw_p_limit);
        rc &= data.unpack(deltaMethod);
        rc &= data.unpack(vbatPidCompensation);
        rc &= data.unpack(setpointRelaxRatio);
        rc &= data.unpack(dterm_setpoint_weight);
        rc &= data.unpack(pidSumLimit);
        rc &= data.unpack(itermThrottleGain);
        rc &= data.unpack(axisAccelerationLimitRollPitch);
        rc &= data.unpack(axisAccelerationLimitYaw);
        return rc;
    }
};


// MSP_SET_PID_ADVANCED: 95
struct SetPidAdvanced : public Response {
    ID id() const { return ID::MSP_SET_PID_ADVANCED; }
    
    uint16_t rollPitchItermIgnoreRate;
    uint16_t yawItermIgnoreRate;
    uint16_t yaw_p_limit;
    uint8_t deltaMethod;
    uint8_t vbatPidCompensation;
    uint8_t setpointRelaxRatio;
    uint8_t dterm_setpoint_weight; //TODO scaled value
    uint16_t pidSumLimit;
    uint8_t itermThrottleGain;
    uint16_t axisAccelerationLimitRollPitch; //TODO scaled and clamped value
    uint16_t axisAccelerationLimitYaw; //TODO scaled and clamped value
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(rollPitchItermIgnoreRate);
        data.pack(yawItermIgnoreRate);
        data.pack(yaw_p_limit);
        data.pack(deltaMethod);
        data.pack(vbatPidCompensation);
        data.pack(setpointRelaxRatio);
        data.pack(dterm_setpoint_weight);
        data.pack(pidSumLimit);
        data.pack(itermThrottleGain);
        data.pack(axisAccelerationLimitRollPitch);
        data.pack(axisAccelerationLimitYaw);
        return data;
    }
};


// MSP_SENSOR_CONFIG: 96
struct SensorConfig : public Request {
    ID id() const { return ID::MSP_SENSOR_CONFIG; }

    uint8_t acc_hardware;
    uint8_t baro_hardware;
    uint8_t mag_hardware;
    uint8_t pitot_hardware;
    uint8_t rangefinder_hardware;
    uint8_t opflow_hardware;
    
    bool decode(ByteVector &data) {
        bool rc = true;
        rc &= data.unpack(acc_hardware);
        rc &= data.unpack(baro_hardware);
        rc &= data.unpack(mag_hardware);
        rc &= data.unpack(pitot_hardware);
        rc &= data.unpack(rangefinder_hardware);
        rc &= data.unpack(opflow_hardware);
        return rc;
    }
};


// MSP_SET_SENSOR_CONFIG: 97
struct SetSensorConfig : public Response {
    ID id() const { return ID::MSP_SET_SENSOR_CONFIG; }
    
    uint8_t acc_hardware;
    uint8_t baro_hardware;
    uint8_t mag_hardware;
    uint8_t pitot_hardware;
    uint8_t rangefinder_hardware;
    uint8_t opflow_hardware;
    
    ByteVector encode() const {
        ByteVector data;
        data.pack(acc_hardware);
        data.pack(baro_hardware);
        data.pack(mag_hardware);
        data.pack(pitot_hardware);
        data.pack(rangefinder_hardware);
        data.pack(opflow_hardware);
        return data;
    }
};


// MSP_SPECIAL_PARAMETERS: 98 // Temporary betaflight parameters before cleanup and k$
// MSP_SET_SPECIAL_PARAMETERS: 99 // Temporary betaflight parameters before cleanup and k$


/////////////////////////////////////////////////////////////////////
/// Requests (1xx)

// MSP_IDENT: 100
struct Ident : public Request {
    ID id() const { return ID::MSP_IDENT; }

	size_t version;
    MultiType type;
	size_t msp_version;
    std::set<Capability> capabilities;

    bool decode(ByteVector &data) {
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
            
        return true;
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
	size_t      current_setting;
    std::set<size_t> active_box_id;

    bool decode(ByteVector &data) {
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
        for(size_t ibox(0); ibox<sizeof(flag)*CHAR_BIT; ibox++) {
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
struct ImuRaw : public Request {
    ID id() const { return ID::MSP_RAW_IMU; }

    std::array<int16_t, 3> acc;
    std::array<int16_t, 3> gyro;
    std::array<int16_t, 3> magn;

    bool decode(ByteVector &data) {
        acc = {{deserialise_int16(data, 0), deserialise_int16(data, 2), deserialise_int16(data, 4)}};
        gyro = {{deserialise_int16(data, 6), deserialise_int16(data, 8), deserialise_int16(data, 10)}};
        magn = {{deserialise_int16(data, 12), deserialise_int16(data, 14), deserialise_int16(data, 16)}};
        return true;
    }
};

// Imu in SI units
struct ImuSI {
    std::array<float, 3> acc;   // m/s^2
    std::array<float, 3> gyro;  // deg/s
    std::array<float, 3> magn;  // uT

    ImuSI(const ImuRaw &imu_raw,
          const float acc_1g,       // sensor value at 1g
          const float gyro_unit,    // resolution in 1/(deg/s)
          const float magn_gain,    // scale magnetic value to uT (micro Tesla)
          const float si_unit_1g    // acceleration at 1g (in m/s^2)
            )
    {
        acc = {{imu_raw.acc[0]/acc_1g*si_unit_1g,
                imu_raw.acc[1]/acc_1g*si_unit_1g,
                imu_raw.acc[2]/acc_1g*si_unit_1g}};

        gyro = {{imu_raw.gyro[0]*gyro_unit,
                 imu_raw.gyro[1]*gyro_unit,
                 imu_raw.gyro[2]*gyro_unit}};

        magn = {{imu_raw.magn[0]*magn_gain,
                 imu_raw.magn[1]*magn_gain,
                 imu_raw.magn[2]*magn_gain}};
    }
};

// MSP_SERVO: 103
struct Servo : public Request {
    ID id() const { return ID::MSP_SERVO; }

    uint16_t servo[N_SERVO];

    bool decode(ByteVector &data) {
        for(unsigned int i=0; i<N_SERVO; i++)
            servo[i] = deserialise_uint16(data, 2*i);
    }
};

// MSP_MOTOR: 104
struct Motor : public Request {
    ID id() const { return ID::MSP_MOTOR; }

    uint16_t motor[N_MOTOR];

    bool decode(ByteVector &data) {
        for(unsigned int i=0; i<N_MOTOR; i++)
            motor[i] = deserialise_uint16(data, 2*i);
    }
};

// MSP_RC: 105
struct Rc : public Request {
    ID id() const { return ID::MSP_RC; }

    std::vector<uint16_t> channels;

    bool decode(ByteVector &data) {
        channels.clear();
        // If feature 'RX_MSP' is active but "USE_RX_MSP" is undefined for the target,
        // no RC data is provided as feedback. See also description at 'MSP_SET_RAW_RC'.
        // In this case, return 0 for all RC channels.
        for(size_t i(0); i<data.size(); i+=sizeof(uint16_t)) {
            channels.push_back(deserialise_uint16(data, i));
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

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
        altitude = deserialise_int32(data, 0)/100.0f;
        vario    = deserialise_int16(data, 4)/100.0f;
    }
};

// MSP_ANALOG: 110
struct Analog : public Request {
    ID id() const { return ID::MSP_ANALOG; }

    float	vbat;           // Volt
    float	powerMeterSum;  // Ah
	size_t	rssi;  // Received Signal Strength Indication [0; 1023]
    float	amperage;       // Ampere

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
        box_pattern.clear();
        for(size_t i(0); i<data.size(); i+=2) {
            const uint16_t box_conf = deserialise_uint16(data, i);
            //box_conf.push_back(deser16(data, i));

            std::array<std::set<SwitchPosition>,NAUX> aux_sp;
            for(size_t iaux(0); iaux<NAUX; iaux++) {
                for(size_t ip(0); ip<3; ip++) {
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

	size_t powerTrigger;
	size_t minThrottle, maxThrottle, failsafeThrottle;
	size_t minCommand;
	size_t arm, lifetime;
    float mag_declination; // degree
    float vbatScale, vbatLevelWarn1, vbatLevelWarn2, vbatLevelCrit;

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
        for(size_t i(0); i<N_MOTOR; i++)
            pwm_pin[i] = data[i];
    }
};

// MSP_BOXNAMES: 116
struct BoxNames : public Request {
    ID id() const { return ID::MSP_BOXNAMES; }

    std::vector<std::string> box_names;

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
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

    ByteVector box_ids;

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
        for(size_t i(0); i<N_SERVO; i++) {
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

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
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

    bool decode(ByteVector &data) {
        msg = std::string((data).begin(), (data).end());
    }
};

// MSP_DEBUG: 254
struct Debug : public Request {
    ID id() const { return ID::MSP_DEBUG; }

    uint16_t debug1;
    uint16_t debug2;
    uint16_t debug3;
    uint16_t debug4;

    bool decode(ByteVector &data) {
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

    std::vector<uint16_t> channels;

    ByteVector encode() const {
        ByteVector data;
        for(const uint16_t c : channels) {
            serialise_uint16(c, data);
        }
        return data;
    }
};

// MSP_SET_RAW_GPS: 201
struct SetRawGPS : public Response {
    ID id() const { return ID::MSP_SET_RAW_GPS; }

    uint8_t fix;
    uint8_t numSat;
    uint32_t lat;
    uint32_t lon;
    uint16_t altitude;
    uint16_t speed;

    ByteVector encode() const {
        ByteVector data;
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

    ByteVector encode() const {
        ByteVector data(7);
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
    ByteVector encode() const {
        return ByteVector();
    }
};

// MSP_MAG_CALIBRATION: 206
struct MagCalibration : public Response {
    ID id() const { return ID::MSP_MAG_CALIBRATION; }
    ByteVector encode() const {
        return ByteVector();
    }
};

// MSP_RESET_CONF: 208
struct ResetConfig : public Response {
    ID id() const { return ID::MSP_RESET_CONF; }
    ByteVector encode() const {
        return ByteVector();
    }
};

// MSP_SELECT_SETTING: 210
struct SelectSetting : public Response {
    ID id() const { return ID::MSP_SELECT_SETTING; }

	size_t current_setting;

    ByteVector encode() const {
        ByteVector data(1);
        data[0] = uint8_t(current_setting);
        return data;
    }
};

// MSP_SET_HEAD: 211
struct SetHeading : public Response {
    ID id() const { return ID::MSP_SET_HEAD; }

    int heading;

    ByteVector encode() const {
        ByteVector data;
        serialise_int16(int16_t(heading), data);
        assert(data.size()==2);
        return data;
    }
};

// MSP_SET_MOTOR: 214
struct SetMotor : public Response {
    ID id() const { return ID::MSP_SET_MOTOR; }

    std::array<uint16_t,N_MOTOR> motor;

    ByteVector encode() const {
        ByteVector data;
        for(size_t i(0); i<N_MOTOR; i++)
            serialise_uint16(motor[i], data);
        assert(data.size()==N_MOTOR*2);
        return data;
    }
};

// MSP_EEPROM_WRITE: 250
struct WriteEEPROM : public Response {
    ID id() const { return ID::MSP_EEPROM_WRITE; }
    ByteVector encode() const {
        return ByteVector();
    }
};

} // namespace msg
} // namespace msp

#endif // MSP_MSG_HPP
