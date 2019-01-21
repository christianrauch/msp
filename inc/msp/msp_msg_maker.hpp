#ifndef MSP_MSG_MAKER_HPP
#define MSP_MSG_MAKER_HPP
#include "msp_msg.hpp"

namespace msp {

class message_maker {
public:
    message_maker(FirmwareVariant v = FirmwareVariant::INAV) : fw_variant(v){};

    void set_default_variant(FirmwareVariant v) { fw_variant = v; }

    std::shared_ptr<Message> operator()(ID& id, FirmwareVariant& v) {
        std::shared_ptr<Message> msg_ptr;
        switch(id) {
        case ID::MSP_API_VERSION:
            msg_ptr.reset(new msg::ApiVersion(v));
            break;
        case ID::MSP_FC_VARIANT:
            msg_ptr.reset(new msg::FcVariant(v));
            break;
        case ID::MSP_FC_VERSION:
            msg_ptr.reset(new msg::FcVersion(v));
            break;
        case ID::MSP_BOARD_INFO:
            msg_ptr.reset(new msg::BoardInfo(v));
            break;
        case ID::MSP_BUILD_INFO:
            msg_ptr.reset(new msg::BuildInfo(v));
            break;

        case ID::MSP_INAV_PID:
            msg_ptr.reset(new msg::InavPid(v));
            break;
        case ID::MSP_SET_INAV_PID:
            msg_ptr.reset(new msg::SetInavPid(v));
            break;
        case ID::MSP_NAME:
            msg_ptr.reset(new msg::BoardName(v));
            break;
        case ID::MSP_SET_NAME:
            msg_ptr.reset(new msg::SetBoardName(v));
            break;
        case ID::MSP_NAV_POSHOLD:
            msg_ptr.reset(new msg::NavPosHold(v));
            break;
        case ID::MSP_SET_NAV_POSHOLD:
            msg_ptr.reset(new msg::SetNavPosHold(v));
            break;
        case ID::MSP_CALIBRATION_DATA:
            msg_ptr.reset(new msg::CalibrationData(v));
            break;
        case ID::MSP_SET_CALIBRATION_DATA:
            msg_ptr.reset(new msg::SetCalibrationData(v));
            break;
        case ID::MSP_POSITION_ESTIMATION_CONFIG:
            msg_ptr.reset(new msg::PositionEstimationConfig(v));
            break;
        case ID::MSP_SET_POSITION_ESTIMATION_CONFIG:
            msg_ptr.reset(new msg::SetPositionEstimationConfig(v));
            break;
        case ID::MSP_WP_MISSION_LOAD:
            msg_ptr.reset(new msg::WpMissionLoad(v));
            break;
        case ID::MSP_WP_MISSION_SAVE:
            msg_ptr.reset(new msg::WpMissionSave(v));
            break;
        case ID::MSP_WP_GETINFO:
            msg_ptr.reset(new msg::WpGetInfo(v));
            break;
        case ID::MSP_RTH_AND_LAND_CONFIG:
            msg_ptr.reset(new msg::RthAndLandConfig(v));
            break;
        case ID::MSP_SET_RTH_AND_LAND_CONFIG:
            msg_ptr.reset(new msg::SetRthAndLandConfig(v));
            break;
        case ID::MSP_FW_CONFIG:
            msg_ptr.reset(new msg::FwConfig(v));
            break;
        case ID::MSP_SET_FW_CONFIG:
            msg_ptr.reset(new msg::SetFwConfig(v));
            break;
        case ID::MSP_BATTERY_CONFIG:
            msg_ptr.reset(new msg::BatteryConfig(v));
            break;
        case ID::MSP_SET_BATTERY_CONFIG:
            msg_ptr.reset(new msg::SetBatteryConfig(v));
            break;
        case ID::MSP_MODE_RANGES:
            msg_ptr.reset(new msg::ModeRanges(v));
            break;
        case ID::MSP_SET_MODE_RANGE:
            msg_ptr.reset(new msg::SetModeRange(v));
            break;
        case ID::MSP_FEATURE:
            msg_ptr.reset(new msg::Feature(v));
            break;
        case ID::MSP_SET_FEATURE:
            msg_ptr.reset(new msg::SetFeature(v));
            break;
        case ID::MSP_BOARD_ALIGNMENT:
            msg_ptr.reset(new msg::BoardAlignment(v));
            break;
        case ID::MSP_SET_BOARD_ALIGNMENT:
            msg_ptr.reset(new msg::SetBoardAlignment(v));
            break;
        case ID::MSP_CURRENT_METER_CONFIG:
            msg_ptr.reset(new msg::CurrentMeterConfig(v));
            break;
        case ID::MSP_SET_CURRENT_METER_CONFIG:
            msg_ptr.reset(new msg::SetCurrentMeterConfig(v));
            break;
        case ID::MSP_MIXER:
            msg_ptr.reset(new msg::Mixer(v));
            break;
        case ID::MSP_SET_MIXER:
            msg_ptr.reset(new msg::SetMixer(v));
            break;
        case ID::MSP_RX_CONFIG:
            msg_ptr.reset(new msg::RxConfig(v));
            break;
        case ID::MSP_SET_RX_CONFIG:
            msg_ptr.reset(new msg::SetRxConfig(v));
            break;
        case ID::MSP_LED_COLORS:
            msg_ptr.reset(new msg::LedColors(v));
            break;
        case ID::MSP_SET_LED_COLORS:
            msg_ptr.reset(new msg::SetLedColors(v));
            break;
        case ID::MSP_LED_STRIP_CONFIG:
            msg_ptr.reset(new msg::LedStripConfigs(v));
            break;
        case ID::MSP_SET_LED_STRIP_CONFIG:
            msg_ptr.reset(new msg::SetLedStripConfig(v));
            break;
        case ID::MSP_RSSI_CONFIG:
            msg_ptr.reset(new msg::RssiConfig(v));
            break;
        case ID::MSP_SET_RSSI_CONFIG:
            msg_ptr.reset(new msg::SetRssiConfig(v));
            break;
        case ID::MSP_ADJUSTMENT_RANGES:
            msg_ptr.reset(new msg::AdjustmentRanges(v));
            break;
        case ID::MSP_SET_ADJUSTMENT_RANGE:
            msg_ptr.reset(new msg::SetAdjustmentRange(v));
            break;
        case ID::MSP_CF_SERIAL_CONFIG:
            msg_ptr.reset(new msg::CfSerialConfig(v));
            break;
        case ID::MSP_SET_CF_SERIAL_CONFIG:
            msg_ptr.reset(new msg::SetCfSerialConfig(v));
            break;
        case ID::MSP_VOLTAGE_METER_CONFIG:
            msg_ptr.reset(new msg::VoltageMeterConfig(v));
            break;
        case ID::MSP_SET_VOLTAGE_METER_CONFIG:
            msg_ptr.reset(new msg::SetVoltageMeterConfig(v));
            break;
        case ID::MSP_SONAR_ALTITUDE:
            msg_ptr.reset(new msg::SonarAltitude(v));
            break;
        case ID::MSP_PID_CONTROLLER:
            msg_ptr.reset(new msg::PidController(v));
            break;
        case ID::MSP_SET_PID_CONTROLLER:
            msg_ptr.reset(new msg::SetPidController(v));
            break;
        case ID::MSP_ARMING_CONFIG:
            msg_ptr.reset(new msg::ArmingConfig(v));
            break;
        case ID::MSP_SET_ARMING_CONFIG:
            msg_ptr.reset(new msg::SetArmingConfig(v));
            break;
        case ID::MSP_RX_MAP:
            msg_ptr.reset(new msg::RxMap(v));
            break;
        case ID::MSP_SET_RX_MAP:
            msg_ptr.reset(new msg::SetRxMap(v));
            break;
        case ID::MSP_BF_CONFIG:
            msg_ptr.reset(new msg::BfConfig(v));
            break;
        case ID::MSP_SET_BF_CONFIG:
            msg_ptr.reset(new msg::SetBfConfig(v));
            break;
        case ID::MSP_REBOOT:
            msg_ptr.reset(new msg::Reboot(v));
            break;
        case ID::MSP_BF_BUILD_INFO:
            msg_ptr.reset(new msg::BfBuildInfo(v));
            break;
        case ID::MSP_DATAFLASH_SUMMARY:
            msg_ptr.reset(new msg::DataflashSummary(v));
            break;
        case ID::MSP_DATAFLASH_READ:
            msg_ptr.reset(new msg::DataflashRead(v));
            break;
        case ID::MSP_DATAFLASH_ERASE:
            msg_ptr.reset(new msg::DataflashErase(v));
            break;
        case ID::MSP_LOOP_TIME:
            msg_ptr.reset(new msg::LoopTime(v));
            break;
        case ID::MSP_SET_LOOP_TIME:
            msg_ptr.reset(new msg::SetLoopTime(v));
            break;
        case ID::MSP_FAILSAFE_CONFIG:
            msg_ptr.reset(new msg::FailsafeConfig(v));
            break;
        case ID::MSP_SET_FAILSAFE_CONFIG:
            msg_ptr.reset(new msg::SetFailsafeConfig(v));
            break;
        case ID::MSP_RXFAIL_CONFIG:
            msg_ptr.reset(new msg::RxFailConfigs(v));
            break;
        case ID::MSP_SET_RXFAIL_CONFIG:
            msg_ptr.reset(new msg::SetRxFailConfigs(v));
            break;
        case ID::MSP_SDCARD_SUMMARY:
            msg_ptr.reset(new msg::SdcardSummary(v));
            break;
        case ID::MSP_BLACKBOX_CONFIG:
            msg_ptr.reset(new msg::BlackboxConfig(v));
            break;
        case ID::MSP_SET_BLACKBOX_CONFIG:
            msg_ptr.reset(new msg::SetBlackboxConfig(v));
            break;
        case ID::MSP_TRANSPONDER_CONFIG:
            msg_ptr.reset(new msg::TransponderConfig(v));
            break;
        case ID::MSP_SET_TRANSPONDER_CONFIG:
            msg_ptr.reset(new msg::SetTransponderConfig(v));
            break;
        case ID::MSP_OSD_CONFIG:
            msg_ptr.reset(new msg::OsdConfig(v));
            break;
        case ID::MSP_SET_OSD_CONFIG:
            msg_ptr.reset(new msg::SetOsdConfig(v));
            break;
        case ID::MSP_OSD_CHAR_READ:
            // msg_ptr.reset(new msg::OsdCharRead(v));
            break;
        case ID::MSP_OSD_CHAR_WRITE:
            msg_ptr.reset(new msg::OsdCharWrite(v));
            break;
        case ID::MSP_VTX_CONFIG:
            msg_ptr.reset(new msg::VtxConfig(v));
            break;
        case ID::MSP_SET_VTX_CONFIG:
            msg_ptr.reset(new msg::SetVtxConfig(v));
            break;
        case ID::MSP_ADVANCED_CONFIG:
            msg_ptr.reset(new msg::AdvancedConfig(v));
            break;
        case ID::MSP_SET_ADVANCED_CONFIG:
            msg_ptr.reset(new msg::SetAdvancedConfig(v));
            break;
        case ID::MSP_FILTER_CONFIG:
            msg_ptr.reset(new msg::FilterConfig(v));
            break;
        case ID::MSP_SET_FILTER_CONFIG:
            msg_ptr.reset(new msg::SetFilterConfig(v));
            break;
        case ID::MSP_PID_ADVANCED:
            msg_ptr.reset(new msg::PidAdvanced(v));
            break;
        case ID::MSP_SET_PID_ADVANCED:
            msg_ptr.reset(new msg::SetPidAdvanced(v));
            break;
        case ID::MSP_SENSOR_CONFIG:
            msg_ptr.reset(new msg::SensorConfig(v));
            break;
        case ID::MSP_SET_SENSOR_CONFIG:
            msg_ptr.reset(new msg::SetSensorConfig(v));
            break;
        case ID::MSP_CAMERA_CONTROL:
            msg_ptr.reset(new msg::CameraControl(v));
            break;
        case ID::MSP_SET_ARMING_DISABLED:
            msg_ptr.reset(new msg::SetArmingDisabled(v));
            break;
        case ID::MSP_IDENT:
            msg_ptr.reset(new msg::Ident(v));
            break;
        case ID::MSP_STATUS:
            msg_ptr.reset(new msg::Status(v));
            break;
        case ID::MSP_RAW_IMU:
            msg_ptr.reset(new msg::RawImu(v));
            break;
        case ID::MSP_SERVO:
            msg_ptr.reset(new msg::Servo(v));
            break;
        case ID::MSP_MOTOR:
            msg_ptr.reset(new msg::Motor(v));
            break;
        case ID::MSP_RC:
            msg_ptr.reset(new msg::Rc(v));
            break;
        case ID::MSP_RAW_GPS:
            msg_ptr.reset(new msg::RawGPS(v));
            break;
        case ID::MSP_MISC:
            msg_ptr.reset(new msg::Misc(v));
            break;
        case ID::MSP_MOTOR_PINS:
            msg_ptr.reset(new msg::MotorPins(v));
            break;
        case ID::MSP_BOXNAMES:
            msg_ptr.reset(new msg::BoxNames(v));
            break;
        case ID::MSP_PIDNAMES:
            msg_ptr.reset(new msg::PidNames(v));
            break;
        case ID::MSP_WP:
            msg_ptr.reset(new msg::WayPoint(v));
            break;
        case ID::MSP_BOXIDS:
            msg_ptr.reset(new msg::BoxIds(v));
            break;
        case ID::MSP_SERVO_CONF:
            msg_ptr.reset(new msg::ServoConf(v));
            break;
        case ID::MSP_NAV_STATUS:
            msg_ptr.reset(new msg::NavStatus(v));
            break;
        case ID::MSP_NAV_CONFIG:
            msg_ptr.reset(new msg::NavConfig(v));
            break;
        case ID::MSP_MOTOR_3D_CONFIG:
            msg_ptr.reset(new msg::Motor3dConfig(v));
            break;
        case ID::MSP_RC_DEADBAND:
            msg_ptr.reset(new msg::RcDeadband(v));
            break;
        case ID::MSP_SENSOR_ALIGNMENT:
            msg_ptr.reset(new msg::SensorAlignment(v));
            break;
        case ID::MSP_LED_STRIP_MODECOLOR:
            msg_ptr.reset(new msg::LedStripModecolor(v));
            break;

        case ID::MSP_VOLTAGE_METERS:
            msg_ptr.reset(new msg::VoltageMeters(v));
            break;
        case ID::MSP_CURRENT_METERS:
            msg_ptr.reset(new msg::CurrentMeters(v));
            break;
        case ID::MSP_BATTERY_STATE:
            msg_ptr.reset(new msg::BatteryState(v));
            break;
        case ID::MSP_MOTOR_CONFIG:
            msg_ptr.reset(new msg::MotorConfig(v));
            break;
        case ID::MSP_GPS_CONFIG:
            msg_ptr.reset(new msg::GpsConfig(v));
            break;
        case ID::MSP_COMPASS_CONFIG:
            msg_ptr.reset(new msg::CompassConfig(v));
            break;
        case ID::MSP_ESC_SENSOR_DATA:
            msg_ptr.reset(new msg::EscSensorData(v));
            break;

        case ID::MSP_STATUS_EX:
            msg_ptr.reset(new msg::StatusEx(v));
            break;
        case ID::MSP_SENSOR_STATUS:
            msg_ptr.reset(new msg::SensorStatus(v));
            break;
        case ID::MSP_UID:
            msg_ptr.reset(new msg::Uid(v));
            break;
        case ID::MSP_GPSSVINFO:
            msg_ptr.reset(new msg::GpsSvInfo(v));
            break;
        case ID::MSP_GPSSTATISTICS:
            msg_ptr.reset(new msg::GpsStatistics(v));
            break;

        case ID::MSP_OSD_VIDEO_CONFIG:
            msg_ptr.reset(new msg::OsdVideoConfig(v));
            break;
        case ID::MSP_SET_OSD_VIDEO_CONFIG:
            msg_ptr.reset(new msg::SetOsdVideoConfig(v));
            break;
        case ID::MSP_DISPLAYPORT:
            msg_ptr.reset(new msg::Displayport(v));
            break;
        case ID::MSP_COPY_PROFILE:
            msg_ptr.reset(new msg::CopyProfile(v));
            break;

        case ID::MSP_BEEPER_CONFIG:
            msg_ptr.reset(new msg::BeeperConfig(v));
            break;
        case ID::MSP_SET_BEEPER_CONFIG:
            msg_ptr.reset(new msg::SetBeeperConfig(v));
            break;
        case ID::MSP_SET_TX_INFO:
            msg_ptr.reset(new msg::SetTxInfo(v));
            break;
        case ID::MSP_TX_INFO:
            msg_ptr.reset(new msg::TxInfo(v));
            break;

        case ID::MSP_SET_RAW_RC:
            msg_ptr.reset(new msg::SetRawRc(v));
            break;
        case ID::MSP_SET_RAW_GPS:
            msg_ptr.reset(new msg::SetRawGPS(v));
            break;
        case ID::MSP_SET_PID:
            msg_ptr.reset(new msg::SetPid(v));
            break;

        case ID::MSP_SET_BOX:
            msg_ptr.reset(new msg::SetBox(v));
            break;
        case ID::MSP_SET_RC_TUNING:
            msg_ptr.reset(new msg::SetRcTuning(v));
            break;
        case ID::MSP_ACC_CALIBRATION:
            msg_ptr.reset(new msg::AccCalibration(v));
            break;
        case ID::MSP_MAG_CALIBRATION:
            msg_ptr.reset(new msg::MagCalibration(v));
            break;

        case ID::MSP_SET_MISC:
            msg_ptr.reset(new msg::SetMisc(v));
            break;
        case ID::MSP_RESET_CONF:
            msg_ptr.reset(new msg::ResetConfig(v));
            break;
        case ID::MSP_SET_WP:
            msg_ptr.reset(new msg::SetWp(v));
            break;
        case ID::MSP_SELECT_SETTING:
            msg_ptr.reset(new msg::SelectSetting(v));
            break;

        case ID::MSP_SET_HEADING:
            msg_ptr.reset(new msg::SetHeading(v));
            break;
        case ID::MSP_SET_SERVO_CONF:
            msg_ptr.reset(new msg::SetServoConf(v));
            break;
        case ID::MSP_SET_MOTOR:
            msg_ptr.reset(new msg::SetMotor(v));
            break;
        case ID::MSP_SET_NAV_CONFIG:
            msg_ptr.reset(new msg::SetNavConfig(v));
            break;

        case ID::MSP_SET_MOTOR_3D_CONF:
            msg_ptr.reset(new msg::SetMotor3dConf(v));
            break;
        case ID::MSP_SET_RC_DEADBAND:
            msg_ptr.reset(new msg::SetRcDeadband(v));
            break;
        case ID::MSP_SET_RESET_CURR_PID:
            msg_ptr.reset(new msg::SetResetCurrPid(v));
            break;
        case ID::MSP_SET_SENSOR_ALIGNMENT:
            msg_ptr.reset(new msg::SetSensorAlignment(v));
            break;

        case ID::MSP_SET_LED_STRIP_MODECOLOR:
            msg_ptr.reset(new msg::SetLedStripModecolor(v));
            break;
        case ID::MSP_SET_MOTOR_CONFIG:
            msg_ptr.reset(new msg::SetMotorConfig(v));
            break;
        case ID::MSP_SET_GPS_CONFIG:
            msg_ptr.reset(new msg::SetGpsConfig(v));
            break;
        case ID::MSP_SET_COMPASS_CONFIG:
            msg_ptr.reset(new msg::SetCompassConfig(v));
            break;

        case ID::MSP_SET_ACC_TRIM:
            msg_ptr.reset(new msg::SetAccTrim(v));
            break;
        case ID::MSP_ACC_TRIM:
            msg_ptr.reset(new msg::AccTrim(v));
            break;
        case ID::MSP_SERVO_MIX_RULES:
            msg_ptr.reset(new msg::ServoMixRules(v));
            break;
        case ID::MSP_SET_SERVO_MIX_RULE:
            msg_ptr.reset(new msg::SetServoMixRule(v));
            break;

        case ID::MSP_PASSTHROUGH_SERIAL:
            msg_ptr.reset(new msg::PassthroughSerial(v));
            break;
        case ID::MSP_SET_4WAY_IF:
            msg_ptr.reset(new msg::Set4WayIF(v));
            break;
        case ID::MSP_SET_RTC:
            msg_ptr.reset(new msg::SetRtc(v));
            break;
        case ID::MSP_RTC:
            msg_ptr.reset(new msg::Rtc(v));
            break;

        case ID::MSP_EEPROM_WRITE:
            msg_ptr.reset(new msg::WriteEEPROM(v));
            break;
        case ID::MSP_RESERVE_1:
            msg_ptr.reset(new msg::Reserve1(v));
            break;
        case ID::MSP_RESERVE_2:
            msg_ptr.reset(new msg::Reserve2(v));
            break;
        case ID::MSP_DEBUGMSG:
            msg_ptr.reset(new msg::DebugMessage(v));
            break;
        case ID::MSP_DEBUG:
            msg_ptr.reset(new msg::Debug(v));
            break;

        case ID::MSP_V2_FRAME:
            msg_ptr.reset(new msg::V2Frame(v));
            break;

        case ID::MSP2_COMMON_TZ:
            msg_ptr.reset(new msg::CommonTz(v));
            break;
        case ID::MSP2_COMMON_SET_TZ:
            msg_ptr.reset(new msg::CommonSetTz(v));
            break;
        case ID::MSP2_COMMON_SETTING:
            msg_ptr.reset(new msg::CommonSetting(v));
            break;
        case ID::MSP2_COMMON_SET_SETTING:
            msg_ptr.reset(new msg::CommonSetSetting(v));
            break;

        case ID::MSP2_COMMON_MOTOR_MIXER:
            msg_ptr.reset(new msg::CommonMotorMixer(v));
            break;
        case ID::MSP2_COMMON_SET_MOTOR_MIXER:
            msg_ptr.reset(new msg::CommonSetMotorMixer(v));
            break;

        case ID::MSP2_INAV_STATUS:
            msg_ptr.reset(new msg::InavStatus(v));
            break;
        case ID::MSP2_INAV_OPTICAL_FLOW:
            msg_ptr.reset(new msg::InavOpticalFlow(v));
            break;
        case ID::MSP2_INAV_ANALOG:
            msg_ptr.reset(new msg::InavAnalog(v));
            break;
        case ID::MSP2_INAV_MISC:
            msg_ptr.reset(new msg::InavMisc(v));
            break;

        case ID::MSP2_INAV_SET_MISC:
            msg_ptr.reset(new msg::InavSetMisc(v));
            break;
        case ID::MSP2_INAV_BATTERY_CONFIG:
            msg_ptr.reset(new msg::InavBatteryConfig(v));
            break;
        case ID::MSP2_INAV_SET_BATTERY_CONFIG:
            msg_ptr.reset(new msg::InavSetBatteryConfig(v));
            break;
        case ID::MSP2_INAV_RATE_PROFILE:
            msg_ptr.reset(new msg::InavRateProfile(v));
            break;

        case ID::MSP2_INAV_SET_RATE_PROFILE:
            msg_ptr.reset(new msg::InavSetRateProfile(v));
            break;
        case ID::MSP2_INAV_AIR_SPEED:
            msg_ptr.reset(new msg::InavAirSpeed(v));
            break;

        default:
            break;
        }

        return msg_ptr;
    }

    std::shared_ptr<Message> operator()(ID id) {
        return this->operator()(id, fw_variant);
    }

private:
    FirmwareVariant fw_variant;
};

}  // namespace msp

#endif
