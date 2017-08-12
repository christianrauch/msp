#ifndef MSG_PRINT_HPP
#define MSG_PRINT_HPP

#include "msp_msg.hpp"
#include <ostream>

std::ostream& operator<<(std::ostream& s, const msp::msg::ApiVersion& api_version);

std::ostream& operator<<(std::ostream& s, const msp::msg::FcVariant& fc_variant);

std::ostream& operator<<(std::ostream& s, const msp::msg::FcVersion& fc_version);

std::ostream& operator<<(std::ostream& s, const msp::msg::BoardInfo& board_info);

std::ostream& operator<<(std::ostream& s, const msp::msg::BuildInfo& build_info);

std::ostream& operator<<(std::ostream& s, const msp::msg::Feature& feature);

std::ostream& operator<<(std::ostream& s, const msp::msg::RxMap& rx_map);

std::ostream& operator<<(std::ostream& s, const msp::msg::Ident& ident);

std::ostream& operator<<(std::ostream& s, const msp::msg::Status& status);

std::ostream& operator<<(std::ostream& s, const msp::msg::ImuRaw& imu);

std::ostream& operator<<(std::ostream& s, const msp::msg::ImuSI& imu);

std::ostream& operator<<(std::ostream& s, const msp::msg::Servo& servo);

std::ostream& operator<<(std::ostream& s, const msp::msg::Motor& motor);

std::ostream& operator<<(std::ostream& s, const msp::msg::Rc& rc);

std::ostream& operator<<(std::ostream& s, const msp::msg::Attitude& attitude);

std::ostream& operator<<(std::ostream& s, const msp::msg::Altitude& altitude);

std::ostream& operator<<(std::ostream& s, const msp::msg::Analog& analog);

std::ostream& operator<<(std::ostream& s, const msp::msg::RcTuning& rc_tuning);

std::ostream& operator<<(std::ostream& s, const msp::msg::Pid& pid);

std::ostream& operator<<(std::ostream& s, const msp::msg::Box& box);

std::ostream& operator<<(std::ostream& s, const msp::msg::Misc& misc);

std::ostream& operator<<(std::ostream& s, const msp::msg::MotorPins& pin);

std::ostream& operator<<(std::ostream& s, const msp::msg::BoxNames& box_names);

std::ostream& operator<<(std::ostream& s, const msp::msg::PidNames& pid_names);

std::ostream& operator<<(std::ostream& s, const msp::msg::BoxIds& box_ids);

std::ostream& operator<<(std::ostream& s, const msp::msg::ServoConf& servo_conf);

std::ostream& operator<<(std::ostream& s, const msp::msg::Debug& debug);

#endif // MSG_PRINT_HPP
