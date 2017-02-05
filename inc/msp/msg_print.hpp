#ifndef MSG_PRINT_HPP
#define MSG_PRINT_HPP

#include "msp_msg.hpp"
#include <ostream>

std::ostream& operator<<(std::ostream& s, const msp::ApiVersion& api_version);

std::ostream& operator<<(std::ostream& s, const msp::FcVariant& fc_variant);

std::ostream& operator<<(std::ostream& s, const msp::FcVersion& fc_version);

std::ostream& operator<<(std::ostream& s, const msp::BoardInfo& board_info);

std::ostream& operator<<(std::ostream& s, const msp::BuildInfo& build_info);

std::ostream& operator<<(std::ostream& s, const msp::Feature& feature);

std::ostream& operator<<(std::ostream& s, const msp::RxMap& rx_map);

std::ostream& operator<<(std::ostream& s, const msp::Ident& ident);

std::ostream& operator<<(std::ostream& s, const msp::Status& status);

std::ostream& operator<<(std::ostream& s, const msp::Imu& imu);

std::ostream& operator<<(std::ostream& s, const msp::Servo& servo);

std::ostream& operator<<(std::ostream& s, const msp::Motor& motor);

std::ostream& operator<<(std::ostream& s, const msp::Rc& rc);

std::ostream& operator<<(std::ostream& s, const msp::Attitude& attitude);

std::ostream& operator<<(std::ostream& s, const msp::Altitude& altitude);

std::ostream& operator<<(std::ostream& s, const msp::Analog& analog);

std::ostream& operator<<(std::ostream& s, const msp::RcTuning& rc_tuning);

std::ostream& operator<<(std::ostream& s, const msp::Pid& pid);

std::ostream& operator<<(std::ostream& s, const msp::Box& box);

std::ostream& operator<<(std::ostream& s, const msp::Misc& misc);

std::ostream& operator<<(std::ostream& s, const msp::MotorPins& pin);

std::ostream& operator<<(std::ostream& s, const msp::BoxNames& box_names);

std::ostream& operator<<(std::ostream& s, const msp::PidNames& pid_names);

std::ostream& operator<<(std::ostream& s, const msp::BoxIds& box_ids);

std::ostream& operator<<(std::ostream& s, const msp::ServoConf& servo_conf);

std::ostream& operator<<(std::ostream& s, const msp::Debug& debug);

#endif // MSG_PRINT_HPP
