#ifndef MSG_PRINT_HPP
#define MSG_PRINT_HPP

#include "msp_msg.hpp"
#include <ostream>

void operator<<(std::ostream& s, const msp::Ident& ident);

void operator<<(std::ostream& s, const msp::Status& status);

void operator<<(std::ostream& s, const msp::Imu& imu);

void operator<<(std::ostream& s, const msp::Servo& servo);

void operator<<(std::ostream& s, const msp::Motor& motor);

void operator<<(std::ostream& s, const msp::Rc& rc);

void operator<<(std::ostream& s, const msp::Attitude& attitude);

void operator<<(std::ostream& s, const msp::Altitude& altitude);

void operator<<(std::ostream& s, const msp::Analog& analog);

void operator<<(std::ostream& s, const msp::RcTuning& rc_tuning);

void operator<<(std::ostream& s, const msp::Pid& pid);

void operator<<(std::ostream& s, const msp::Box& box);

void operator<<(std::ostream& s, const msp::Misc& misc);

void operator<<(std::ostream& s, const msp::MotorPins& pin);

void operator<<(std::ostream& s, const msp::BoxNames& box_names);

void operator<<(std::ostream& s, const msp::PidNames& pid_names);

void operator<<(std::ostream& s, const msp::BoxIds& box_ids);

void operator<<(std::ostream& s, const msp::ServoConf& servo_conf);

void operator<<(std::ostream& s, const msp::Debug& debug);

#endif // MSG_PRINT_HPP
