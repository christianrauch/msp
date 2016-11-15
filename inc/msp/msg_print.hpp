#ifndef MSG_PRINT_HPP
#define MSG_PRINT_HPP

#include "fcu_msg.hpp"
#include <ostream>

void operator<<(std::ostream& s, const fcu::Ident& ident);

void operator<<(std::ostream& s, const fcu::Status& status);

void operator<<(std::ostream& s, const fcu::Imu& imu);

void operator<<(std::ostream& s, const msp::Servo& servo);

void operator<<(std::ostream& s, const msp::Motor& motor);

void operator<<(std::ostream& s, const msp::Rc& rc);

void operator<<(std::ostream& s, const fcu::Attitude& attitude);

void operator<<(std::ostream& s, const fcu::Altitude& altitude);

void operator<<(std::ostream& s, const fcu::Analog& analog);

void operator<<(std::ostream& s, const fcu::RcTuning& rc_tuning);

void operator<<(std::ostream& s, const fcu::PID& pid);

void operator<<(std::ostream& s, const fcu::Box& box);

void operator<<(std::ostream& s, const fcu::Misc& misc);

void operator<<(std::ostream& s, const msp::MotorPins& pin);

void operator<<(std::ostream& s, const msp::BoxNames& box_names);

void operator<<(std::ostream& s, const msp::PidNames& pid_names);

void operator<<(std::ostream& s, const msp::BoxIds& box_ids);

void operator<<(std::ostream& s, const msp::ServoConf& servo_conf);

void operator<<(std::ostream& s, const msp::Debug& debug);

#endif // MSG_PRINT_HPP
