#ifndef MSG_PRINT_HPP
#define MSG_PRINT_HPP

#include "fcu_msg.hpp"
#include <ostream>

std::ostream& operator<<(std::ostream& s, const fcu::Ident& ident);

std::ostream& operator<<(std::ostream& s, const fcu::Status& status);

std::ostream& operator<<(std::ostream& s, const fcu::Imu& imu);

std::ostream& operator<<(std::ostream& s, const msp::Servo& servo);

std::ostream& operator<<(std::ostream& s, const msp::Motor& motor);

std::ostream& operator<<(std::ostream& s, const msp::Rc& rc);

std::ostream& operator<<(std::ostream& s, const fcu::Attitude& attitude);

std::ostream& operator<<(std::ostream& s, const fcu::Altitude& altitude);

std::ostream& operator<<(std::ostream& s, const fcu::Analog& analog);

std::ostream& operator<<(std::ostream& s, const msp::RcTuning& rc_tuning);

std::ostream& operator<<(std::ostream& s, const fcu::PID& pid);

std::ostream& operator<<(std::ostream& s, const fcu::Box& box);

std::ostream& operator<<(std::ostream& s, const fcu::Misc& misc);

std::ostream& operator<<(std::ostream& s, const msp::MotorPin& pin);

std::ostream& operator<<(std::ostream& s, const msp::BoxNames& box_names);

std::ostream& operator<<(std::ostream& s, const msp::PidNames& pid_names);

std::ostream& operator<<(std::ostream& s, const msp::BoxIds& box_ids);

#endif // MSG_PRINT_HPP
