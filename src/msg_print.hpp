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

#endif // MSG_PRINT_HPP
