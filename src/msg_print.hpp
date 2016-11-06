#ifndef MSG_PRINT_HPP
#define MSG_PRINT_HPP

#include <fcu_msg.hpp>
#include <ostream>

namespace fcu {

std::ostream& operator<<(std::ostream& s, const fcu::Ident& ident);

std::ostream& operator<<(std::ostream& s, const fcu::Status& status);

std::ostream& operator<<(std::ostream& s, const fcu::Imu& imu);

} // namespace fcu

#endif // MSG_PRINT_HPP
