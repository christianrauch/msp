// MSP message definitions
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

#ifndef MSP_MSG_H
#define MSP_MSG_H

#include <cstdint>

/// command message_id
#define MSP_IDENT   100
#define MSP_STATUS  101
#define MSP_RAW_IMU 102
#define MSP_SERVO   103
#define MSP_MOTOR   104


/// memory layout
namespace msp {

namespace data {

#pragma pack(push, 1)

struct Ident {
    uint8_t     version;
    uint8_t     type;
    uint8_t     msp_version;
    uint32_t    capability;
};

struct Status {
    uint16_t    time;   // in us
    uint16_t    i2c_errors_count;
    uint16_t    sensor;
    uint32_t    flag;
    uint8_t     current_setting;
};

#pragma pack(pop)

} // namespace data

} // namespace msp

#endif // MSP_MSG_H
