#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <cstdint>

namespace msp {

/**
 * @brief ByteVector vector of bytes
 */
typedef std::vector<uint8_t> ByteVector;

/**
 * @brief ID id of a message
 */
typedef uint8_t ID;


/////////////////////////////////////////////////////////////////////
/// Generic message types

struct Message {
    virtual ID id() = 0;
};

// send to FC
struct Request : public Message {
    virtual void decode(const std::vector<uint8_t> &data) = 0;
};

// received from FC
struct Response : public Message {
    virtual std::vector<uint8_t> encode() const = 0;
};

} // namespace msp

#endif // TYPES_HPP
