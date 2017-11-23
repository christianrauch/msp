#ifndef DESERIALISE_HPP
#define DESERIALISE_HPP

#include "types.hpp"
#include <cstdlib>

namespace msp {

/////////////////////////////////////////////////////////////////////
/// de-/serialization for 16 and 32 bit unsigned integer

static void serialise_uint16(const uint16_t val, ByteVector &data) {
    data.push_back(val>>0);
    data.push_back(val>>8);
}

static uint16_t deserialise_uint16(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8);
}

static void serialise_int16(const int16_t val, ByteVector &data) {
    data.push_back(val>>0);
    data.push_back(val>>8);
}

static int16_t deserialise_int16(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8);
}

static int32_t deserialise_int32(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8) | (data[start+2]<<16) | (data[start+3]<<24);
}

static void serialise_uint32(const uint32_t val, ByteVector &data) {
    data.push_back(val>>0);
    data.push_back(val>>8);
    data.push_back(val>>16);
    data.push_back(val>>24);
}

static uint32_t deserialise_uint32(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8) | (data[start+2]<<16) | (data[start+3]<<24);
}

}

#endif // DESERIALISE_HPP
