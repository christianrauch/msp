#ifndef DESERIALISE_HPP
#define DESERIALISE_HPP

#include "types.hpp"
#include <cstdlib>

namespace msp {

/////////////////////////////////////////////////////////////////////
/// de-/serialization for 16 and 32 bit unsigned integer

void serialise_uint16(const uint16_t val, ByteVector &data);

uint16_t deserialise_uint16(const ByteVector &data, const size_t start);

void serialise_int16(const int16_t val, ByteVector &data);

int16_t deserialise_int16(const ByteVector &data, const size_t start);

int32_t deserialise_int32(const ByteVector &data, const size_t start);

void serialise_uint32(const uint32_t val, ByteVector &data);

uint32_t deserialise_uint32(const ByteVector &data, const size_t start);

}

#endif // DESERIALISE_HPP
