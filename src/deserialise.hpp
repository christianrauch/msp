#ifndef DESERIALISE_HPP
#define DESERIALISE_HPP

#include "types.hpp"
#include <cstdlib>

namespace msp {

/////////////////////////////////////////////////////////////////////
/// de-/serialization for 16 and 32 bit unsigned integer

void ser16(const uint16_t val, ByteVector &data);

uint16_t deser16(const ByteVector &data, const size_t start);

int16_t deser_int16(const ByteVector &data, const size_t start);

void ser32(const uint32_t val, ByteVector &data);

uint32_t deser32(const ByteVector &data, const size_t start);

}

#endif // DESERIALISE_HPP
