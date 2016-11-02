#include "deserialise.hpp"

namespace msp {

void ser16(const uint16_t val, ByteVector &data) {
    data.push_back(val>>0);
    data.push_back(val>>8);
}

uint16_t deser16(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8);
}

int16_t deser_int16(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8);
}

void ser32(const uint32_t val, ByteVector &data) {
    data.push_back(val>>0);
    data.push_back(val>>8);
    data.push_back(val>>16);
    data.push_back(val>>24);
}

uint32_t deser32(const ByteVector &data, const size_t start) {
    return (data[start]<<0) | (data[start+1]<<8) | (data[start+2]<<16) | (data[start+3]<<24);
}

} // namespace msp
