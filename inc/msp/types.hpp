#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <string>
#include <stdint.h>
#include "msp_id.hpp"


#include <iostream>

namespace msp {

/**
 * @brief ByteVector vector of bytes
 */

class ByteVector : public std::vector<uint8_t>
{
public:
    ByteVector() : offset(0) {};
    
    template<typename T1>
    ByteVector(T1 arg1) : std::vector<uint8_t>(arg1), offset(0)
    {}
    
    template<typename T1, typename T2>
    ByteVector(T1 arg1, T2 arg2) : std::vector<uint8_t>(arg1,arg2), offset(0)
    {}
    
    void pack(const uint8_t val) {
        this->push_back(val);
    }
    
    void pack(const uint16_t val) {
        this->push_back(val>>0);
        this->push_back(val>>8);
    }
    
    void pack(const uint32_t val) {
        this->push_back(val>>0);
        this->push_back(val>>8);
        this->push_back(val>>16);
        this->push_back(val>>24);
    }
    void pack(const int8_t val) {
        this->push_back(val);
    }
    
    void pack(const int16_t val) {
        this->push_back(val>>0);
        this->push_back(val>>8);
    }
    
    void pack(const int32_t val) {
        this->push_back(val>>0);
        this->push_back(val>>8);
        this->push_back(val>>16);
        this->push_back(val>>24);
    }
    
    void pack(std::string val, size_t max_len = 0) {
        size_t count = 0;
        for (auto c : val) {
            this->push_back(c);
            if (max_len && (++count == max_len)) break;
        }
    }
    
    bool unpack(bool& val) {
        if (this->size() - offset < 1) return false;
        val = (*this)[offset++];
        return true;
    }
    
    bool unpack(uint8_t& val) {
        if (this->size() - offset < 1) return false;
        val = (*this)[offset++];
        return true;
    }
    
    bool unpack(uint16_t& val) {
        if (this->size() - offset < 2) return false;
        val = ((*this)[offset]<<0) | ((*this)[offset +1]<<8);
        offset += 2;
        return true;
    }
    
    bool unpack(uint32_t& val) {
        if (this->size() - offset < 4) return false;
        val = ((*this)[offset]<<0) | ((*this)[offset+1]<<8) | ((*this)[offset+2]<<16) | ((*this)[offset+3]<<24);
        offset += 4;
        return true;
    }
    
    bool unpack(int8_t& val)  {
        if (this->size() - offset < 1) return false;
        val = (*this)[offset++];
        return true;
    }
    
    bool unpack(int16_t& val) {
        if (this->size() - offset < 2) return false;
        val = ((*this)[offset]<<0) | ((*this)[offset +1]<<8);
        offset += 2;
        return true;
    }
    
    bool unpack(int32_t& val) {
        if (this->size() - offset < 4) return false;
        val = reinterpret_cast<int32_t>(((*this)[offset]<<0) | ((*this)[offset+1]<<8) | ((*this)[offset+2]<<16) | ((*this)[offset+3]<<24));
        offset += 4;
        return true;
    }
    
    bool unpack(std::string& val, size_t count) {
        bool rc = true;
        val.clear();
        int8_t tmp;
        for (uint i = 0; i < count; ++i) {
            rc &= unpack(tmp);
            val += tmp;
        }
        return rc;
    }

protected:
    std::size_t offset;

};


//typedef std::vector<uint8_t> ByteVector;


    
/////////////////////////////////////////////////////////////////////
/// Generic message types

struct Message {
    virtual ID id() const = 0;

    virtual ~Message() { }
};

// send to FC
struct Request : public Message {
    virtual bool decode(ByteVector &data) = 0;
};

// received from FC
struct Response : public Message {
    virtual ByteVector encode() const = 0;
};

} // namespace msp

#endif // TYPES_HPP
