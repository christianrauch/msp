#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <string>
#include <stdint.h>
#include "msp_id.hpp"
#include <map>
#include <memory>
#include <utility>
#include <iostream>

#undef major
#undef minor

namespace msp {

template<class T>
class value {
public:
    value& operator= (T rhs) {
        data.first = rhs;
        data.second = true;
        return *this;
    }
    
    value& operator= (value rhs) {
        data.first = rhs.first;
        data.second = rhs.second;
        return *this;
    }
    
    T& operator() () {
        return data.first;
    }
    
    T operator() () const {
        return data.first;
    }
    
    bool set() const {
        return data.second;
    }
    
    bool& set() {
        return data.second;
    }
private:
    std::pair<T,bool> data;
};

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
    
    void pack(const std::string val, size_t max_len = 0) {
        size_t count = 0;
        for (auto c : val) {
            this->push_back(c);
            if (max_len && (++count == max_len)) break;
        }
    }
    
    void pack(const ByteVector data, size_t max_len = 0) {
        size_t count = 0;
        for (auto c : data) {
            this->push_back(c);
            if (max_len && (++count == max_len)) break;
        }
    }
    
    template<class T>
    void pack(const value<T> val) {
        pack(val());
    }
    
    void pack(const value<std::string> val, size_t max_len = 0) {
        pack(val(),max_len);
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
    
    template<class T>
    bool unpack(value<T>& val) {
        return val.set() = unpack(val()); 
    }
    
    bool unpack(std::string& val, size_t count = 0) {
        bool rc = true;
        val.clear();
        int8_t tmp;
        if (count) count = val.size();
        for (size_t i = 0; i < count; ++i) {
            rc &= unpack(tmp);
            val += tmp;
        }
        return rc;
    }
    
    bool unpack(value<std::string>& val, size_t count = 0) {
        return val.set() = unpack(val(),count); 
    }
    
    template<typename encoding_T>
    bool unpack(float& val, float scale = 1.f) {
        bool rc = true;
        encoding_T tmp;
        rc &= unpack(tmp);
        val = tmp*scale;
        return rc;
    }
    
    template<typename encoding_T>
    bool unpack(double& val, double scale = 1.d) {
        bool rc = true;
        encoding_T tmp;
        rc &= unpack(tmp);
        val = tmp*scale;
        return rc;
    }
    
    
    template<typename encoding_T>
    bool unpack(value<float>& val, float scale = 1.f) {
        return val.set() = unpack<encoding_T>(val(),scale);
    }
    
    template<typename encoding_T>
    bool unpack(value<double>& val, double scale = 1.f) {
        return val.set() = unpack<encoding_T>(val(),scale);
    }
    
    std::size_t unpacking_offset() {
        return offset;
    }
    
    std::vector<uint8_t>::iterator unpacking_iterator() {
        return this->begin()+offset;
    }
    
    bool consume(std::size_t count) {
        if (count < -offset) return false;
        if (count > this->size() - offset) return false;
        offset += count;
        return true;
    }
    
    std::size_t unpacking_remaining() {
        return this->size() - offset;
    }

protected:
    std::size_t offset;

};

enum class variant : int {
    MWII = 1,
    BAFL = 2,
    BTFL = 3,
    CLFL = 4,
    INAV = 5,
    RCFL = 6
};

std::map<std::string,variant> variant_map = 
{
    {"MWII", variant::MWII}, 
    {"BAFL", variant::BAFL}, 
    {"BTFL", variant::BTFL}, 
    {"CLFL", variant::CLFL}, 
    {"INAV", variant::INAV}, 
    {"RCFL", variant::RCFL}
};
    
/////////////////////////////////////////////////////////////////////
/// Generic message types

class Message {
public:
    virtual ID id() const = 0;
    
    Message(variant v = variant::INAV) : variant_(v) { };
    virtual ~Message() { };
    
    void set_variant(variant v) 
    {
        variant_ = v;
    };
    
    virtual bool decode(ByteVector &data) 
    {
        return false;
    };
    
    virtual ByteVector encode() const 
    {
        return ByteVector();
    };
    
protected:
    variant variant_;
};


} // namespace msp

template<class T>
std::ostream& operator<<(std::ostream& s, msp::value<T> val) {
    if ( val.set() )
        s << val();
    else
        s << "<unset>";
    
    return s;
};



#endif // TYPES_HPP
