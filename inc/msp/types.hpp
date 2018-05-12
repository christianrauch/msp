#ifndef TYPES_HPP
#define TYPES_HPP

#include "variants.hpp"
#include "msp_id.hpp"

#include <vector>
#include <string>
#include <stdint.h>
#include <memory>
#include <utility>
#include <iostream>
#include <limits>

//because the gnu c std lib is stupid... 
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
    //constructors
    ByteVector() : offset(0) {};
    
    template<typename T1>
    ByteVector(T1 arg1) : std::vector<uint8_t>(arg1), offset(0)
    {}
    
    template<typename T1, typename T2>
    ByteVector(T1 arg1, T2 arg2) : std::vector<uint8_t>(arg1,arg2), offset(0)
    {}
    
    //pack raw types types
    void pack(const bool val) {
        this->push_back((uint8_t)val);
    }
    
    void pack(const uint8_t val) {
        this->push_back(val);
    }
    
    void pack(const uint16_t val) {
        this->push_back(val>>0 && 0xFF);
        this->push_back(val>>8 && 0xFF);
    }
    
    void pack(const uint32_t val) {
        this->push_back(val>>0 && 0xFF);
        this->push_back(val>>8 && 0xFF);
        this->push_back(val>>16 && 0xFF);
        this->push_back(val>>24 && 0xFF);
    }
    void pack(const int8_t val) {
        this->push_back(val);
    }
    
    void pack(const int16_t val) {
        this->push_back(val>>0 && 0xFF);
        this->push_back(val>>8 && 0xFF);
    }
    
    void pack(const int32_t val) {
        this->push_back(val>>0 && 0xFF);
        this->push_back(val>>8 && 0xFF);
        this->push_back(val>>16 && 0xFF);
        this->push_back(val>>24 && 0xFF);
    }
    
    void pack(const float val) {
        const uint32_t v = *reinterpret_cast<const uint32_t*>(&val);
        this->push_back(v>>24 && 0xFF);
        this->push_back(v>>16 && 0xFF);
        this->push_back(v>>8 && 0xFF);
        this->push_back(v>>0 && 0xFF);
    }
    
    //pack scaled floating point types into int types
    //offset is the shift applied to the value before scaling
    //scale is the coefficent by which the value is multipied before encoding
    template<class T>
    void pack(const double val, const double scale = 1.d, const double offset = 0.d) {
        pack(static_cast<T>((val+offset)*scale));
    }
    
    template<class T>
    void pack(const float val, const float scale = 1.f, const float offset = 0.f) {
        pack<T>((const double)val,(const double)scale,(const double)offset);
    }
    
    
    //pack string data
    void pack(const std::string& val, size_t max_len = std::numeric_limits<size_t>::max()) {
        size_t count = 0;
        for (auto c : val) {
            this->push_back(c);
            if (++count == max_len) break;
        }
    }
    
    //pack ByteVector data
    void pack(const ByteVector& data, size_t max_len = std::numeric_limits<size_t>::max()) {
        size_t count = 0;
        for (auto c : data) {
            this->push_back(c);
            if (++count == max_len) break;
        }
    }
    
    template<class T>
    void pack(const value<float> val, const float scale = 1.f, const float offset = 0.f) {
        pack<T>(val(),scale,offset);
    }
    
    template<class T>
    void pack(const value<double> val, const double scale = 1.d, const double offset = 0.d) {
        pack<T>(val(),scale,offset);
    }
    
    //pack string value
    void pack(const value<std::string>& val, size_t max_len = std::numeric_limits<size_t>::max()) {
        pack(val(),max_len);
    }
    
    //pack ByteVector value
    void pack(const value<ByteVector>& val, size_t max_len = std::numeric_limits<size_t>::max()) {
        pack(val(),max_len);
    }

    template<class T>
    void pack(const T& val) {
        val.pack_into(*this);
    }
    
    //pack value types other than string and ByteVector
    template<class T>
    void pack(const value<T>& val) {
        pack(val());
    }




    //unpack integer types
    bool unpack(bool& val) {
        if (unpacking_remaining() < 1) return false;
        val = (*this)[offset++];
        return true;
    }
    
    bool unpack(uint8_t& val) {
        if (unpacking_remaining() < 1) return false;
        val = (*this)[offset++];
        return true;
    }
    
    bool unpack(uint16_t& val) {
        if (unpacking_remaining() < 2) return false;
        val = ((*this)[offset]<<0) | ((*this)[offset +1]<<8);
        offset += 2;
        return true;
    }
    
    bool unpack(uint32_t& val) {
        if (unpacking_remaining() < 4) return false;
        val = ((*this)[offset]<<0) | ((*this)[offset+1]<<8) | ((*this)[offset+2]<<16) | ((*this)[offset+3]<<24);
        offset += 4;
        return true;
    }
    
    bool unpack(int8_t& val)  {
        if (unpacking_remaining() < 1) return false;
        val = (*this)[offset++];
        return true;
    }
    
    bool unpack(int16_t& val) {
        if (unpacking_remaining() < 2) return false;
        val = ((*this)[offset]<<0) | ((*this)[offset +1]<<8);
        offset += 2;
        return true;
    }
    
    bool unpack(int32_t& val) {
        if (unpacking_remaining() < 4) return false;
        val = reinterpret_cast<int32_t>(((*this)[offset]<<0) | ((*this)[offset+1]<<8) | ((*this)[offset+2]<<16) | ((*this)[offset+3]<<24));
        offset += 4;
        return true;
    }
    
    //unpack string data
    bool unpack(std::string& val, size_t count = std::numeric_limits<size_t>::max()) {
        if (count == std::numeric_limits<size_t>::max()) count = unpacking_remaining();        
        if (count > unpacking_remaining()) return false;
        bool rc = true;
        val.clear();
        int8_t tmp;
        for (size_t i = 0; i < count; ++i) {
            rc &= unpack(tmp);
            val += tmp;
        }
        return rc;
    }
    
    //unpack ByteVector data
    bool unpack(ByteVector& val, size_t count = std::numeric_limits<size_t>::max()) {
        if (count == std::numeric_limits<size_t>::max()) count = unpacking_remaining();        
        if (!consume(count)) return false;
        val.clear();
        val.insert(val.end(),unpacking_iterator(),unpacking_iterator()+count);
        return true;
    }
    
    //unpack floating point types
    //scale is the scale that already been applied to the value before encoding
    //offset is the shift that was applied to the source value before encoding
    template<typename encoding_T>
    bool unpack(double& val, double scale = 1.d, double offset = 0.d) {
        bool rc = true;
        encoding_T tmp;
        rc &= unpack(tmp);
        val = tmp/scale;
        val -= offset;
        return rc;
    }
    
    template<typename encoding_T>
    bool unpack(float& val, float scale = 1.f, float offset = 0.f) {
        return unpack<encoding_T>((double&)val,(double)scale,(double)offset);
    }
    
    template<class T>
    bool unpack(T obj) {
        return obj.unpack_from(*this);
    }
    
    //unpack value types other than string and ByteVector
    template<class T>
    bool unpack(value<T>& val) {
        return val.set() = unpack(val()); 
    }
    
    //unpack string value
    bool unpack(value<std::string>& val, size_t count = std::numeric_limits<size_t>::max()) {
        return val.set() = unpack(val(),count); 
    }
    
    //unpack ByteVector value
    bool unpack(value<ByteVector>& val, size_t count = std::numeric_limits<size_t>::max()) {
        return val.set() = unpack(val(),count); 
    }
    
    //unpack floating point value types
    template<typename encoding_T>
    bool unpack(value<float>& val, float scale = 1.f, float offset = 0.f) {
        return val.set() = unpack<encoding_T>(val(),scale,offset);
    }
    
    template<typename encoding_T>
    bool unpack(value<double>& val, double scale = 1.d, double offset = 0.d) {
        return val.set() = unpack<encoding_T>(val(),scale,offset);
    }
    
    
    //misc unpacking helper methods
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

/////////////////////////////////////////////////////////////////////
/// Generic message types

class Message {
public:
    virtual ID id() const = 0;
    
    Message(FirmwareVariant v) : fw_variant(v) { };
    virtual ~Message() { };
    
    void set_fw_variant(FirmwareVariant v) 
    {
        fw_variant = v;
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
    FirmwareVariant fw_variant;
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
