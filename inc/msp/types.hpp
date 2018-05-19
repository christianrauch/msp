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
#include <type_traits>

//because the gnu c std lib is stupid... 
#undef major
#undef minor

namespace msp {

template<class T>
class value {
public:
    value<T>& operator= (value<T>& rhs) {
        data.first = rhs();
        data.second = rhs.set();
        return *this;
    }
    /*
    value<T>& operator= (T& rhs) {
        data.first = rhs;
        data.second = true;
        return *this;
    }
    */
    value<T>& operator= (T rhs) {
        data.first = rhs;
        data.second = true;
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

struct Packable;
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
    
    
    
    template<typename T, typename std::enable_if<std::is_integral<T>::value, T>::type* = nullptr> 
    bool pack(const T& val) {
        //little endian packing
        for (size_t i(0); i < sizeof(val) ; ++i) {
            this->push_back( val>>(i*8) & 0xFF);
        }
        return true;
    }
    
    template<typename T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    bool pack(const T& val) {
        ByteVector b(sizeof(val));
        reinterpret_cast<T&>(b.front()) = val;
        return this->pack(b);
    }
    
    //pack scaled floating point types into int types
    //offset is the shift applied to the value before scaling
    //scale is the coefficent by which the value is multipied before encoding
    template< typename encoding_T, typename T1, typename T2, 
                typename std::enable_if<std::is_arithmetic<T1>::value, T1>::type* = nullptr,
                typename std::enable_if<std::is_arithmetic<T2>::value, T2>::type* = nullptr >
    bool pack(const T1 val, const T2 scale, const T2 offset = 0) {
        auto tmp = (val+offset)*scale;
        if (tmp <= std::numeric_limits<encoding_T>::min()) return pack(std::numeric_limits<encoding_T>::min());
        else if (tmp >= std::numeric_limits<encoding_T>::max()) return pack(std::numeric_limits<encoding_T>::max());
        return pack(static_cast<encoding_T>(tmp));
    }
    
    
    //pack string data
    bool pack(const std::string& val, size_t max_len = std::numeric_limits<size_t>::max()) {
        size_t count = 0;
        for (auto c : val) {
            this->push_back(c);
            if (++count == max_len) break;
        }
        return true;
    }
    
    //pack ByteVector data
    bool pack(const ByteVector& data, size_t max_len = std::numeric_limits<size_t>::max()) {
        size_t count = 0;
        for (auto c : data) {
            this->push_back(c);
            if (++count == max_len) break;
        }
        return true;
    }
    
    template<typename T, typename std::enable_if<std::is_base_of<Packable,T>::value, T>::type* = nullptr>
    bool pack(const T& val) {
        return val.pack_into(*this);
    }
    
    
    template< typename encoding_T, typename T1, typename T2, 
                typename std::enable_if<std::is_arithmetic<T1>::value, T1>::type* = nullptr,
                typename std::enable_if<std::is_arithmetic<T2>::value, T2>::type* = nullptr >
    bool pack(const value<T1> val, const T2 scale, const T2 offset = 0) {
        if (!val.set()) return false;
        return pack<encoding_T>(val(),scale,offset);
    }
    
    //pack array type value
    bool pack(const value<ByteVector>& val, size_t max_len = std::numeric_limits<size_t>::max()) {
        if (!val.set()) return false;
        return pack(val(),max_len);
    }
    
    bool pack(const value<std::string>& val, size_t max_len = std::numeric_limits<size_t>::max()) {
        if (!val.set()) return false;
        return pack(val(),max_len);
    }
    
    //pack value types other than string and ByteVector
    template<class T>
    bool pack(const value<T>& val) {
        if (!val.set()) return false;
        return pack(val());
    }




    //unpack integer types
    template<typename T, typename std::enable_if<std::is_integral<T>::value, T>::type* = nullptr> 
    bool unpack(T& val) {
        if (unpacking_remaining() < sizeof(val)) return false;
        val = 0;
        for (size_t i(0); i < sizeof(val); ++i) {
            val |= (*this)[offset++]<<(8*i);
        }
        return true;
    }

    template<typename T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    bool unpack(T& val) {
        if (unpacking_remaining() < sizeof(val)) return false;
        val = reinterpret_cast<T&>((*this)[offset]);
        offset += sizeof(val);
        
        //int32_t bits = reinterpret_cast<int32_t>(((*this)[offset]<<24) | ((*this)[offset+1]<<16) | ((*this)[offset+2]<<8) | ((*this)[offset+3]<<0));
        //val = reinterpret_cast<float&>(bits);
        //offset += 4;
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
    template< typename encoding_T, typename T1, typename T2, 
                typename std::enable_if<std::is_arithmetic<T1>::value, T1>::type* = nullptr,
                typename std::enable_if<std::is_arithmetic<T2>::value, T2>::type* = nullptr >
    bool unpack(T1& val, T2 scale, T2 offset = 0) {
        bool rc = true;
        encoding_T tmp;
        rc &= unpack(tmp);
        val = tmp/scale;
        val -= offset;
        return rc;
    }
    
    template<typename T, typename std::enable_if<std::is_base_of<Packable,T>::value, T>::type* = nullptr>
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
    
    bool unpack(value<ByteVector>& val, size_t count = std::numeric_limits<size_t>::max()) {
        return val.set() = unpack(val(),count); 
    }
    
    //unpack floating point value types
    template< typename encoding_T, typename T1, typename T2, 
                typename std::enable_if<std::is_arithmetic<T1>::value, T1>::type* = nullptr,
                typename std::enable_if<std::is_arithmetic<T2>::value, T2>::type* = nullptr >
    bool unpack(value<T1>& val, T2 scale = 1, T2 offset = 0) {
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

struct Packable {
    virtual bool pack_into(ByteVector &data) const = 0;
    virtual bool unpack_from(ByteVector &data) = 0;
};

typedef std::shared_ptr<ByteVector> ByteVector_ptr;
typedef std::unique_ptr<ByteVector> ByteVector_uptr;
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
    
    virtual ByteVector_uptr encode() const
    {
        return ByteVector_uptr();
    };
    
    virtual std::ostream& print(std::ostream& s) const
    {
        return s;
    };
    
protected:
    FirmwareVariant fw_variant;
};


} // namespace msp


std::ostream& operator<<(std::ostream& s, const msp::Message& val) {
    return val.print(s);
};


template<class T>
std::ostream& operator<<(std::ostream& s, const msp::value<T>& val) {
    if ( val.set() )
        s << val();
    else
        s << "<unset>";
    return s;
};



#endif // TYPES_HPP
