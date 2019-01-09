#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include "variants.hpp"
#include "msp_id.hpp"
#include "byte_vector.hpp"

#include <vector>
#include <string>
#include <memory>
#include <iostream>




namespace msp {


class Message {
public:
    virtual ID id() const = 0;
    
    Message() {};
    Message(FirmwareVariant v) : fw_variant(v) {};
    virtual ~Message() { };
    
    void set_fw_variant(FirmwareVariant v) 
    {
        fw_variant = v;
    };
    
    FirmwareVariant get_fw_variant() const
    {
        return fw_variant;
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

/*
std::ostream& operator<<(std::ostream& s, const msp::Message& val) {
    return val.print(s);
}
*/



#endif // TYPES_HPP
