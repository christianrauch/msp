#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include "FirmwareVariants.hpp"
#include "msp_id.hpp"
#include "ByteVector.hpp"

#include <vector>
#include <string>
#include <memory>
#include <iostream>

namespace msp {

class Message {
public:
    /**
     * @brief get the ID of the message
     * @returns ID 
     */
    virtual ID id() const = 0;
    
    /**
     * @brief Message constructor
     */
    Message() {};
    
    /**
     * @brief Message constructor accepting a FirmwareVariant
     * @param v FirmwareVariant specifing which firmware this message should
     * tailor itself to.
     */
    Message(FirmwareVariant v) : fw_variant(v) {};
    
    /**
     * @brief Message destructor
     */
    virtual ~Message() { };
    
    /**
     * @brief Set the firmware the message should work with
     * @param v FirmwareVariant specifing which firmware this message should
     * tailor itself to.
     */
    void setFirmwareVariant(FirmwareVariant v) 
    {
        fw_variant = v;
    }
    
    /**
     * @brief Queries the firmware variant the message works with
     * @returns FirmwareVariant for this message
     */
    FirmwareVariant getFirmwareVariant() const
    {
        return fw_variant;
    }
    
    /**
     * @brief Decode message contents from a ByteVector
     * @param data Source of data
     * @returns False. Override methods should return true on success
     */
    virtual bool decode(ByteVector &data) 
    {
        return false;
    }
    
    /**
     * @brief Encode all data into a ByteVector
     * @returns Unique pointer to a ByteVector of data
     */
    virtual ByteVectorUptr encode() const
    {
        return ByteVectorUptr();
    }
    
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
}

#endif // TYPES_HPP
