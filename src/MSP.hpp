#ifndef MSP_HPP
#define MSP_HPP

#include "SerialPort.hpp"
#include "msp_msg.hpp"

#include <iostream>
#include <stdexcept>

namespace msp {

class MalformedHeader : public std::runtime_error {
public:
    MalformedHeader() : std::runtime_error("malformed header") {}
};

class WrongMessageType : public std::runtime_error {
public:
    WrongMessageType() : std::runtime_error("Wrong message id") {}
};

class NoData : public std::runtime_error {
public:
    NoData() : std::runtime_error("No data") {}
};

class WrongCRC : public std::runtime_error {
public:
    WrongCRC() : std::runtime_error("CRC not matching") {}
};


class MSP {
public:
    typedef std::vector<uint8_t> ByteVector;

    MSP(const std::string &device);

    // request (get) data from FC
    bool request(msp::Request &request);

    // respond (set) with data to FC
    bool respond(msp::Response &response);

    bool sendData(const uint8_t id, const ByteVector &data = ByteVector(0));

    ByteVector receiveData(const uint8_t id);

    void setWait(unsigned int wait_us) {
        wait = wait_us;
    }

private:
    uint8_t crc(const uint8_t id, const ByteVector &data);

    SerialPort sp;      // serial port
    unsigned int wait;  // time (micro seconds) to wait before waiting for response
};

} // namespace msp

#endif // MSP_HPP
