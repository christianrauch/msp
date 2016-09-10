#ifndef MSP_HPP
#define MSP_HPP

#include "SerialPort.hpp"
#include "msp_msg.hpp"

#include <iostream>

class MSP {
public:
    typedef std::vector<uint8_t> ByteVector;

    MSP(const std::string &device);

    // request (get) data from FC
    msp::Request &request();

    // respond (set) with data FC
    bool response(msp::Response &response);

    bool sendData(const uint8_t id, const ByteVector &data = ByteVector(0));

    ByteVector receiveData(const uint8_t id);

    //ByteVector compileRequest(const uint8_t request_id);

    //bool sendRequest(const uint8_t request_id);

    uint8_t crc(const uint8_t id, const ByteVector &data);

private:
    SerialPort sp;
};

#endif // MSP_HPP
