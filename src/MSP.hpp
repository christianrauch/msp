#ifndef MSP_HPP
#define MSP_HPP

#include "SerialPort.hpp"
#include "msp_msg.h"

#include <iostream>

class MSP {
public:
    typedef std::vector<uint8_t> ByteVector;

    MSP(const std::string &device);

    ByteVector compileRequest(const uint8_t request_id);

    bool sendRequest(const uint8_t request_id);

    std::tuple<ByteVector, uint8_t> readData();

    //template<typename T>
    void unpack(const ByteVector &data, msp::Response &msg) {
        //assert(data.size()==sizeof(T));
        //std::memcpy(&msp_type, data.data(), sizeof(T));
        msg.decode(data);
    }

    template<typename T>
    void pack(const T &msp_type, ByteVector &data) {
        // allocate memory
        data.resize(sizeof(T));
        // copy to byte vector
        std::memcpy(data.data(), &msp_type, data.size());
    }

private:
    SerialPort sp;
};

#endif // MSP_HPP
