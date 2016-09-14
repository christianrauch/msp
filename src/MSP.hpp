#ifndef MSP_HPP
#define MSP_HPP

#include "SerialPort.hpp"
#include "msp_msg.hpp"

#include <stdexcept>

namespace msp {

// exception to throw when header contains wrong data
class MalformedHeader : public std::runtime_error {
public:
    MalformedHeader() : std::runtime_error("Malformed header") {}
};

// exception to throw if reported CRC does not match with computed
class WrongCRC : public std::runtime_error {
public:
    WrongCRC() : std::runtime_error("CRC not matching") {}
};

/**
 * @brief The DataID struct
 */
struct DataID {
    ByteVector data;    //!< rawdata vector
    ID id;              //!< message ID

    /**
     * @brief DataID
     * @param data vector of raw data bytes
     * @param id message ID
     */
    DataID(ByteVector data, ID id) : data(data), id(id) {}
};

class MSP {
public:
    MSP(const std::string &device);

    // request (get) data from FC
    bool request(msp::Request &request);

    // respond (set) with data to FC
    bool respond(msp::Response &response);

    /**
     * @brief sendData send raw data and ID to flight controller
     * @param id message ID
     * @param data raw data
     * @return true on success
     * @return false on failure
     */
    bool sendData(const uint8_t id, const ByteVector &data = ByteVector(0));

    /**
     * @brief receiveData receive raw data from flight controller
     * @return pair of data and message ID
     */
    DataID receiveData();

    /**
     * @brief setWait set time (microseconds) between sending and receiving
     * After sending a request to the FC, we need to wait a small amount of time
     * for the FC to process our request and to respond.
     *
     * @param wait_us waiting time in microseconds
     */
    void setWait(unsigned int wait_us) {
        wait = wait_us;
    }

private:
    /**
     * @brief crc compute checksum of data package
     * @param id message ID
     * @param data raw data vector
     * @return checksum
     */
    uint8_t crc(const uint8_t id, const ByteVector &data);

    SerialPort sp;      //!< serial port
    unsigned int wait;  //!< time (micro seconds) to wait before waiting for response
};



} // namespace msp

#endif // MSP_HPP
