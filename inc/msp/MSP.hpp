#ifndef MSP_HPP
#define MSP_HPP

#include "SerialPort.hpp"
#include "types.hpp"

#include <stdexcept>
#include <chrono>

namespace msp {

// exception to throw when header contains wrong data
class MalformedHeader : public std::runtime_error {
public:
    MalformedHeader() : std::runtime_error("Malformed header") {}
};

class UnknownMsgId : public std::runtime_error {
public:
    UnknownMsgId(uint8_t id)
        : runtime_error("Unknown MSP id!"),
          id(id)
    { }

    virtual const char* what() throw() {
        std::stringstream ss_msg;
        ss_msg << runtime_error::what() << ": ";
        ss_msg << "FC refused to process message with id: "<<(uint)id;

        msg = ss_msg.str();
        return msg.c_str();
    }
private:
    uint8_t id;
    std::string msg;        ///<! error message
};

// exception to throw if reported CRC does not match with computed
class WrongCRC : public std::runtime_error {
public:
    WrongCRC(const msp::ID msg_id, const uint8_t exp, const uint8_t rcv)
        : std::runtime_error("CRC not matching"),
          msg_id(msg_id),
          expected(exp),
          received(rcv)
    { }

    virtual const char* what() throw() {
        std::stringstream ss_msg;
        ss_msg << runtime_error::what() << ": ";
        ss_msg << "Message " << (uint)msg_id << " ";
        ss_msg << "expected CRC " << (int)expected << ", ";
        ss_msg << "received CRC " << (int)received;

        msg = ss_msg.str();
        return msg.c_str();
    }
private:
    const msp::ID msg_id;   ///<! ID of message
    const uint8_t expected; ///<! expected CRC
    const uint8_t received; ///<! received CRC
    std::string msg;        ///<! error message
};

class NoData : public std::runtime_error {
public:
    NoData() : std::runtime_error("No data available!") { }
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

/**
 * @brief The MSP class
 */
class MSP {
public:
    /**
     * @brief MSP constructor msp communication
     * @param device device path
     */
    MSP(const std::string &device);

    /**
     * @brief request send command and request data from FC once
     * @param request request message
     * @return true on success
     * @return false on failure
     */
    bool request(msp::Request &request);

    /**
     * @brief request_block continuously send command and request data until data has been received
     * @param request request message
     * @return true when data has been received
     */
    bool request_block(msp::Request &request);

    /**
     * @brief request_timeout wait for data while continuously sending command
     * @param request request message
     * @param timeout_ms timeout in milliseconds until resending request
     * @return true when data has been received
     */
    bool request_timeout(msp::Request &request, unsigned int timeout_ms);

    /**
     * @brief respond send data to FC and read acknowledge
     * @param response response message
     * @return true on success
     * @return false on failure
     */
    bool respond(const msp::Response &response);

    /**
     * @brief respond_block send data to FC until acknowledge has been received
     * @param response response message with data
     * @return true when acknowledge has been received
     */
    bool respond_block(const msp::Response &response);

    /**
     * @brief sendData send raw data and ID to flight controller
     * @param id message ID
     * @param data raw data
     * @return true on success
     * @return false on failure
     */
    bool sendData(const ID id, const ByteVector &data = ByteVector(0));

    /**
     * @brief send encode message and send payload
     * @param response message sent to FC
     * @return true on success
     * @return false on failure
     */
    bool send(const msp::Response &response) {
        return sendData(response.id(), response.encode());
    }

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
    uint8_t crc(const ID id, const ByteVector &data);

    SerialPort sp;      //!< serial port
    unsigned int wait;  //!< time (micro seconds) to wait before waiting for response
};

} // namespace msp

#endif // MSP_HPP
