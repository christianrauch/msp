#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <asio.hpp>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include "ByteVector.hpp"
#include "FirmwareVariants.hpp"
#include "Message.hpp"
#include "Subscription.hpp"

namespace msp {
namespace client {

typedef asio::buffers_iterator<asio::streambuf::const_buffers_type> iterator;

enum LoggingLevel { SILENT, WARNING, INFO, DEBUG };

enum MessageStatus {
    OK,       // no errors
    FAIL_ID,  // message ID is unknown
    FAIL_CRC  // wrong CRC
};

struct ReceivedMessage {
    msp::ID id;
    ByteVector payload;
    MessageStatus status;
};

class Client {
public:
    /**
     * @brief Client Constructor
     * @param device String describing the path to the serial device
     * @param baudrate Baudrate of the connection (default 115200,
     * unnecessary for direct USB connection)
     */
    Client();

    /**
     * @brief ~Client Destructor
     */
    ~Client();

    /**
     * @brief Set the verbosity of the output
     * @param level LoggingLevel matching the desired amount of output (default
     * to WARNING)
     */
    void setLoggingLevel(const LoggingLevel& level);

    /**
     * @brief Change the device path on the next connect
     * @param ver Version of MSP to use
     * @return True if successful
     */
    bool setVersion(const int& ver);

    /**
     * @brief Query the cached device path
     * @return Cached path to device
     */
    int getVersion() const;

    /**
     * @brief Change the device path on the next connect
     * @param device Path to device
     */
    void setVariant(const FirmwareVariant& v);

    /**
     * @brief Query the cached device path
     * @return Cached path to device
     */
    FirmwareVariant getVariant() const;

    /**
     * @brief Start communications with a flight controller
     * @return True on success
     */
    bool start(const std::string& device, const size_t baudrate = 115200);

    /**
     * @brief Stop communications with a flight controller
     * @return True on success
     */
    bool stop();

    /**
     * @brief Query the system to see if a connection is active
     * @return true on success
     */
    bool isConnected() const;

    /**
     * @brief Send a message to the connected flight controller. If
     * the message sends data to the flight controller, it will be packed into
     * a buffer and sent. The method will block (optionally for a finite amount
     * of time) until a matching response is received from the flight
     * controller. If the response includes data, it will be unpacked back into
     * the same Message object.
     * @param message Reference to a Message-derived object to be sent/recieved.
     * @param timeout Maximum amount of time to block waiting for a response.
     * A value of 0 (default) means wait forever.
     */
    bool sendMessage(msp::Message& message, const double& timeout = 0);

    /**
     * @brief Send a message, but do not wait for any response
     * @param message Reference to a Message-derived object to be sent
     */
    bool sendMessageNoWait(const msp::Message& message);

    /**
     * @brief Register callback function that is called when a message of
     * matching ID is received
     * @param callback Pointer to callback function (class method)
     * @param context Object containing callback method
     * @param tp Period of timer that will send subscribed requests (in
     * seconds).
     * @return pointer to subscription that is added to internal list
     */
    template <typename T, typename C,
              class = typename std::enable_if<
                  std::is_base_of<msp::Message, T>::value>::type>
    std::shared_ptr<SubscriptionBase> subscribe(void (C::*callback)(const T&),
                                                C* context, const double& tp) {
        return subscribe<T>(std::bind(callback, context, std::placeholders::_1),
                            tp);
    }

    /**
     * @brief Register callback function that is called when a
     * message of matching ID is received
     * @param recv_callback Function to be called upon receipt of message
     * @param tp Period of timer that will send subscribed requests (in
     * seconds).
     * @return pointer to subscription that is added to internal list
     */
    template <typename T, class = typename std::enable_if<
                              std::is_base_of<msp::Message, T>::value>::type>
    std::shared_ptr<SubscriptionBase> subscribe(
        const std::function<void(const T&)>& recv_callback, const double& tp) {
        // validate the period
        if(!(tp >= 0.0)) throw std::runtime_error("Period must be positive!");

        // get the id of the message in question
        const msp::ID id = T(fw_variant).id();
        if(log_level_ >= INFO)
            std::cout << "SUBSCRIBING TO " << id << std::endl;

        // generate the callback for sending messages
        std::function<bool(const Message&)> send_callback =
            std::bind(&Client::sendMessageNoWait, this, std::placeholders::_1);

        // create a shared pointer to a new Subscription and set all properties
        auto subscription = std::make_shared<Subscription<T>>(
            recv_callback, send_callback, std::make_unique<T>(fw_variant), tp);

        // gonna modify the subscription map, so lock the mutex
        std::lock_guard<std::mutex> lock(mutex_subscriptions);

        // move the new subscription into the subscription map
        subscriptions.emplace(id, std::move(subscription));
        return subscriptions[id];
    }

    /**
     * @brief Check if message ID already has a subscription
     * @param id Message ID
     * @return True if there is already a matching subscription
     */
    bool hasSubscription(const msp::ID& id) const {
        return (subscriptions.count(id) == 1);
    }

    /**
     * @brief Get pointer to subscription
     * @param id Message ID
     * @return Shared pointer to subscription (empty if there was no match)
     */
    std::shared_ptr<SubscriptionBase> getSubscription(const msp::ID& id) {
        return subscriptions.at(id);
    }

    /**
     * @brief Main entry point for processing received data. It
     * is called directly by the ASIO library, and as such it much match the
     * function signatures expected by ASIO.
     * @param ec ASIO error code
     * @param bytes_transferred Number of byte available for processing
     */
    void processOneMessage(const asio::error_code& ec,
                           const std::size_t& bytes_transferred);

    /**
     * @brief Send an ID and payload to the flight controller
     * @param id Message ID
     * @param data Raw data (default zero length, meaning no data to send
     * outbound)
     * @return true on success
     */
    bool sendData(const msp::ID id, const ByteVector& data = ByteVector(0));

    /**
     * @brief Send an ID and payload to the flight controller
     * @param id Message ID
     * @param data Unique pointer to data. May be empty if there is no data to
     * send
     * @return true on success
     */
    bool sendData(const msp::ID id, const ByteVectorUptr&& data) {
        if(!data) return sendData(id);
        return sendData(id, *data);
    }

protected:
    /**
     * @brief Establish connection to serial device and start read thread
     * @return True on success
     */
    bool connectPort(const std::string& device, const size_t baudrate = 115200);

    /**
     * @brief Break connection to serial device and stop read thread
     * @return True on success
     */
    bool disconnectPort();

    /**
     * @brief Starts the receiver thread that handles incomming messages
     * @return True on success
     */
    bool startReadThread();

    /**
     * @brief Stops the receiver thread
     * @return True on success
     */
    bool stopReadThread();

    /**
     * @brief Starts the receiver thread that handles incomming messages
     * @return True on success
     */
    bool startSubscriptions();

    /**
     * @brief Stops the receiver thread
     * @return True on success
     */
    bool stopSubscriptions();

    /**
     * @brief Read a single byte from either the buffer or the serial device
     * @return byte from buffer or device
     */
    uint8_t extractChar();

    /**
     * @brief messageReady Method used by ASIO library to determine if a
     * full message is present in receiving buffer. It must match the function
     * signature expected by ASIO.
     * @return std::pair<iterator, bool> indicating where the start the next
     * message check operation and whether the current check was successful
     */
    std::pair<iterator, bool> messageReady(iterator begin, iterator end) const;

    /**
     * @brief processOneMessageV1 Iterates over characters in the ASIO buffer
     * to identify and unpack a MSPv1 encoded message
     * @return ReceivedMessage data structure containing the results of
     * unpacking
     */
    ReceivedMessage processOneMessageV1();

    /**
     * @brief processOneMessageV2 Iterates over characters in the ASIO buffer
     * to identify and unpack a MSPv2 encoded message
     * @return ReceivedMessage data structure containing the results of
     * unpacking
     */
    ReceivedMessage processOneMessageV2();

    /**
     * @brief packMessageV1 Packs data ID and data payload into a MSPv1
     * formatted buffer ready for sending to the serial device
     * @param id msp::ID of the message being packed
     * @param data Optional binary payload to be packed into the outbound buffer
     * @return ByteVector of full MSPv1 message ready for sending
     */
    ByteVector packMessageV1(const msp::ID id,
                             const ByteVector& data = ByteVector(0)) const;

    /**
     * @brief crcV1 Computes a checksum for MSPv1 messages
     * @param id uint8_t MSP ID
     * @param data Payload which is also part of the checksum
     * @return uint8_t checksum
     */
    uint8_t crcV1(const uint8_t id, const ByteVector& data) const;

    /**
     * @brief packMessageV2 Packs data ID and data payload into a MSPv2
     * formatted buffer ready for sending to the serial device
     * @param id msp::ID of the message being packed
     * @param data Optional binary payload to be packed into the outbound buffer
     * @return ByteVector of full MSPv2 message ready for sending
     */
    ByteVector packMessageV2(const msp::ID id,
                             const ByteVector& data = ByteVector(0)) const;

    /**
     * @brief crcV2 Computes a checksum for MSPv2 messages
     * @param crc Checksum value from which to start calculations
     * @param data ByteVector of data to be wrapped into the checksum
     * @return uint8_t checksum
     */
    uint8_t crcV2(uint8_t crc, const ByteVector& data) const;

    /**
     * @brief crcV2 Computes a checksum for MSPv2 messages
     * @param crc Checksum value from which to start calculations
     * @param data Single byte to use in the checksum calculation
     * @return uint8_t checksum
     */
    uint8_t crcV2(uint8_t crc, const uint8_t& b) const;

protected:
    asio::io_service io;     ///<! io service
    asio::serial_port port;  ///<! port for serial device
    asio::streambuf buffer;

    // read thread management
    std::thread thread;
    std::atomic_flag running_ = ATOMIC_FLAG_INIT;

    // thread safety and synchronization
    std::condition_variable cv_response;
    std::mutex cv_response_mtx;
    std::mutex mutex_response;
    std::mutex mutex_buffer;
    std::mutex mutex_send;

    // holder for received data
    std::unique_ptr<ReceivedMessage> request_received;

    // subscription management
    std::mutex mutex_subscriptions;
    std::map<msp::ID, std::shared_ptr<SubscriptionBase>> subscriptions;

    // debugging
    LoggingLevel log_level_;

    // reference values
    int msp_ver_;
    FirmwareVariant fw_variant;
};

}  // namespace client
}  // namespace msp

#endif  // CLIENT_HPP
