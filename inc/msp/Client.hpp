#ifndef CLIENT_HPP
#define CLIENT_HPP

#include "byte_vector.hpp"
#include "variants.hpp"
#include "message.hpp"
#include "msp_id.hpp"
//#include "periodic_timer.hpp"
#include "subscription.hpp"

#include <string>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <map>

#include <asio.hpp>

//struct SerialPortImpl;

namespace msp {
namespace client {

typedef asio::buffers_iterator<asio::streambuf::const_buffers_type> iterator;

enum MessageStatus {
    OK,         // no errors
    FAIL_ID,    // message ID is unknown
    FAIL_CRC    // wrong CRC
};

struct ReceivedMessage {
    uint32_t id;
    ByteVector data;
    MessageStatus status;
};

class Client {
public:
    Client();

    ~Client();
    
    void printWarnings(const bool& val = true);
    void printDebug(const bool& val = true);
    
    bool setVersion(int ver);
    int getVersion();
    
    void setVariant(FirmwareVariant v);
    FirmwareVariant getVariant();

    /**
     * @brief connect establish connection to serial device
     * @param device path or name of serial device
     * @param baudrate serial baudrate (default: 115200)
     * @return true on success
     */
    void connect(const std::string &device, const size_t baudrate=115200);


    //synchronous message (wait for ack or response)
    bool sendMessage(msp::Message& message, const double timeout = 0);
    //bool sendMessage(msp::Message* message, const double timeout = 0);
    /*
    template<typename T, typename C>
    bool asyncSendMessage(T& message, void (C::*callback)(T&), C *context, const double timeout = 0); 
    
    template<typename T>
    bool asyncSendMessage(T& message, std::function<void(T&)>, const double timeout = 0); 
    */
    bool asyncSendMessage(msp::Message& message);
    
    std::pair<iterator, bool> messageReady(iterator begin, iterator end);



    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback pointer to callback function (class method)
     * @param context class with callback method
     * @param tp period of timer that will send subscribed requests (in seconds), by default this is 0 and requests are not sent periodically
     * @return pointer to subscription that is added to internal list
     */
    template<typename T, typename C>
    std::shared_ptr<SubscriptionBase> subscribe(void (C::*callback)(T&), C *context, const double tp = 0.0) {
        return subscribe<T>(std::bind(callback, context, std::placeholders::_1), tp);
    }

    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback function (e.g. lambda, class method, function pointer)
     * @param tp period of timer that will send subscribed requests (in seconds), by default this is 0 and requests are not sent periodically
     * @return pointer to subscription that is added to internal list
     */
    template<typename T>
    std::shared_ptr<SubscriptionBase> subscribe(const std::function<void(T&)> &recv_callback, const double tp = 0.0) {

        if(!std::is_base_of<msp::Message, T>::value)
            throw std::runtime_error("Callback parameter needs to be of Request type!");

        if(!(tp>=0.0))
            throw std::runtime_error("Period must be positive!");

        const msp::ID id = T(fw_variant).id();
        std::cout << "SUBSCRIBING TO " << (uint32_t)id << std::endl;
        
        
        //generate the callback for sending messages
        std::function<void(T&)> send_callback = std::bind( &Client::asyncSendMessage, this, std::placeholders::_1);
        //create a shared pointer to a new Subscription and set all properties
        auto subscription = std::make_shared<Subscription<T>>(  );
        subscription->setReceiveCallback(recv_callback);
        subscription->setSendCallback(send_callback);
        subscription->setIoObject( std::make_unique<T>(fw_variant) );
        subscription->setTimerPeriod(tp);
        //gonna modify the subscription map, so lock the mutex
        std::lock_guard<std::mutex> lock(mutex_subscriptions);
        //move the new subscription into the subscription map
        subscriptions.emplace(id,std::move(subscription));

        return subscriptions[id];
    }

    /**
     * @brief hasSubscription check if message ID is subscribed
     * @param id message ID
     * @return true if there is already a subscription
     * @return false if ID is not subscribed
     */
    bool hasSubscription(const msp::ID& id) {
        return (subscriptions.count(id)==1);
    }

    /**
     * @brief getSubscription get pointer to subscription
     * @param id message ID
     * @return pointer to subscription
     */
    std::shared_ptr<SubscriptionBase> getSubscription(const msp::ID& id) {
        return subscriptions.at(id);
    }

    void processOneMessage(const asio::error_code& ec,std::size_t bytes_transferred);

    void startRead();


    /**
     * @brief start starts the receiver thread that handles incomming messages
     */
    void start();

    /**
     * @brief stop stops the receiver thread
     */
    void stop();

    /**
     * @brief read blocking read a single byte from either the buffer or the serial device
     * @return byte from buffer or device
     */
    uint8_t read();

    /**
     * @brief sendData send raw data and ID to flight controller, accepts any uint8 id
     * @param id message ID
     * @param data raw data
     * @return true on success
     * @return false on failure
     */
    
    bool sendData(const msp::ID id, const ByteVector &data = ByteVector(0));
    bool sendData(const msp::ID id, const ByteVector_uptr data)
    {
        //if (!data) std::cout << "ByteVector_uptr is empty" << std::endl;
        if (!data) return sendData(id);
        //std::cout << "sending data from ByteVector_uptr: " << *data << std::endl;
        return sendData(id,*data);
    }
    
    

protected:
   
    ReceivedMessage processOneMessageV1();
    ReceivedMessage processOneMessageV2();

    ByteVector packMessageV1(const msp::ID id, const ByteVector &data = ByteVector(0));
    uint8_t crcV1(const uint8_t id, const ByteVector &data);

    ByteVector packMessageV2(const msp::ID id, const ByteVector &data = ByteVector(0));
    uint8_t crcV2(uint8_t crc, const ByteVector &data);
    uint8_t crcV2(uint8_t crc, const uint8_t& b);

protected:
    // I/O
    //std::unique_ptr<SerialPortImpl> pimpl;
    asio::io_service io;     ///<! io service
    asio::serial_port port;  ///<! port for serial device
    asio::streambuf buffer;
    // threading
    std::thread thread;
    bool running;
    
    //synchronous messaging
    std::condition_variable cv_response;
    std::mutex cv_response_mtx;
    
    std::mutex mutex_response;
    std::mutex mutex_buffer;
    std::mutex mutex_send;
    
    // message for request method
    std::unique_ptr<ReceivedMessage> request_received;

    std::mutex mutex_subscriptions;
    std::map<msp::ID, std::shared_ptr<SubscriptionBase>> subscriptions;
    

    // debugging
    bool print_warnings_;
    bool print_debug_;
    
    int msp_ver_;
    FirmwareVariant fw_variant;
    
    

};

} // namespace client
} // namespace msp

#endif // CLIENT_HPP
