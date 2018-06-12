#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <string>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <map>
#include "types.hpp"

namespace msp {

struct SerialPortImpl;

class PeriodicTimer {
public:

    /**
     * @brief PeriodicTimer define a periodic timer
     * @param funct function that is called periodically
     * @param period_seconds period in seconds
     */
    PeriodicTimer(const std::function<void()> funct, const double period_seconds);

    ~PeriodicTimer() { stop(); }

    /**
     * @brief start define and start background thread
     */
    void start();

    /**
     * @brief stop tell thread to stop and wait for end
     */
    void stop();

    /**
     * @brief getPeriod get period in seconds
     * @return period in seconds
     */
    double getPeriod() {
        return period_us.count()/1.e6;
    }

    /**
     * @brief setPeriod change the update period of timer thread
     * This will stop and restart the thread.
     * @param period_seconds period in seconds
     */
    void setPeriod(const double period_seconds);

private:
    std::shared_ptr<std::thread> thread_ptr;
    std::function<void()> funct;
    std::chrono::duration<size_t, std::micro> period_us;
    std::timed_mutex mutex_timer;
    bool running;
};

} // namespace msp

namespace msp {
namespace client {

class SubscriptionBase {
public:
    SubscriptionBase(PeriodicTimer *timer=NULL) : timer(timer) { }

    virtual ~SubscriptionBase() {
        if(timer!=NULL) { delete timer; }
    }

    virtual void call(const msp::Request &req) = 0;

    bool hasTimer() {
        // subscription with manual sending of requests
        return !(timer->getPeriod()>0);
    }

    /**
     * @brief setTimerPeriod change the period of the timer
     * @param period_seconds period in seconds
     */
    void setTimerPeriod(const double period_seconds) {
        timer->setPeriod(period_seconds);
    }

    /**
     * @brief setTimerFrequency change the update rate of timer
     * @param rate_hz frequency in Hz
     */
    void setTimerFrequency(const double rate_hz) {
        timer->setPeriod(1.0/rate_hz);
    }

protected:
    PeriodicTimer *timer;
};

template<typename T>
class Subscription : public SubscriptionBase {
public:
    typedef std::function<void(const T&)> Callback;

    Subscription(const Callback &callback) : callback(callback) { }

    Subscription(const Callback &callback, PeriodicTimer *timer)
        : SubscriptionBase(timer), callback(callback)
    {
        this->timer->start();
    }

    void call(const msp::Request &req) {
        callback( dynamic_cast<const T&>(req) );
    }

private:
    Callback callback;
};

enum MessageStatus {
    OK,         // no errors
    FAIL_ID,    // message ID is unknown
    FAIL_CRC    // wrong CRC
};

struct ReceivedMessage {
    uint8_t id;
    std::vector<uint8_t> data;
    MessageStatus status;
};

class Client {
public:
    Client();

    ~Client();

    void setPrintWarnings(const bool warnings) {
        print_warnings = warnings;
    }

    /**
     * @brief connect establish connection to serial device
     * @param device path or name of serial device
     * @param baudrate serial baudrate (default: 115200)
     * @return true on success
     */
    void connect(const std::string &device, const size_t baudrate=115200);

    /**
     * @brief waitForOneMessage block until one message has been received
     */
    void waitForOneMessage();

    void waitForOneMessageBlock();

    /**
     * @brief start starts the receiver thread that handles incomming messages
     */
    void start();

    /**
     * @brief stop stops the receiver thread
     */
    void stop();

    /**
     * @brief sendData send raw data and ID to flight controller, accepts any uint8 id
     * @param id message ID
     * @param data raw data
     * @return true on success
     * @return false on failure
     */
    bool sendData(const uint8_t id, const ByteVector &data = ByteVector(0));

    /**
     * @brief sendRequest request payload from FC
     * @param id message ID
     * @return true on success
     * @return false on failure
     */
    bool sendRequest(const uint8_t id) {
        return sendData(id, ByteVector());
    }

    bool sendRequest(const msp::ID id) {
        return sendData(uint8_t(id), ByteVector());
    }

    /**
     * @brief sendResponse send payload to FC
     * @param response response with payload
     * @return true on success
     * @return false on failure
     */
    bool sendResponse(const msp::Response &response) {
        return sendData(uint8_t(response.id()), response.encode());
    }

    /**
     * @brief request requests payload from FC and block until payload has been received
     * @param request request whose data will be set by the received payload
     * @param timeout (optional) timeout in seconds
     * @return true on success
     * @return false on failure
     * @return -1 on timeout
     */
    int request(msp::Request &request, const double timeout = 0);

    /**
     * @brief request_raw request raw unstructured payload data
     * @param id message ID
     * @param data reference to data buffer at which the received data will be stores
     * @param timeout (optional) timeout in seconds
     * @return 1 on success
     * @return 0 on failure
     * @return -1 on timeout
     */
    int request_raw(const uint8_t id, ByteVector &data, const double timeout = 0);

    /**
     * @brief respond send payload to FC and block until an ACK has been received
     * @param response response with payload
     * @param wait_ack if set, method will wait for message acknowledgement
     * @return true on success
     * @return false on failure
     */
    bool respond(const msp::Response &response, const bool wait_ack=true);

    /**
     * @brief respond_raw send raw unstructured payload data
     * @param id message ID
     * @param data raw payload
     * @param wait_ack if set, method will wait for message acknowledgement
     * @return true on success
     * @return false on failure
     */
    bool respond_raw(const uint8_t id, const ByteVector &data, const bool wait_ack=true);

    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback pointer to callback function (class method)
     * @param context class with callback method
     * @param tp period of timer that will send subscribed requests (in seconds), by default this is 0 and requests are not sent periodically
     * @return pointer to subscription that is added to internal list
     */
    template<typename T, typename C>
    SubscriptionBase* subscribe(void (C::*callback)(const T&), C *context, const double tp = 0.0) {
        return subscribe<T>(std::bind(callback, context, std::placeholders::_1), tp);
    }

    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback function (e.g. lambda, class method, function pointer)
     * @param tp period of timer that will send subscribed requests (in seconds), by default this is 0 and requests are not sent periodically
     * @return pointer to subscription that is added to internal list
     */
    template<typename T>
    SubscriptionBase* subscribe(const std::function<void(const T&)> &callback, const double tp = 0.0) {

        if(!std::is_base_of<msp::Request, T>::value)
            throw std::runtime_error("Callback parameter needs to be of Request type!");

        if(!(tp>=0.0))
            throw std::runtime_error("Period must be positive!");

        const msp::ID id = T().id();

        std::lock_guard<std::mutex> lock(mutex_callbacks);

        // register message
        if(subscribed_requests.count(id)) { delete subscribed_requests[id]; }
        subscribed_requests[id] = new T();

        // register subscription
        subscriptions[id] = new Subscription<T>(callback,
            new PeriodicTimer(
                std::bind(static_cast<bool(Client::*)(msp::ID)>(&Client::sendRequest), this, id),
                tp
            )
        );

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
    SubscriptionBase* getSubscription(const msp::ID& id) {
        return subscriptions.at(id);
    }

    void processOneMessage();

private:
    /**
     * @brief crc compute checksum of data package
     * @param id message ID
     * @param data raw data vector
     * @return checksum
     */
    uint8_t crc(const uint8_t id, const ByteVector &data);

private:
    // I/O
    std::unique_ptr<SerialPortImpl> pimpl;
    // threading
    std::thread thread;
    bool running;
    std::condition_variable cv_request;
    std::condition_variable cv_ack;
    std::mutex mutex_cv_request;
    std::mutex mutex_cv_ack;
    std::mutex mutex_request;
    std::mutex mutex_callbacks;
    std::mutex mutex_send;
    // message for request method
    ReceivedMessage request_received;
    // subscriptions
    std::map<msp::ID, SubscriptionBase*> subscriptions;
    std::map<msp::ID, msp::Request*> subscribed_requests;
    // debugging
    bool print_warnings;
};

} // namespace client
} // namespace msp

#endif // CLIENT_HPP
