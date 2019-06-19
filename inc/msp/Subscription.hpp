#ifndef SUBSCRIPTION_HPP
#define SUBSCRIPTION_HPP

#include <functional>
#include "Client.hpp"
#include "Message.hpp"
#include "PeriodicTimer.hpp"

namespace msp {
namespace client {

class SubscriptionBase {
public:
    SubscriptionBase() {}

    virtual ~SubscriptionBase() {}

    virtual void decode(msp::ByteVector& data) const = 0;

    virtual void makeRequest() const = 0;

    virtual void handleResponse() const = 0;

    virtual const msp::Message& getMsgObject() const = 0;

    /**
     * @brief Checks to see if the subscription fires automatically
     * @returns True if the request happens automatically
     */
    bool isAutomatic() const {
        return hasTimer() && (timer_->getPeriod() > 0.0);
    }

    /**
     * @brief Checks to see if the timer has been created
     * @returns True if there is a timer
     */
    bool hasTimer() const { return timer_ ? true : false; }

    /**
     * @brief Start the timer for automatic execution
     * @returns True if the timer starts successfully
     */
    bool start() const { return this->timer_->start(); }

    /**
     * @brief Stop the timer's automatic execution
     * @returns True if the timer stops successfully
     */
    bool stop() const { return this->timer_->stop(); }

    /**
     * @brief setTimerPeriod change the period of the timer
     * @param period_seconds period in seconds
     */
    void setTimerPeriod(const double& period_seconds) {
        if(timer_) {
            timer_->setPeriod(period_seconds);
        }
        else if(period_seconds > 0.0) {
            timer_ = std::unique_ptr<PeriodicTimer>(new PeriodicTimer(
                std::bind(&SubscriptionBase::makeRequest, this),
                period_seconds));
            this->timer_->start();
        }
    }

    /**
     * @brief setTimerFrequency change the update rate of timer
     * @param rate_hz frequency in Hz
     */
    void setTimerFrequency(const double& rate_hz) {
        if(timer_) {
            timer_->setPeriod(1.0 / rate_hz);
        }
        else if(rate_hz > 0.0) {
            timer_ = std::unique_ptr<PeriodicTimer>(new PeriodicTimer(
                std::bind(&SubscriptionBase::makeRequest, this),
                1.0 / rate_hz));
            this->timer_->start();
        }
    }

protected:
    std::unique_ptr<PeriodicTimer> timer_;
};

template <typename T> class Subscription : public SubscriptionBase {
public:
    typedef std::function<void(const T&)> CallbackT;
    typedef std::function<void(const msp::Message&)> CallbackM;

    /**
     * @brief Subscription constructor
     */
    Subscription() {}

    /**
     * @brief Subscription constructor setting all parameters
     * @param recv_callback Callback to execute upon receipt of message
     * @param send_callback Callback to execute periodically to send message
     * @param io_object Object which is used for encoding/decoding data
     * @param period Repition rate of the request
     */
    Subscription(const CallbackT& recv_callback, const CallbackM& send_callback,
                 std::unique_ptr<T>&& io_object, const double& period = 0.0) :
        recv_callback_(recv_callback),
        send_callback_(send_callback),
        io_object_(std::move(io_object)) {
        if(period > 0.0) {
            timer_ = std::unique_ptr<PeriodicTimer>(new PeriodicTimer(
                std::bind(&Subscription<T>::makeRequest, this), period));
            this->timer_->start();
        }
    }

    /**
     * @brief Virtual method for decoding received data
     * @param data Data to be unpacked
     */
    virtual void decode(msp::ByteVector& data) const override {
        io_object_->decode(data);
        recv_callback_(*io_object_);
    }

    /**
     * @brief Sets the object used for packing and unpacking data
     * @param obj unique_ptr to a Message-derived object
     */
    void setIoObject(std::unique_ptr<T>&& obj) const {
        io_object_ = std::move(obj);
    }

    /**
     * @brief Gets a reference to the IO object
     * @returns
     */
    const T& getIoObject() const { return *io_object_; }

    /**
     * @brief Gets a reference to the internal IO object as a Message
     * @returns reference to a Message
     */
    virtual const msp::Message& getMsgObject() const override {
        return *io_object_;
    }

    /**
     * @brief Sets the callback to be executed on success
     * @param recv_callback the callback to be executed
     */
    void setReceiveCallback(const CallbackT& recv_callback) const {
        recv_callback_ = recv_callback;
    }

    /**
     * @brief Calls the receive callback if it exists
     */
    virtual void handleResponse() const override {
        if(recv_callback_) recv_callback_(*io_object_);
    }

    /**
     * @brief Sets the callback used to send the request
     * @param send_callback the callback to be executed
     */
    void setSendCallback(const CallbackM& send_callback) const {
        send_callback_ = send_callback;
    }

    /**
     * @brief Calls the send callback if it exists
     */
    virtual void makeRequest() const override {
        if(send_callback_) send_callback_(*io_object_);
    }

protected:
    CallbackT recv_callback_;
    CallbackM send_callback_;
    std::unique_ptr<T> io_object_;
};

}  // namespace client
}  // namespace msp

#endif
