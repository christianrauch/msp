#ifndef SUBSCRIPTION_HPP
#define SUBSCRIPTION_HPP

#include "periodic_timer.hpp"
#include "message.hpp"
#include "Client.hpp"
#include <functional>




namespace msp {
namespace client {


class SubscriptionBase {
public:
    SubscriptionBase() {};
    
    virtual void decode(msp::ByteVector& data) = 0;
    
    virtual void makeRequest() = 0;
    
    virtual void handleResponse() = 0;
    
    virtual msp::Message& getMsgObject() = 0;
    
    bool isAutomatic()
    {
        return hasTimer() && (timer_->getPeriod() > 0.0);
    }
    
    bool hasTimer()
    {
        return timer_ ? true : false;
    }

    /**
     * @brief setTimerPeriod change the period of the timer
     * @param period_seconds period in seconds
     */
    void setTimerPeriod(const double& period_seconds) {
        if (timer_) {
            timer_->setPeriod(period_seconds);
        }
        else if (period_seconds > 0.0) {
            timer_ = std::unique_ptr<PeriodicTimer>(new PeriodicTimer(std::bind(&SubscriptionBase::makeRequest,this),period_seconds));
            this->timer_->start();
        }
        
    }

    /**
     * @brief setTimerFrequency change the update rate of timer
     * @param rate_hz frequency in Hz
     */
    void setTimerFrequency(const double& rate_hz) {
        if (timer_) {
            timer_->setPeriod(1.0/rate_hz);
        }
        else if (rate_hz > 0.0) {
            timer_ = std::unique_ptr<PeriodicTimer>(new PeriodicTimer(std::bind(&SubscriptionBase::makeRequest,this),1.0/rate_hz));
            this->timer_->start();
        }
        
    }
    
protected:
    
    std::unique_ptr<PeriodicTimer> timer_;
    
    

};

template<typename T>
class Subscription : public SubscriptionBase {
public:
    typedef std::function<void(T&)> Callback;
    
    
    Subscription() {}
    
    virtual void decode(msp::ByteVector& data) override
    {
        io_object_->decode(data);
        recv_callback_(*io_object_);
    }
    
    
    void setIoObject(std::unique_ptr<T> obj)
    {
        io_object_ = std::move(obj);
    }
    
    T& getIoObject(std::unique_ptr<T> obj)
    {
        return *io_object_;
    }
    
    virtual msp::Message& getMsgObject() override
    {
        return *io_object_;
    }
    
    void setReceiveCallback(const Callback& recv_callback)
    {
        recv_callback_ = recv_callback;
    }
    
    
    virtual void handleResponse() override
    {
        if(recv_callback_) recv_callback_( *io_object_ );
    }
    
    void setSendCallback(const Callback& send_callback)
    {
        send_callback_ = send_callback;
    }
    
    virtual void makeRequest() override
    {
        if (send_callback_) send_callback_( *io_object_ );
    }
    
    
    Subscription(const Callback& recv_callback, const Callback& send_callback, T& io_object, const double& period = 0.0)
        : recv_callback_(recv_callback), send_callback_(send_callback)
    {
        this->setIoObject(io_object);
        if (period > 0.0) {
            timer_ = std::unique_ptr<PeriodicTimer>(new PeriodicTimer(std::bind(&Subscription<T>::makeRequest,this),period));
            this->timer_->start();
        }
    }
    
    Subscription(const Callback& recv_callback, const std::function<void(msp::Message&)>& send_callback, T& io_object, const double& period = 0.0)
        : recv_callback_(recv_callback), send_callback_(send_callback)
    {
        this->setIoObject(io_object);
        if (period > 0.0) {
            timer_ = std::unique_ptr<PeriodicTimer>(new PeriodicTimer(std::bind(&Subscription<T>::makeRequest,this),period));
            this->timer_->start();
        }
    }
    
    


protected:
    
    Callback recv_callback_;
    Callback send_callback_;
    std::unique_ptr<T> io_object_;
    
};

}
}

#endif
