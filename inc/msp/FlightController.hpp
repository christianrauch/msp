#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP

#include "MSP.hpp"
#include "msp_id.hpp"

#include "PeriodicTimer.hpp"
#include <map>

namespace fcu {

class SubscriptionBase {
public:
    SubscriptionBase(PeriodicTimer *timer=NULL) : timer(timer) { }

    virtual ~SubscriptionBase() {
        if(timer!=NULL) { delete timer; }
    }

    virtual void call(const msp::Request &req) = 0;

    bool hasTimer() {
        return timer!=NULL;
    }

protected:
    PeriodicTimer *timer;
};

template<typename T, typename C>
class Subscription : public SubscriptionBase {
public:
    typedef void(C::*Callback)(T&);

    Subscription(const Callback caller, C *const context_class)
        : funct(caller), context(context_class) {
    }

    Subscription(const Callback caller, C *const context_class, PeriodicTimer *timer)
        : SubscriptionBase(timer), funct(caller), context(context_class)
    {
        this->timer->start();
    }

    void call(const msp::Request &req) {
        (*context.*funct)( dynamic_cast<const T&>(req) );
    }

private:
    Callback funct;
    C *const context;
};

class FlightController {
public:
    FlightController(const std::string &device);

    ~FlightController();

    /**
     * @brief getMSP expose underlying MSP instance for low-level access
     * @return reference to MSP instance
     */
    msp::MSP &getMSP() { return msp; }

    void waitForConnection();

    template<typename T>
    void populate(T* req) {
        database[req->id()] = req;
    }

    void populate_database();

    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback pointer to callback function (method of class)
     * @param context class of callback method
     */
    template<typename T, typename C>
    void subscribe(void (C::*callback)(T&), C *context, const double tp = 0.0) {
        if(std::is_base_of<msp::Request, T>::value) {
            if(tp>0.0) {
                // subscription with periodic sending of requests
                subscriptions[T().id()] = new Subscription<T,C>(callback, context,
                    new PeriodicTimer(
                        std::bind(&FlightController::sendRequest, this, T().id()),
                        tp
                    )
                );
            }
            else {
                // subscription with manual sending of requests
                subscriptions[T().id()] = new Subscription<T,C>(callback, context);
            }
        }
        else {
            throw std::runtime_error("Callback parameter needs to be of Request type!");
        }
    }

    /**
     * @brief handle listen for messages and call callback functions
     */
    void handle();

    /**
     * @brief handle listen for messages and call callback functions
     */
    void handle_batch() {
        sendRequests();
        handleRequests();
    }

    /**
     * @brief sendRequests send all subscribed requests
     */
    void sendRequests();

    /**
     * @brief sendRequest send request with ID
     * @param id message ID of request
     * @return true on success
     * @return false on failure
     */
    bool sendRequest(const msp::ID id);

    /**
     * @brief handleRequests read incomming data and call corresponding callbacks
     */
    void handleRequests();

    void setAcc1G(const float acc1g) { acc_1g=acc1g; }

    void setGyroUnit(const float gyro) { gyro_unit=gyro; }

    void setMagnGain(const float gain) { magn_gain=gain; }

    void setStandardGravity(const float gravity) { standard_gravity=gravity; }

    bool setRc(const uint roll, const uint pitch, const uint yaw, const uint throttle);

    bool arm(const bool arm);

private:
    msp::Request* getRequestById(const msp::ID id) {
        return database[id];
    }

    msp::MSP msp;

    std::map<msp::ID, msp::Request*> database;

    std::map<msp::ID, SubscriptionBase*> subscriptions;

   // sensor specific units
    float acc_1g;       // reported acceleration value at 1G

    float gyro_unit;

    float magn_gain;    // scale magnetic value to uT (micro Tesla)

    float standard_gravity; // standard gravity for 1g in m/s^2
};

} // namespace msp

#endif // FLIGHTCONTROLLER_HPP
