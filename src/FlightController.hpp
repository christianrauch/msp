#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP

#include "MSP.hpp"
#include "msp_id.hpp"

#include <map>
#include <unordered_map>

#include <typeinfo>
#include <typeindex>

#include <functional>

namespace fcu {

class SubscriptionBase {
public:
    msp::ID channel;
    virtual void call(const void* const msg) = 0;
    virtual void call(const msp::Request* const msg) = 0;
};

template<typename T, typename C>
class Subscription : public SubscriptionBase {
public:
    typedef void(C::*Callback)(T);

    Subscription(const msp::ID id, const Callback caller, C *const context_class) : context(context_class) {
        setCaller(caller);
        channel = id;
    }

    void setCaller(Callback caller) { funct = caller; }

    void call(const msp::Request* const msg) {
        // test cast, e.g. check if T is derived from Request
        if(dynamic_cast<T>(msg)==NULL) {
            throw std::bad_cast();
        }
        // call callback function
        (*context.*funct)(dynamic_cast<T>(msg));

    }

    void call(const void* const msg) {
        (*context.*funct)(static_cast<T>(msg));
    }

private:
    Callback funct;
    C *const context;
};

class FlightController {
public:
    FlightController(const std::string &device);

    ~FlightController();

    template<typename T, typename C>
    /**
     * @brief subscribe register message type with callback function
     * @param id message type ID
     * @param callback pointer to callback function (method of class)
     * @param context class of callback method
     */
    void subscribe(msp::ID id, void (C::*callback)(T), C *context) {
        subscriptions[id] = new Subscription<T,C>(id, callback, context);
    }

    /**
     * @brief handle listen for messages and call callback functions
     */
    void handle();

    void setAcc1G(const float acc1g) { acc_1g=acc1g; }

    void setGyroUnit(const float gyro) { gyro_unit=gyro; }

    void setMagnGain(const float gain) { magn_gain=gain; }

    void setStandardGravity(const float gravity) { standard_gravity=gravity; }

    bool setRc(const uint roll, const uint pitch, const uint yaw, const uint throttle);

    bool arm(const bool arm);

private:
    void populate(msp::Request *req);

    void populate_all();

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
