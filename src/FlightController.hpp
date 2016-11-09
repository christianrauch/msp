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
    void subscribe(msp::ID id, void (C::*callback)(T), C *context) {
        subscriptions[id] = new Subscription<T,C>(id, callback, context);
    }

    void handle();

    void setAcc1G(const float acc1g) { acc_1g=acc1g; }

    void setGyroUnit(const float gyro) { gyro_unit=gyro; }

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
    float acc_1g;

    float gyro_unit;
};

} // namespace msp

#endif // FLIGHTCONTROLLER_HPP
