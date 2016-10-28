#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP

#include "MSP.hpp"
#include "msp_id.hpp"

#include <map>
#include <unordered_map>

#include <typeinfo>
#include <typeindex>

#include <functional>

#include <iostream>

namespace fcu {

class SubscriptionBase {
public:
    msp::ID channel;
    virtual void call(const void* const msg) = 0;
    virtual void call(const msp::Request* const msg) = 0;
};

//template<typename T, typename R>
template<typename T>
class Subscription : public SubscriptionBase {
public:
    typedef std::function<void(const T&)> Callback;

    Subscription(const msp::ID id, const Callback caller) {
        setCaller(caller);
        channel = id;
    }

    void setCaller(Callback caller) { funct = caller; }

    void call(const msp::Request* const msg) {
        std::cout<<"Request type"<<std::endl;
        // test cast, e.g. check if T is derived from Request
        if(dynamic_cast<T>(msg)==NULL) {
            throw std::bad_cast();
        }
        // call callback function
        funct(dynamic_cast<T>(msg));

    }

    void call(const void* const msg) {
        std::cout<<"Other type"<<std::endl;
        // call callback function
        funct(static_cast<T>(msg));
    }

private:
    Callback funct;
};

class FlightController {
public:
    FlightController(const std::string &device);

    ~FlightController();

    template<typename T>
    void subscribe(msp::ID id, void (callback)(T)) {
        subscriptions[id] = new Subscription<T>(id, callback);
    }

    void handle();

    void setAcc1G(const float acc1g) { acc_1g=acc1g; }

    void setGyroUnit(const float gyro) { gyro_unit=gyro; }

private:
    void populate(msp::Request *req);

    void populate_all();

    msp::MSP msp;

    std::map<msp::ID, msp::Request*> database;

    std::map<msp::ID, SubscriptionBase*> subscriptions;

   // sensor specific units
    float acc_1g;

    float gyro_unit;
};

} // namespace msp

#endif // FLIGHTCONTROLLER_HPP
