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
    virtual void call(void *msg) = 0;
//    virtual void setMsg(void *msg) = 0;
};

//template<typename T, typename R>
template<typename T>
class Subscription : public SubscriptionBase {
public:
    typedef std::function<void(T)> Callback;

    Subscription(Callback caller) { setCaller(caller); }

    void setCaller(Callback caller) { funct = caller; }

    void call(void *msg) { funct(*static_cast<T*>(msg)); }

private:
    Callback funct;
};

class FlightController {
public:
    FlightController(const std::string &device);

    ~FlightController();

    template<typename T>
    void subscribe(msp::ID id, void (callback)(T)) {
        subscriptions[id] = new Subscription<T>(callback);
    }

    void handle();

private:
    void populate(msp::Request *req);

    void populate_all();

    msp::MSP msp;

    std::map<msp::ID, msp::Request*> database;

    std::map<msp::ID, SubscriptionBase*> subscriptions;
};

} // namespace msp

#endif // FLIGHTCONTROLLER_HPP
