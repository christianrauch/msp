#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP

#include "MSP.hpp"
#include "msp_id.hpp"

#include <map>

namespace msp {

class FlightController {
public:
    // function pointer
    typedef void (*RequestCallback)(Request *request);

    FlightController(const std::string &device);

    ~FlightController();

    void subscribe(ID id, RequestCallback callback);

    void handle();

private:
    void populate(Request *req);

    void populate_all();

    msp::MSP msp;

    std::map<ID, Request*> database;

    std::map<ID, RequestCallback> subscriptions;
};

} // namespace msp

#endif // FLIGHTCONTROLLER_HPP
