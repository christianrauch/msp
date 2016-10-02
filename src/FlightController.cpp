#include "FlightController.hpp"
#include "msp_msg.hpp"

namespace msp {

FlightController::FlightController(const std::string &device) : msp(device) {
    populate_all();

    msp::Ident ident;
    msp.request_timeout(ident, 10);
    //std::cout<<"MSP version "<<(int)ident.version<<" ready"<<std::endl;
}

FlightController::~FlightController() { }

void FlightController::populate(Request *req) {
    database[req->id()] = req;
}

void FlightController::populate_all() {
    populate(new Ident);
    populate(new Status);
}

void FlightController::subscribe(ID id, RequestCallback callback) {
    subscriptions[id] = callback;
}

void FlightController::handle() {
    for(auto s : subscriptions) {
        //msp.request_block(*database[s.first]);
        msp.request(*database[s.first]);
        s.second(database[s.first]);
    }
}

} // namespace msp
