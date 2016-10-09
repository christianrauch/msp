#include "FlightController.hpp"
#include "msp_msg.hpp"

#include <iostream>

namespace msp {

FlightController::FlightController(const std::string &device) : msp(device) {
    populate_all();

    std::cout<<"Wait for FC..."<<std::endl;
    msp::Ident ident;
    msp.request_timeout(ident, 10);
    std::cout<<"MSP version "<<(int)ident.version<<" ready"<<std::endl;
}

FlightController::~FlightController() { }

void FlightController::populate(Request *req) {
    database[req->id()] = req;
}

void FlightController::populate_all() {
    populate(new Ident);
    populate(new Status);
}

void FlightController::handle() {
    for(auto s : subscriptions) {
        //msp.request_block(*database[s.first]);
        msp.request(*database[s.first]);
        s.second->call(database[s.first]);
    }
}

} // namespace msp
