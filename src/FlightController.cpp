#include "FlightController.hpp"
#include "msp_msg.hpp"
#include "fcu_msg.hpp"

#include <iostream>

namespace fcu {

FlightController::FlightController(const std::string &device) : msp(device) {
    populate_all();

    std::cout<<"Wait for FC..."<<std::endl;
    msp::Ident ident;
    msp.request_timeout(ident, 10);
    std::cout<<"MSP version "<<(int)ident.version<<" ready"<<std::endl;
}

FlightController::~FlightController() { }

void FlightController::populate(msp::Request *req) {
    database[req->id()] = req;
}

void FlightController::populate_all() {
    populate(new msp::Ident);
    populate(new msp::Status);
    populate(new msp::RawImu);
    populate(new msp::Servo);
    populate(new msp::Motor);
    populate(new msp::Rc);
    populate(new msp::RawGPS);
    populate(new msp::CompGPS);
    populate(new msp::Attitude);
    populate(new msp::Altitude);
    populate(new msp::Analog);
    populate(new msp::RcTuning);
    populate(new msp::Pid);
}

void FlightController::handle() {
    for(auto s : subscriptions) {
        //msp.request_block(*database[s.first]);
        msp.request(*database[s.first]);
        try {
            s.second->call(database[s.first]);
        }
        catch(const std::bad_cast &e) {
            std::cerr<<"no Request, other type"<<std::endl;
            switch(s.first) {
            case msp::ID::MSP_IDENT:
                fcu::Ident ident;
                ident.fromIdent(*dynamic_cast<msp::Ident*>(database[s.first]));
                s.second->call(&ident);
                break;
            }
        }
    }
}

} // namespace msp
