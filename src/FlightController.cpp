#include "FlightController.hpp"
#include "fcu_msg.hpp"

namespace fcu {

FlightController::FlightController(const std::string &device) : msp(device) {
    populate_all();

//    std::cout<<"Wait for FC..."<<std::endl;
//    msp::Ident ident;
//    msp.request_timeout(ident, 10);
//    std::cout<<"MSP version "<<(int)ident.version<<" ready"<<std::endl;
}

FlightController::~FlightController() { }

void FlightController::populate(msp::Request *req) {
    database[req->id()] = req;
}

void FlightController::populate_all() {
    populate(new msp::Ident);   // 100
    populate(new msp::Status);
    populate(new msp::RawImu);
    populate(new msp::Servo);
    populate(new msp::Motor);
    populate(new msp::Rc);
    populate(new msp::RawGPS);
    populate(new msp::CompGPS);
    populate(new msp::Attitude);
    populate(new msp::Altitude);
    populate(new msp::Analog);  // 110
    populate(new msp::RcTuning);
    populate(new msp::Pid);
    populate(new msp::Box);
    populate(new msp::Misc);
    populate(new msp::MotorPins);
    populate(new msp::BoxNames);
    populate(new msp::PidNames);
    populate(new msp::WayPoint);
    populate(new msp::BoxIds);
    populate(new msp::ServoConf);
    populate(new msp::NavStatus);
    populate(new msp::NavConfig);   // 122
}

void FlightController::handle() {
    for(auto s : subscriptions) {
        // pointer to request
        const msp::ID id = s.first;
        msp::Request* const req = getRequestById(id);
        SubscriptionBase* const sub = s.second;

        // request data
        msp.request(*req);

        // callback
        try {
            s.second->call(req);
        }
        catch(const std::bad_cast &e) {
            switch(id) {
            case msp::ID::MSP_IDENT: {
                const fcu::Ident msg(*(msp::Ident*)req);
                sub->call(&msg); break; }
            case msp::ID::MSP_STATUS: {
                const fcu::Status msg(*(msp::Status*)req);
                sub->call(&msg); break; }
            case msp::ID::MSP_RAW_IMU: {
                const fcu::Imu msg(*(msp::RawImu*)req, acc_1g, gyro_unit);
                sub->call(&msg); break; }
            case msp::ID::MSP_ATTITUDE: {
                const fcu::Attitude msg(*(msp::Attitude*)req);
                sub->call(&msg); break; }
            case msp::ID::MSP_ALTITUDE: {
                const fcu::Altitude msg(*(msp::Altitude*)req);
                sub->call(&msg); break; }
            case msp::ID::MSP_ANALOG: {
                const fcu::Analog msg(*(msp::Analog*)req);
                sub->call(&msg); break; }
            case msp::ID::MSP_RC_TUNING: {
                const fcu::RcTuning msg(*(msp::RcTuning*)req);
                sub->call(&msg); break; }
            case msp::ID::MSP_PID: {
                const fcu::PID msg(*(msp::Pid*)req);
                sub->call(&msg); break; }
            case msp::ID::MSP_BOX: {
                const fcu::Box msg(*(msp::Box*)req);
                sub->call(&msg); break; }
            case msp::ID::MSP_MISC: {
                const fcu::Misc msg(*(msp::Misc*)req);
                sub->call(&msg); break; }
            } // switch ID
        }
    }
}

} // namespace msp
