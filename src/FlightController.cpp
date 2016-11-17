#include "FlightController.hpp"
#include "fcu_msg.hpp"

#include <iostream>

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
    populate(new msp::DebugMessage);// 253
    populate(new msp::Debug);       // 254
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
                const fcu::Imu msg(*(msp::RawImu*)req, acc_1g, gyro_unit, magn_gain, standard_gravity);
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
            default:
                throw std::runtime_error("message ID not handeled");
            } // switch ID
        }
    }
}

void FlightController::sendRequests() {
    for(auto s : subscriptions) {
        msp.sendData(s.first);
    }
}

bool FlightController::sendRequest(const msp::ID id) {
    return msp.sendData(id);
}

void FlightController::handleRequests() {
    while(true) {
        msp::ID id;
        msp::ByteVector data;

        try {
            const msp::DataID data_id = msp.receiveData();
            id = data_id.id;
            data = data_id.data;
        }
        catch(const msp::MalformedHeader &e) {
            std::cerr<<e.what()<<std::endl;
            break;
        }
        catch(msp::WrongCRC &e) {
            std::cerr<<e.what()<<std::endl;
            break;
        }
        catch(msp::UnknownMsgId &e) {
            std::cerr<<e.what()<<std::endl;
            break;
        }
        catch(boost::system::system_error &e) {
            // no more data
            break;
        }

        // search for correct subscribtion
        for(auto s : subscriptions) {
            if(s.first==id) {
                // get correct request type and decode message
                msp::Request *const req = getRequestById(id);
                req->decode(data);

                SubscriptionBase* const sub = s.second;
                try {
                    sub->call(req);
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
                        const fcu::Imu msg(*(msp::RawImu*)req, acc_1g, gyro_unit, magn_gain, standard_gravity);
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
                    default:
                        throw std::runtime_error("message ID not handeled");
                    } // switch ID
                }
            } // check ID
        } // subscriptions
    } // while there is still data
}

bool FlightController::setRc(const uint roll, const uint pitch, const uint yaw, const uint throttle) {
    msp::SetRc rc;
    rc.roll = roll;
    rc.pitch = pitch;
    rc.yaw = yaw;
    rc.throttle = throttle;

    // send MSP_SET_RAW_RC without waiting for ACK
    return msp.send(rc);
}

bool FlightController::arm(const bool arm) {
    // arm:
    // throttle: 1000 (bottom), yaw: 2000 (right)
    // disarm:
    // throttle: 1000 (bottom), yaw: 1000 (left)

    const uint yaw = arm ? 2000 : 1000;

    return setRc(0,0,yaw,1000);
}

} // namespace msp
