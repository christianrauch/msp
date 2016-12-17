#include "FlightController.hpp"

#include <iostream>

namespace fcu {

FlightController::FlightController(const std::string &device) : msp(device) { }

FlightController::~FlightController() {
    for(const std::pair<msp::ID, msp::Request*> d : database)
        delete d.second;

    for(const std::pair<msp::ID, SubscriptionBase*> s : subscriptions)
        delete s.second;
}

void FlightController::waitForConnection() {
    std::cout<<"Wait for FC..."<<std::endl;
    msp::Ident ident;
    msp.request_wait(ident, 100);
    std::cout<<"MultiWii version "<<uint(ident.version)<<" ready"<<std::endl;
}

void FlightController::initialise() {
    populate_database();

    // wait for connection to be established
    //msp.request_timeout(ident, 1000);
    msp.request_wait(ident, 100);

    msp::ApiVersion api;
    if(msp.request_block(api)) {
        // this is Cleanflight
        firmware = FirmwareType::CLEANFLIGHT;
        std::cout<<"Cleanflight API "<<api.major<<"."<<api.minor<<"."<<api.protocol<<" ready"<<std::endl;
    }
    else {
        // this is MultiWii
        firmware = FirmwareType::MULTIWII;
        std::cout<<"MultiWii version "<<uint(ident.version)<<" ready"<<std::endl;
    }

    // get sensors
    msp::Status status;
    msp.request_block(status);
    sensors = status.sensors;

    // get boxes
    initBoxes();
}

bool FlightController::isFirmware(const FirmwareType firmware_type) {
    return firmware == firmware_type;
}

void FlightController::populate_database() {
    populate(new msp::Ident);   // 100
    populate(new msp::Status);
    populate(new msp::Imu(acc_1g, gyro_unit, magn_gain, standard_gravity));
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
        if(msp.request(*req)) {
            // callback
            sub->call(*req);
        }
    }
}

void FlightController::sendRequests() {
    for(auto s : subscriptions) {
        // send requests only if there is no periodic timer already sending the same requests
        if(!s.second->hasTimer()) { msp.sendData(s.first); }
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
        catch(msp::NoData) {
            break;
        }
        catch(boost::system::system_error) {
            break;
        }

        // search for correct subscribtion
        if(subscriptions.count(id)) {
            // get correct request type and decode message
            msp::Request *const req = getRequestById(id);
            req->decode(data);

            SubscriptionBase* const sub = subscriptions.at(id);
            sub->call(*req);
        } // check ID in subscriptions
    } // while there is still data
}

void FlightController::initBoxes() {
    // get box names
    msp::BoxNames box_names;
    msp.request_block(box_names);

    // get box IDs
    msp::BoxIds box_ids;
    msp.request_block(box_ids);

    assert(box_names.box_names.size()==box_ids.box_ids.size());

    box_name_ids.clear();
    for(uint ibox(0); ibox<box_names.box_names.size(); ibox++) {
        box_name_ids[box_names.box_names[ibox]] = box_ids.box_ids[ibox];
    }
}

bool FlightController::isArmed() {
    if(box_name_ids.count("ARM")==0) {
        // box ids have not been initialised
        throw std::runtime_error("Box ID of 'ARM' is unknown! You need to call 'initBoxes()' first.");
    }

    msp::Status status;
    msp.request_block(status);

    // check if ARM box id is amongst active box IDs
    return status.active_box_id.count(box_name_ids.at("ARM"));
}

bool FlightController::setRc(const uint roll, const uint pitch,
                             const uint yaw, const uint throttle,
                             const uint aux1, const uint aux2,
                             const uint aux3, const uint aux4)
{
    if(hasDynBal()) {
        throw std::runtime_error(
            "DYNBALANCE is active!\n"
            "RC commands will have no effect on motors.");
    }

    msp::SetRc rc;
    rc.roll = roll;
    rc.pitch = pitch;
    rc.yaw = yaw;
    rc.throttle = throttle;
    rc.aux1 = aux1;
    rc.aux2 = aux2;
    rc.aux3 = aux3;
    rc.aux4 = aux4;

    // send MSP_SET_RAW_RC without waiting for ACK
    return msp.send(rc);
}

bool FlightController::setMotors(const std::array<uint16_t,msp::N_MOTOR> &motor_values) {
    if(!hasDynBal()) {
        throw std::runtime_error(
            "DYNBALANCE is not active!\n"
            "Set '#define DYNBALANCE' in your MultiWii 'config.h'");
    }

    msp::SetMotor motor;
    motor.motor = motor_values;
    return msp.respond_block(motor);
}

bool FlightController::arm(const bool arm) {
    // arm:
    // throttle: 1000 (bottom), yaw: 2000 (right)
    // disarm:
    // throttle: 1000 (bottom), yaw: 1000 (left)

    const uint yaw = arm ? 2000 : 1000;

    return setRc(1500, 1500, yaw, 1000,
                 1000, 1000, 1000, 1000);
}

bool FlightController::arm_block() {
    // attempt to arm while FC is disarmed
    while(isArmed()==false) {
        arm(true);
    }
    return true;
}

bool FlightController::disarm_block() {
    // attempt to disarm while FC is armed
    while(isArmed()==true) {
        arm(false);
    }
    return true;
}

} // namespace msp
