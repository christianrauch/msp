#include "FlightController.hpp"

#include <iostream>

namespace fcu {

FlightController::FlightController(const std::string &device, const uint baudrate) {
    msp.connect(device, baudrate);
}

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
    // wait for connection to be established
    msp.request_wait(ident, 100, 7); // 7 byte payload for MSP_IDENT

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

    // determine channel mapping
    if(isFirmwareMultiWii()) {
        // default mapping
        channel_map.clear();
        for(uint i(0); i<MAX_MAPPABLE_RX_INPUTS; i++) {
            channel_map[i] = i;
        }
    }
    else {
        // get channel mapping from MSP_RX_MAP
        msp::RxMap rx_map;
        msp.request_block(rx_map);
        channel_map = rx_map.map;
    }
}

bool FlightController::isFirmware(const FirmwareType firmware_type) {
    return firmware == firmware_type;
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
            id = msp::ID(data_id.id);
            data = data_id.data;
        }
        catch(const msp::MalformedHeader &e) {
            std::cerr<<e.what()<<std::endl;
            break;
        }
        catch(const msp::WrongCRC &e) {
            std::cerr<<e.what()<<std::endl;
            break;
        }
        catch(const msp::UnknownMsgId &e) {
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
        if(isFirmwareCleanflight()) {
            // workaround for wrong box ids in cleanflight
            // cleanflight's box ids are in order of the box names
            // https://github.com/cleanflight/cleanflight/issues/2606
            box_name_ids[box_names.box_names[ibox]] = ibox;
        }
        else {
            box_name_ids[box_names.box_names[ibox]] = box_ids.box_ids[ibox];
        }
    }
}

bool FlightController::isStatusActive(const std::string& status_name) {
    if(box_name_ids.count(status_name)==0) {
        // box ids have not been initialised or requested status is unsupported by FC
        throw std::runtime_error("Box ID of "+status_name+" is unknown! You need to call 'initBoxes()' first.");
    }

    msp::Status status;
    msp.request_block(status);

    // check if ARM box id is amongst active box IDs
    return status.active_box_id.count(box_name_ids.at(status_name));
}

bool FlightController::setRc(const uint16_t roll, const uint16_t pitch,
                             const uint16_t yaw, const uint16_t throttle,
                             const uint16_t aux1, const uint16_t aux2,
                             const uint16_t aux3, const uint16_t aux4)
{
    if(isFirmwareMultiWii() && hasDynBal()) {
        throw std::runtime_error(
            "DYNBALANCE is active!\n"
            "RC commands will have no effect on motors.");
    }

    const std::array<uint16_t, MAX_MAPPABLE_RX_INPUTS> rc_order =
        {{roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4}};

    msp::SetRc rc;
    rc.roll = rc_order[channel_map[0]];
    rc.pitch = rc_order[channel_map[1]];
    rc.yaw = rc_order[channel_map[2]];
    rc.throttle = rc_order[channel_map[3]];
    rc.aux1 = rc_order[channel_map[4]];
    rc.aux2 = rc_order[channel_map[5]];
    rc.aux3 = rc_order[channel_map[6]];
    rc.aux4 = rc_order[channel_map[7]];

    // send MSP_SET_RAW_RC without waiting for ACK
    return msp.send(rc);
}

bool FlightController::setMotors(const std::array<uint16_t,msp::N_MOTOR> &motor_values) {
    if(isFirmwareMultiWii() && !hasDynBal()) {
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

    const uint16_t yaw = arm ? 2000 : 1000;

    return setRc(1500, 1500, yaw, 1000, 1000, 1000, 1000, 1000);
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

int FlightController::updateFeatures(const std::set<std::string> &add,
                                     const std::set<std::string> &remove)
{
    // get original feature configuration
    msp::Feature feature_in;
    if(!msp.request_block(feature_in))
        return -1;

    // update feature configuration
    msp::SetFeature feature_out;
    feature_out.features = feature_in.features;
    // enable features
    for(const std::string &a : add) {
        feature_out.features.insert(a);
    }
    // disable features
    for(const std::string &rem : remove) {
        feature_out.features.erase(rem);
    }

    // check if feature configuration changed
    if(feature_out.features==feature_in.features)
        return 0;

    if(!msp.respond_block(feature_out))
        return -1;

    // make settings permanent and reboot
    if(!writeEEPROM())
        return -1;
    if(!reboot())
        return -1;

    return 1;
}

bool FlightController::reboot() {
    return msp.respond_block(msp::Reboot());
}

bool FlightController::writeEEPROM() {
    return msp.respond_block(msp::WriteEEPROM());
}

} // namespace msp
