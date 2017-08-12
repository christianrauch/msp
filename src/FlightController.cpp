#include "FlightController.hpp"

#include <iostream>

namespace fcu {

FlightController::FlightController(const std::string &device, const uint baudrate) {
    client.connect(device, baudrate);
    client.start();
}

FlightController::~FlightController() {
    client.stop();
}

void FlightController::waitForConnection() {
    std::cout<<"Wait for FC..."<<std::endl;
    msp::msg::Ident ident;
    while(!client.request(ident, 0.5));
    std::cout<<"MultiWii version "<<uint(ident.version)<<" ready"<<std::endl;
}

void FlightController::initialise() {
    // wait for connection to be established
    while(client.request(ident, 0.5)==-1);

    msp::msg::ApiVersion api;
    if(client.request(api)) {
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
    msp::msg::Status status;
    client.request(status);
    sensors = status.sensors;

    // get boxes
    initBoxes();

    // determine channel mapping
    if(isFirmwareMultiWii()) {
        // default mapping
        channel_map.clear();
        for(uint8_t i(0); i<MAX_MAPPABLE_RX_INPUTS; i++) {
            channel_map.push_back(i);
        }
    }
    else {
        // get channel mapping from MSP_RX_MAP
        msp::msg::RxMap rx_map;
        client.request(rx_map);
        channel_map = rx_map.map;
    }
}

bool FlightController::isFirmware(const FirmwareType firmware_type) {
    return firmware == firmware_type;
}

void FlightController::initBoxes() {
    client.setPrintWarnings(true);
    // get box names
    msp::msg::BoxNames box_names;
    if(!client.request(box_names))
        throw std::runtime_error("Cannot get BoxNames!");

    // get box IDs
    msp::msg::BoxIds box_ids;
    if(!client.request(box_ids))
        throw std::runtime_error("Cannot get BoxIds!");

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
    client.setPrintWarnings(false);
}

bool FlightController::isStatusActive(const std::string& status_name) {
    if(box_name_ids.count(status_name)==0) {
        // box ids have not been initialised or requested status is unsupported by FC
        throw std::runtime_error("Box ID of "+status_name+" is unknown! You need to call 'initBoxes()' first.");
    }

    msp::msg::Status status;
    client.request(status);

    // check if ARM box id is amongst active box IDs
    return status.active_box_id.count(box_name_ids.at(status_name));
}

bool FlightController::setRc(const uint16_t roll, const uint16_t pitch,
                             const uint16_t yaw, const uint16_t throttle,
                             const uint16_t aux1, const uint16_t aux2,
                             const uint16_t aux3, const uint16_t aux4,
                             const std::vector<uint16_t> auxs)
{
    if(isFirmwareMultiWii() && hasDynBal()) {
        throw std::runtime_error(
            "DYNBALANCE is active!\n"
            "RC commands will have no effect on motors.");
    }

    msp::msg::SetRc rc;
    // insert mappable channels
    rc.channels.resize(MAX_MAPPABLE_RX_INPUTS);
    rc.channels[channel_map[0]] = roll;
    rc.channels[channel_map[1]] = pitch;
    rc.channels[channel_map[2]] = yaw;
    rc.channels[channel_map[3]] = throttle;
    rc.channels[channel_map[4]] = aux1;
    rc.channels[channel_map[5]] = aux2;
    rc.channels[channel_map[6]] = aux3;
    rc.channels[channel_map[7]] = aux4;

    // insert remaining aux channels
    rc.channels.insert(std::end(rc.channels), std::begin(auxs), std::end(auxs));

    // send MSP_SET_RAW_RC without waiting for ACK
    return client.respond(rc, false);
}

bool FlightController::setRc(const std::vector<uint16_t> channels) {
    msp::msg::SetRc rc;
    rc.channels = channels;
    return client.respond(rc, false);
}

bool FlightController::setMotors(const std::array<uint16_t,msp::msg::N_MOTOR> &motor_values) {
    if(isFirmwareMultiWii() && !hasDynBal()) {
        throw std::runtime_error(
            "DYNBALANCE is not active!\n"
            "Set '#define DYNBALANCE' in your MultiWii 'config.h'");
    }

    msp::msg::SetMotor motor;
    motor.motor = motor_values;
    return client.respond(motor);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

bool FlightController::disarm_block() {
    // attempt to disarm while FC is armed
    while(isArmed()==true) {
        arm(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

int FlightController::updateFeatures(const std::set<std::string> &add,
                                     const std::set<std::string> &remove)
{
    // get original feature configuration
    msp::msg::Feature feature_in;
    if(!client.request(feature_in))
        return -1;

    // update feature configuration
    msp::msg::SetFeature feature_out;
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

    if(!client.respond(feature_out))
        return -1;

    // make settings permanent and reboot
    if(!writeEEPROM())
        return -1;
    if(!reboot())
        return -1;

    return 1;
}

bool FlightController::reboot() {
    return client.respond(msp::msg::Reboot());
}

bool FlightController::writeEEPROM() {
    return client.respond(msp::msg::WriteEEPROM());
}

} // namespace msp
