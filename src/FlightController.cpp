#include "FlightController.hpp"

#include <iostream>

namespace fcu {

FlightController::FlightController(const std::string &device, const size_t baudrate) : 
    client_(device,baudrate), msp_version_(1), connected_(false),armed_(false), control_source_(ControlSource::NONE),
    msp_timer_(std::bind(&FlightController::generateMSP,this),0.1)
{
    
}

FlightController::~FlightController() 
{
    disconnect();
}


bool FlightController::connect(bool blocking, double timeout)
{
    
    if (!client_.connect()) return false;
    
    client_.printWarnings(true);

    int rc;
    msp::msg::FcVariant fcvar(fw_variant_);
    rc = client_.sendMessage(fcvar);
    if ( rc == 1) {
        fw_variant_ = msp::variant_map[fcvar.identifier()];
        std::cout << fcvar;
    }
    else std::cout << rc << std::endl;
    
    
    if (fw_variant_ != msp::FirmwareVariant::MWII) {
        msp::msg::ApiVersion api_version(fw_variant_);
        if(client_.sendMessage(api_version)) {
            std::cout << api_version;
            msp_version_ = api_version.major();
            client_.setVersion(msp_version_);
        }
    }
    
    msp::msg::FcVersion fcver(fw_variant_);
    rc = client_.sendMessage(fcver);
    if (rc == 1) {
        std::cout << fcver;
    }
    else std::cout << rc << std::endl;
    
    
    msp::msg::BoardInfo boardinfo(fw_variant_);
    rc = client_.sendMessage(boardinfo);
    if (rc == 1) {
        std::cout << boardinfo;
        board_name_ == boardinfo.name();
    }
    else std::cout << rc << std::endl;
    
    
    msp::msg::BuildInfo buildinfo(fw_variant_);
    rc = client_.sendMessage(buildinfo);
    if (rc == 1)
    std::cout << buildinfo;
    else std::cout << rc << std::endl;
    

    msp::msg::Status status(fw_variant_);
    client_.sendMessage(status);
    std::cout << status;
    sensors_ = status.sensors;

    // get boxes
    initBoxes();

    // determine channel mapping
    if (getFwVariant() == msp::FirmwareVariant::MWII) {
        // default mapping
        for(uint8_t i(0); i<msp::msg::MAX_MAPPABLE_RX_INPUTS; ++i) {
            channel_map_[i] = i;
        }
    }
    else {
        // get channel mapping from MSP_RX_MAP
        msp::msg::RxMap rx_map(fw_variant_);
        client_.sendMessage(rx_map);
        std::cout << rx_map;
        channel_map_ = rx_map.map;
    }
    
    return true;
}

bool FlightController::disconnect(bool blocking, double timeout)
{
    stopMspControl();
    return client_.disconnect();

}    
    
void FlightController::arm(bool blocking) {
    armed_ = true;
}

void FlightController::disarm(bool blocking) {
    armed_ = false;
}

bool FlightController::armSet() {
    return armed_;
}


void FlightController::printDebug(bool on)
{
    client_.printDebug(on);
}


void FlightController::setFlightMode(FlightMode mode)
{
    {
        std::lock_guard<std::mutex> lock(msp_updates_mutex);
        flight_mode_ = mode;
    }
    generateMSP();
}

FlightMode FlightController::getFlightMode()
{
    return flight_mode_;
}

void FlightController::setControlSource(ControlSource source)
{
    if (source == control_source_) return;
    //client_.printDebug(true);
    
    msp::msg::RxConfig rxConfig(fw_variant_);
    if (!client_.sendMessage(rxConfig)) std::cout << "client_.sendMessage(rxConfig) failed" << std::endl;
    std::cout << rxConfig;
    
    msp::msg::SetRxConfig setRxConfig(fw_variant_);
    static_cast<msp::msg::RxConfigSettings&>(setRxConfig) = static_cast<msp::msg::RxConfigSettings&>(rxConfig);
    
    if (source == ControlSource::SBUS) {
        setRxConfig.receiverType = 3;
        setRxConfig.serialrx_provider = 2;
    } else if (source == ControlSource::MSP) {
        setRxConfig.receiverType = 4;
    }
    //std::cout << setRxConfig;
    client_.sendMessage(setRxConfig);
    
    if (source == ControlSource::MSP) startMspControl();
    else stopMspControl();
    
    control_source_ = source;
    //client_.printDebug(false);
}

ControlSource FlightController::getControlSource()
{
    msp::msg::RxConfig rxConfig(fw_variant_);
    client_.sendMessage(rxConfig);
    
    if (rxConfig.receiverType && rxConfig.receiverType() == 4) return ControlSource::MSP;
    else if (rxConfig.serialrx_provider() == 2) return ControlSource::SBUS;
    return ControlSource::OTHER;
    
}

void FlightController::setRPYT(std::array<double,4>& rpyt)
{
    //std::cout << "void FlightController::setRPYT(std::array<double,4>& rpyt)" << std::endl;
    {
        std::lock_guard<std::mutex> lock(msp_updates_mutex);
        rpyt_.swap(rpyt);
    }
    generateMSP();
    //std::cout << "finished setRPYT" << std::endl;
}

void FlightController::startMspControl()
{
    std::cout << "STARTING MSP CONTROL" << std::endl;
    msp_timer_.start();
}

void FlightController::stopMspControl()
{
    std::cout << "STOPPING MSP CONTROL" << std::endl;
    msp_timer_.stop();
}

void FlightController::generateMSP()
{
    std::vector<uint16_t> cmds(18,1000);
    //manually remapping from RPYT to TAER (american RC)
    {
        std::lock_guard<std::mutex> lock(msp_updates_mutex);
        cmds[0] = (rpyt_[3]*500)+1500;
        cmds[1] = (rpyt_[0]*500)+1500;
        cmds[2] = (rpyt_[1]*500)+1500;
        cmds[3] = (rpyt_[2]*500)+1500;
    
        if ( !((uint32_t)flight_mode_.modifier & (uint32_t)FlightMode::MODIFIER::ARM) ) cmds[4] = 1000;
        else if ((uint32_t)flight_mode_.secondary & (uint32_t)FlightMode::SECONDARY_MODE::NAV_ALTHOLD) cmds[4] = 2000;
        else cmds[4] = 1500;
        
        switch (flight_mode_.primary) {
        case FlightMode::PRIMARY_MODE::ANGLE:
            cmds[5] = 1000;
            break;
        case FlightMode::PRIMARY_MODE::NAV_POSHOLD:
            cmds[5] = 1500;
            break;
        case FlightMode::PRIMARY_MODE::NAV_RTH:
            cmds[5] = 2000;
            break;
        default:
            break;
        }
    }
    setRc(cmds);
}
    
bool FlightController::saveSettings() {
    msp::msg::WriteEEPROM writeEEPROM(fw_variant_);
    return client_.sendMessage(writeEEPROM);
}

bool FlightController::reboot() {
    msp::msg::Reboot reboot(fw_variant_);
    return client_.sendMessage(reboot);
}


msp::FirmwareVariant FlightController::getFwVariant() {
    return fw_variant_;
}

int FlightController::getProtocol() {
    return msp_version_;
}

std::string FlightController::getBoardName() {
    return board_name_;
}


void FlightController::initBoxes() {
    client_.printWarnings(true);
    // get box names
    msp::msg::BoxNames box_names(fw_variant_);
    if(!client_.sendMessage(box_names))
        throw std::runtime_error("Cannot get BoxNames!");
    std::cout << box_names << std::endl;
    // get box IDs
    msp::msg::BoxIds box_ids(fw_variant_);
    if(!client_.sendMessage(box_ids))
        throw std::runtime_error("Cannot get BoxIds!");
    std::cout << box_ids << std::endl;
    assert(box_names.box_names.size()==box_ids.box_ids.size());

    box_name_ids_.clear();
    for(size_t ibox(0); ibox<box_names.box_names.size(); ibox++) {
        if(getFwVariant() == msp::FirmwareVariant::CLFL) {
            // workaround for wrong box ids in cleanflight
            // cleanflight's box ids are in order of the box names
            // https://github.com/cleanflight/cleanflight/issues/2606
            box_name_ids_[box_names.box_names[ibox]] = ibox;
        }
        else {
            box_name_ids_[box_names.box_names[ibox]] = box_ids.box_ids[ibox];
        }
    }
    //client_.printWarnings(false);
}

bool FlightController::isStatusActive(const std::string& status_name) {
    if(box_name_ids_.count(status_name)==0) {
        // box ids have not been initialised or requested status is unsupported by FC
        throw std::runtime_error("Box ID of "+status_name+" is unknown! You need to call 'initBoxes()' first.");
    }

    msp::msg::Status status(fw_variant_);
    client_.sendMessage(status);

    // check if ARM box id is amongst active box IDs
    return status.box_mode_flags.count(box_name_ids_.at(status_name));
}

bool FlightController::setRc(const uint16_t roll, const uint16_t pitch,
                             const uint16_t yaw, const uint16_t throttle,
                             const uint16_t aux1, const uint16_t aux2,
                             const uint16_t aux3, const uint16_t aux4,
                             const std::vector<uint16_t> auxs)
{
    
    msp::msg::SetRawRc rc(fw_variant_);
    // insert mappable channels
    rc.channels.resize(msp::msg::MAX_MAPPABLE_RX_INPUTS);
    rc.channels[channel_map_[0]] = roll;
    rc.channels[channel_map_[1]] = pitch;
    rc.channels[channel_map_[2]] = yaw;
    rc.channels[channel_map_[3]] = throttle;
    
    rc.channels.emplace_back(aux1);
    rc.channels.emplace_back(aux2);
    rc.channels.emplace_back(aux3);
    rc.channels.emplace_back(aux4);

    // insert remaining aux channels
    rc.channels.insert(std::end(rc.channels), std::begin(auxs), std::end(auxs));

    // send MSP_SET_RAW_RC without waiting for ACK
    return client_.asyncSendMessage(rc);
}

bool FlightController::setRc(const std::vector<uint16_t> channels) {
    msp::msg::SetRawRc rc(fw_variant_);
    rc.channels = channels;
    return client_.asyncSendMessage(rc);
}

bool FlightController::setMotors(const std::array<uint16_t,msp::msg::N_MOTOR> &motor_values) {
    
    msp::msg::SetMotor motor(fw_variant_);
    motor.motor = motor_values;
    return client_.sendMessage(motor);
}


int FlightController::updateFeatures(const std::set<std::string> &add,
                                     const std::set<std::string> &remove)
{
    // get original feature configuration
    msp::msg::Feature feature_in(fw_variant_);
    if(!client_.sendMessage(feature_in))
        return -1;

    // update feature configuration
    msp::msg::SetFeature feature_out(fw_variant_);
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

    if(!client_.sendMessage(feature_out))
        return -1;

    // make settings permanent and reboot
    if(!saveSettings())
        return -1;
    if(!reboot())
        return -1;

    return 1;
}


} // namespace msp
