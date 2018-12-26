#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP

#include "Client.hpp"
#include "msp_msg.hpp"
#include "flightmode.hpp"
#include "periodic_timer.hpp"

namespace fcu {

enum class ControlSource : uint8_t
{
    NONE,
    SBUS,
    MSP,
    OTHER
};

class FlightController {
public:
    FlightController(const std::string &device, const size_t baudrate=115200);

    ~FlightController();
    
    bool connect(bool blocking = false, double timeout = 0.0);
    bool disconnect(bool blocking = false, double timeout = 0.0);
    void arm(bool blocking = false);
    void disarm(bool blocking = false);
    bool armSet();
    
    void setFlightMode(FlightMode mode);
    FlightMode getFlightMode();
    
    void setControlSource(ControlSource source);
    ControlSource getControlSource();
    
    void setRPYT(std::array<double,4>& rpyt);
    
    void startMspControl();
    void stopMspControl();
    void generateMSP();
    
    bool saveSettings();
    bool reboot();
    
    
    msp::FirmwareVariant getFwVariant();
    
    int getProtocol();
    
    std::string getBoardName();
    
    
    /**
     * @brief setRc set RC channels in order: roll, pitch, yaw, throttle by using channel mapping
     * @param roll
     * @param pitch
     * @param yaw
     * @param throttle
     * @param aux1
     * @param aux2
     * @param aux3
     * @param aux4
     * @param auxs
     * @return
     */
    bool setRc(const uint16_t roll, const uint16_t pitch,
               const uint16_t yaw, const uint16_t throttle,
               const uint16_t aux1 = 1000, const uint16_t aux2 = 1000,
               const uint16_t aux3 = 1000, const uint16_t aux4 = 1000,
               const std::vector<uint16_t> auxs = std::vector<uint16_t>());
    
    /**
     * @brief setRc set RC channels in raw order as it is interpreted by the FC
     * @param channels list of channel values (1000-2000)
     * @return
     */
    bool setRc(const std::vector<uint16_t> channels);
    
    
    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback pointer to callback function (class method)
     * @param context class with callback method
     * @param tp period of timer that will send subscribed requests (in seconds), by default this is 0 and requests are not sent periodically
     * @return pointer to subscription that is added to internal list
     */
    template<typename T, typename C>
    std::shared_ptr<msp::client::SubscriptionBase> subscribe(void (C::*callback)(T&), C *context, const double tp = 0.0) {
        return client_.subscribe(callback, context, tp);
    }

    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback function (e.g. lambda, class method, function pointer)
     * @param tp period of timer that will send subscribed requests (in seconds), by default this is 0 and requests are not sent periodically
     * @return pointer to subscription that is added to internal list
     */
    template<typename T>
    std::shared_ptr<msp::client::SubscriptionBase> subscribe(const std::function<void(T&)> &callback, const double tp = 0.0) {
        return client_.subscribe(callback, tp);
    }

    /**
     * @brief hasSubscription check if message ID is subscribed
     * @param id message ID
     * @return true if there is already a subscription
     * @return false if ID is not subscribed
     */
    bool hasSubscription(const msp::ID& id) {
        return client_.hasSubscription(id);
    }

    /**
     * @brief getSubscription get pointer to subscription
     * @param id message ID
     * @return pointer to subscription
     */
    std::shared_ptr<msp::client::SubscriptionBase> getSubscription(const msp::ID& id) {
        return client_.getSubscription(id);
    }
    
    bool sendMessage(msp::Message &request, const double timeout = 0) {
        return client_.sendMessage(request, timeout);
    }
    
    
    


    
    void initBoxes();

    std::map<std::string, size_t> &getBoxNames() {
        return box_name_ids_;
    }
/*
    bool hasCapability(const msp::msg::Capability &cap) const {
        return ident.capabilities.count(cap);
    }

    bool hasBind() const {
        return hasCapability(msp::msg::Capability::BIND);
    }

    bool hasDynBal() const {
        return hasCapability(msp::msg::Capability::DYNBAL);
    }

    bool hasFlap() const {
        return hasCapability(msp::msg::Capability::FLAP);
    }
*/
    bool hasSensor(const msp::msg::Sensor &sensor) const {
        return sensors_.count(sensor);
    }

    bool hasAccelerometer() const {
        return hasSensor(msp::msg::Sensor::Accelerometer);
    }

    bool hasBarometer() const {
        return hasSensor(msp::msg::Sensor::Barometer);
    }

    bool hasMagnetometer() const {
        return hasSensor(msp::msg::Sensor::Magnetometer);
    }

    bool hasGPS() const {
        return hasSensor(msp::msg::Sensor::GPS);
    }

    bool hasSonar() const {
        return hasSensor(msp::msg::Sensor::Sonar);
    }

    bool isStatusActive(const std::string& status_name);

    bool isArmed() { return isStatusActive("ARM"); }

    bool isStatusFailsafe() { return isStatusActive("FAILSAFE"); }

    


    bool setMotors(const std::array<uint16_t,msp::msg::N_MOTOR> &motor_values);


    /**
     * @brief updateFeatures enable and disable features on the FC
     * To apply updates, changes will be written to the EEPROM and the FC will reboot.
     * @param add set of features to enable
     * @param remove set of features to disable
     * @return 1 if features have been changed
     * @return 0 if no changes have been applied
     * @return -1 on failure
     */
    int updateFeatures(const std::set<std::string> &add = std::set<std::string>(),
                       const std::set<std::string> &remove = std::set<std::string>());


    //bool writeEEPROM();

private:

    msp::client::Client client_;
    
    //set by user
    std::string device_;
    int baudrate_;
    
    //configuration params
    std::string board_name_;
    msp::FirmwareVariant fw_variant_;
    int msp_version_;
    
    std::map<std::string, size_t> box_name_ids_;
    std::set<msp::msg::Sensor> sensors_;
    static const uint8_t MAX_MAPPABLE_RX_INPUTS = msp::msg::MAX_MAPPABLE_RX_INPUTS;
    std::array<uint8_t,MAX_MAPPABLE_RX_INPUTS> channel_map_;
    
    
    //auto updates
    std::mutex msp_updates_mutex;
    std::array<double,4> rpyt_;
    bool connected_;
    bool armed_;
    FlightMode flight_mode_;
    ControlSource control_source_;
    
    msp::PeriodicTimer msp_timer_;

};

} // namespace msp

#endif // FLIGHTCONTROLLER_HPP
