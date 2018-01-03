#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP

#include "Client.hpp"
#include "msp_msg.hpp"

namespace fcu {

enum class FirmwareType {
    MULTIWII,
    CLEANFLIGHT
};

class FlightController {
public:
    FlightController(const std::string &device, const size_t baudrate=115200);

    ~FlightController();

    void waitForConnection();

    void initialise();

    /**
     * @brief isFirmware determine firmware type (e.g. to distiguish accepted messages)
     * @param firmware_type type of firmware (enum FirmwareType)
     * @return true if firmware is firmware_type
     * @return false if firmware is not firmware_type
     */
    bool isFirmware(const FirmwareType firmware_type);

    bool isFirmwareMultiWii() { return isFirmware(FirmwareType::MULTIWII); }

    bool isFirmwareCleanflight() { return isFirmware(FirmwareType::CLEANFLIGHT); }

    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback pointer to callback function (class method)
     * @param context class with callback method
     * @param tp period of timer that will send subscribed requests (in seconds), by default this is 0 and requests are not sent periodically
     * @return pointer to subscription that is added to internal list
     */
    template<typename T, typename C>
    msp::client::SubscriptionBase* subscribe(void (C::*callback)(const T&), C *context, const double tp = 0.0) {
        return client.subscribe(callback, context, tp);
    }

    /**
     * @brief subscribe register callback function that is called when type is received
     * @param callback function (e.g. lambda, class method, function pointer)
     * @param tp period of timer that will send subscribed requests (in seconds), by default this is 0 and requests are not sent periodically
     * @return pointer to subscription that is added to internal list
     */
    template<typename T>
    msp::client::SubscriptionBase* subscribe(const std::function<void(const T&)> &callback, const double tp = 0.0) {
        return client.subscribe(callback, tp);
    }

    /**
     * @brief hasSubscription check if message ID is subscribed
     * @param id message ID
     * @return true if there is already a subscription
     * @return false if ID is not subscribed
     */
    bool hasSubscription(const msp::ID& id) {
        return client.hasSubscription(id);
    }

    /**
     * @brief getSubscription get pointer to subscription
     * @param id message ID
     * @return pointer to subscription
     */
    msp::client::SubscriptionBase* getSubscription(const msp::ID& id) {
        return client.getSubscription(id);
    }

    /**
     * @brief sendRequest send request with ID
     * @param id message ID of request
     * @return true on success
     * @return false on failure
     */
    bool sendRequest(const uint8_t id) {
        return client.sendRequest(id);
    }

    bool request(msp::Request &request, const double timeout = 0) {
        return client.request(request, timeout);
    }

    bool request_raw(const uint8_t id, msp::ByteVector &data, const double timeout = 0) {
        return client.request_raw(id, data, timeout);
    }

    bool respond(const msp::Response &response, const bool wait_ack=true) {
        return client.respond(response, wait_ack);
    }

    bool respond_raw(const uint8_t id, msp::ByteVector &data, const bool wait_ack=true) {
        return client.respond_raw(id, data, wait_ack);
    }

    void initBoxes();

    std::map<std::string, size_t> &getBoxNames() {
        return box_name_ids;
    }

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

    bool hasSensor(const msp::msg::Sensor &sensor) const {
        return sensors.count(sensor);
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

    bool setMotors(const std::array<uint16_t,msp::msg::N_MOTOR> &motor_values);

    /**
     * @brief arm arm or disarm FC
     * @param arm true: will arm FC, false: will disarm FC
     * @return true on success
     */
    bool arm(const bool arm);

    /**
     * @brief arm_block attempt to arm and wait for status feedback, e.g. this method will block until the FC is able to aim
     * @return
     */
    bool arm_block();

    /**
     * @brief disarm_block attempt to disarm and wait for status feedback
     * @return
     */
    bool disarm_block();

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

    /**
     * @brief enableRxMSP enable the "RX_MSP" feature
     * The features "RX_MSP", "RX_PARALLEL_PWM", "RX_PPM" and "RX_SERIAL" are
     * mutually exclusive. Hence one of the features "RX_PARALLEL_PWM", "RX_PPM"
     * or "RX_SERIAL" will be disabled if active.
     * @return true on success
     */
    bool enableRxMSP() {
        return updateFeatures(
            {"RX_MSP"}, // add
            {"RX_PARALLEL_PWM", "RX_PPM", "RX_SERIAL"} // remove
        );
    }

    bool reboot();

    bool writeEEPROM();

private:

    static const uint8_t MAX_MAPPABLE_RX_INPUTS = 8;

    msp::client::Client client;

    std::map<std::string, size_t> box_name_ids;

    msp::msg::Ident ident;

    std::set<msp::msg::Sensor> sensors;

    FirmwareType firmware;

    std::vector<uint8_t> channel_map;
};

} // namespace msp

#endif // FLIGHTCONTROLLER_HPP
