#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP

#include "Client.hpp"
#include "FlightMode.hpp"
#include "PeriodicTimer.hpp"
#include "msp_msg.hpp"

namespace fcu {

enum class ControlSource { NONE, SBUS, MSP, OTHER };

class FlightController {
public:
    /**
     * @brief FlightController Constructor
     */
    FlightController();

    /**
     * @brief ~FlightController Destructor
     */
    ~FlightController();

    /**
     * @brief Connects to a physical flight controller, which includes
     * connecting the internal Client object and querying the flight controller
     * for information necessary to configure the internal state to match the
     * capabiliites of the flight controller.
     * @param timeout Timeout passed to each internal operation (seconds)
     * @return True on success
     */
    bool connect(const std::string &device, const size_t baudrate = 115200,
                 const double &timeout = 0.0, const bool print_info = false);

    /**
     * @brief Stops MSP control if active, and disconnects the internal Client
     * object
     * @return True on success
     */
    bool disconnect();

    /**
     * @brief Tests connection to a flight controller
     * @return True if connected
     */
    bool isConnected() const;

    /**
     * @brief Sets data structure with all flags governing flight mode
     * @param mode FlightMode object
     */
    void setFlightMode(const FlightMode mode);

    /**
     * @brief Queries data structure with all flags governing flight mode
     * @return FlightMode object matchhing current flight mode flags
     */
    FlightMode getFlightMode() const;

    /**
     * @brief Sets which instruction source the flight controller should
     * listen to. Also starts periodic MSP control message if MSP is selected
     * @param mode FlightMode object
     */
    void setControlSource(const ControlSource source);

    /**
     * @brief Queries the currently active control source
     * @return ControlSource object matching current control source
     */
    ControlSource getControlSource();

    /**
     * @brief Sets roll, pitch, yaw, and throttle values. They are sent
     * immediately and stored for resending if automatic MSP control is enabled.
     * @param rpyt An array of doubles representing roll, pitch, yaw, and
     * throttle in that order. Values are expected to be in the range -1.0 to
     * +1.0, mapping to 1000us to 2000us pulse widths.
     */
    void setRPYT(std::array<double, 4> &&rpyt);

    /**
     * @brief Method used to generate the Rc message sent to the flight
     * controller
     */
    void generateMSP();

    /**
     * @brief Sends message to flight controller to save all settings
     */
    bool saveSettings();

    /**
     * @brief Starts message to flight controller signalling that it should
     * reboot. Will result in the serial device disappearing if using a direct
     * USB connection to the board.
     */
    bool reboot();

    /**
     * @brief Queries the currently set firmware variant (Cleanflight,
     * Betaflight, etc.)
     * @return msp::FirmwareVariant enum matching the current firmware
     */
    msp::FirmwareVariant getFwVariant() const;

    /**
     * @brief Queries the currently set protocol (MSPv1 or MSPv2)
     * @return integer matching the protocol version
     */
    int getProtocolVersion() const;

    /**
     * @brief Queries the currently set board name
     * @return std::String of the board name
     */
    std::string getBoardName() const;

    /**
     * @brief Set the verbosity of the output
     * @param level LoggingLevel matching the desired amount of output (default
     * to WARNING)
     */
    void setLoggingLevel(const msp::client::LoggingLevel &level);

    /**
     * @brief Set RC channels in order: roll, pitch, yaw, throttle by using
     * channel mapping
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
    bool setRc(const uint16_t roll, const uint16_t pitch, const uint16_t yaw,
               const uint16_t throttle, const uint16_t aux1 = 1000,
               const uint16_t aux2 = 1000, const uint16_t aux3 = 1000,
               const uint16_t aux4              = 1000,
               const std::vector<uint16_t> auxs = std::vector<uint16_t>());

    /**
     * @brief Set RC channels in raw order as it is interpreted by the FC
     * @param channels list of channel values (1000-2000)
     * @return
     */
    bool setRc(const std::vector<uint16_t> channels);

    /**
     * @brief Register callback function that is called when type is received
     * @param callback Pointer to callback function (class method)
     * @param context Object with callback method
     * @param tp Period of timer that will send subscribed requests (in
     * seconds), by default this is 0 and requests are not sent periodically
     * @return Pointer to subscription that is added to internal list
     */
    template <typename T, typename C,
              class = typename std::enable_if<
                  std::is_base_of<msp::Message, T>::value>::type>
    std::shared_ptr<msp::client::SubscriptionBase> subscribe(
        void (C::*callback)(const T &), C *context, const double tp = 0.0) {
        return client_.subscribe(callback, context, tp);
    }

    /**
     * @brief Register callback function that is called when type is received
     * @param callback Function to be called (e.g. lambda, class method,
     * function pointer)
     * @param tp Period of timer that will send subscribed requests (in
     * seconds), by default this is 0 and requests are not sent periodically
     * @return Pointer to subscription that is added to internal list
     */
    template <typename T, class = typename std::enable_if<
                              std::is_base_of<msp::Message, T>::value>::type>
    std::shared_ptr<msp::client::SubscriptionBase> subscribe(
        const std::function<void(const T &)> &callback, const double tp = 0.0) {
        return client_.subscribe(callback, tp);
    }

    /**
     * @brief Check if message ID is subscribed
     * @param id Message ID
     * @return True if there is already a subscription
     */
    bool hasSubscription(const msp::ID &id) const {
        return client_.hasSubscription(id);
    }

    /**
     * @brief Get pointer to subscription
     * @param id Message ID
     * @return Pointer to subscription
     */
    std::shared_ptr<msp::client::SubscriptionBase> getSubscription(
        const msp::ID &id) {
        return client_.getSubscription(id);
    }

    /**
     * @brief Sends a message to the flight controller
     * @param message Reference to a Message-derived object
     * @param timeout Number of seconds to wait for response. Default value of 0
     * means no timeout.
     * @return True on success
     */
    bool sendMessage(msp::Message &message, const double timeout = 0) {
        return client_.sendMessage(message, timeout);
    }

    /**
     * @brief Queries the flight controller for Box (flight mode) information
     */
    void initBoxes();

    /**
     * @brief Gets the information collected by the initBoxes() method
     * @return Reference to the internal mapping of strings to box IDs
     */
    const std::map<std::string, size_t> &getBoxNames() const {
        return box_name_ids_;
    }

    /**
     * @brief Queries the cached flight controller information to see
     * if a particular capability is present
     * @return True if the sensor is present
     */
    bool hasCapability(const msp::msg::Capability &cap) const {
        return capabilities_.count(cap);
    }

    /**
     * @brief Queries for the presence of the BIND capability
     * @return True if the BIND capaibility is present
     */
    bool hasBind() const { return hasCapability(msp::msg::Capability::BIND); }

    /**
     * @brief Queries for the presence of the DYNBAL capability
     * @return True if the DYNBAL capaibility is present
     */
    bool hasDynBal() const {
        return hasCapability(msp::msg::Capability::DYNBAL);
    }

    /**
     * @brief Queries for the presence of the FLAP capability
     * @return True if the FLAP capaibility is present
     */
    bool hasFlap() const { return hasCapability(msp::msg::Capability::FLAP); }

    /**
     * @brief Queries the cached flight controller information to see
     * if a particular sensor is present
     * @return True if the sensor is present
     */
    bool hasSensor(const msp::msg::Sensor &sensor) const {
        return sensors_.count(sensor);
    }

    /**
     * @brief Queries for the presence of an accelerometer
     * @return True if there is an accelerometer
     */
    bool hasAccelerometer() const {
        return hasSensor(msp::msg::Sensor::Accelerometer);
    }

    /**
     * @brief Queries for the presence of a barometer
     * @return True if there is a barometer
     */
    bool hasBarometer() const { return hasSensor(msp::msg::Sensor::Barometer); }

    /**
     * @brief Queries for the presence of a magentometer
     * @return True if there is a magentometer
     */
    bool hasMagnetometer() const {
        return hasSensor(msp::msg::Sensor::Magnetometer);
    }

    /**
     * @brief Queries for the presence of a GPS
     * @return True if there is a GPS
     */
    bool hasGPS() const { return hasSensor(msp::msg::Sensor::GPS); }

    /**
     * @brief Queries for the presence of a sonar
     * @return True if there is a sonar
     */
    bool hasSonar() const { return hasSensor(msp::msg::Sensor::Sonar); }

    /**
     * @brief Queries the flight controller to see if a status is active
     * @return True if status if active
     */
    bool isStatusActive(const std::string &status_name);

    /**
     * @brief Queries the flight controller to see if the ARM status is active.
     * Not to be confused with armSet(), which queries whether the flight
     * controller has been instructued to turn on the ARM status.
     * @return True if the ARM status is active
     */
    bool isArmed() { return isStatusActive("ARM"); }

    /**
     * @brief Queries the flight controller to see if the FAILSAFE status is
     * active.
     * @return True if the FAILSAFE status is active
     */
    bool isStatusFailsafe() { return isStatusActive("FAILSAFE"); }

    /**
     * @brief Directly sets motor values using SetMotor message
     * @return True on successful message delivery
     */
    bool setMotors(const std::array<uint16_t, msp::msg::N_MOTOR> &motor_values);

    /**
     * @brief Enable and disable features on the FC
     * To apply updates, changes will be written to the EEPROM and the FC will
     * reboot.
     * @param add set of features to enable
     * @param remove set of features to disable
     * @return 1 if features have been changed
     * @return 0 if no changes have been applied
     * @return -1 on failure
     */
    int updateFeatures(
        const std::set<std::string> &add    = std::set<std::string>(),
        const std::set<std::string> &remove = std::set<std::string>());

private:
    // Client instance for managing the actual comms with the flight controller
    msp::client::Client client_;

    // parameters updated by the connect method to cache flight controller info
    std::string board_name_;
    msp::FirmwareVariant fw_variant_;
    int msp_version_;
    std::map<std::string, size_t> box_name_ids_;
    std::set<msp::msg::Sensor> sensors_;
    std::array<uint8_t, msp::msg::MAX_MAPPABLE_RX_INPUTS> channel_map_;
    std::set<msp::msg::Capability> capabilities_;

    // parameters updated by the user, and consumed by MSP control messages
    std::array<double, 4> rpyt_;
    FlightMode flight_mode_;

    std::mutex msp_updates_mutex;

    ControlSource control_source_;

    msp::PeriodicTimer msp_timer_;
};

}  // namespace fcu

#endif  // FLIGHTCONTROLLER_HPP
