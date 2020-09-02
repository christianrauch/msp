#include <FlightController.hpp>
#include <msp_msg.hpp>

#define MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT 0x01

int main(int argc, char* argv[]) {
    msp::FirmwareVariant fw_variant = msp::FirmwareVariant::INAV;
    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    fcu::FlightController fcu;
    fcu.setLoggingLevel(msp::client::LoggingLevel::INFO);
    // wait for connection
    fcu.connect(device, baudrate);

    msp::msg::SetWp setwp(fw_variant);

    setwp.wp_no    = 255;
    setwp.action   = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    setwp.lat      = 40811854;
    setwp.lon      = 29360025;
    setwp.alt      = 1500;
    setwp.p1       = 80;
    setwp.p2       = 0;
    setwp.p3       = 0;
    setwp.nav_flag = 0;

    if(fcu.sendMessage(setwp)) {
        std::cout << "Acknowled";
    }
    else
        std::cout << "wasted";
}