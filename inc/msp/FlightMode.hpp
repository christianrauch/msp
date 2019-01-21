#ifndef FLIGHTMODE_HPP
#define FLIGHTMODE_HPP

namespace fcu {

struct FlightMode {
    enum class PRIMARY_MODE : uint32_t {
        UNSET = 0,
        ANGLE,
        HORIZON,
        NAV_POSHOLD,
        NAV_CRUISE,
        NAV_RTH,
        NAV_WP,
        MANUAL
    };

    enum class SECONDARY_MODE : uint32_t {
        NONE         = 0,
        NAV_ALTHOLD  = 1 << 0,
        TURN_ASSIST  = 1 << 1,
        AIR_MODE     = 1 << 2,
        SURFACE      = 1 << 3,
        HEADING_HOLD = 1 << 4,
        HEADFREE     = 1 << 5,
        HEADADJ      = 1 << 6
    };

    enum class MODIFIER : uint32_t {
        NONE           = 0,
        ARM            = 1 << 0,
        CAMSTAB        = 1 << 1,
        BEEPER         = 1 << 2,
        LEDLOW         = 1 << 3,
        OSD_SW         = 1 << 4,
        TELEMETRY      = 1 << 5,
        BLACKBOX       = 1 << 6,
        FAILSAFE       = 1 << 7,
        HOME_RESET     = 1 << 8,
        GCS_NAV        = 1 << 9,
        FLAPERON       = 1 << 10,
        NAV_LAUNCH     = 1 << 11,
        SERVO_AUTOTRIM = 1 << 12,
        AUTOTUNE       = 1 << 13
    };

    PRIMARY_MODE primary;
    SECONDARY_MODE secondary;
    MODIFIER modifier;
};

}  // namespace fcu

#endif
