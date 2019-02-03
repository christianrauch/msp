#ifndef VARIANTS_HPP
#define VARIANTS_HPP

#include <map>
#include <string>

namespace msp {

/**
 * @brief Enum of firmware variants
 *
 */
enum class FirmwareVariant : int {
    NONE = 0, /**< not specified */
    MWII = 1, /**< MultiWii */
    BAFL = 2, /**< BetaFlight */
    BTFL = 3, /**< ButterFlight */
    CLFL = 4, /**< CleanFlight */
    INAV = 5, /**< INAV */
    RCFL = 6  /**< RaceFlight */
};

const static std::map<std::string, FirmwareVariant> variant_map = {
    {"MWII", FirmwareVariant::MWII},
    {"BAFL", FirmwareVariant::BAFL},
    {"BTFL", FirmwareVariant::BTFL},
    {"CLFL", FirmwareVariant::CLFL},
    {"INAV", FirmwareVariant::INAV},
    {"RCFL", FirmwareVariant::RCFL}};

/**
 * @brief Converts a FirmwareVariant into a matching string
 * @param variant Enum of FirmwareVariant
 * @returns A string matching the firmware type
 */
inline std::string firmwareVariantToString(FirmwareVariant variant) {
    std::string var;
    switch(variant) {
    case FirmwareVariant::MWII:
        var = "MWII";
        break;
    case FirmwareVariant::BAFL:
        var = "BAFL";
        break;
    case FirmwareVariant::BTFL:
        var = "BTFL";
        break;
    case FirmwareVariant::CLFL:
        var = "CLFL";
        break;
    case FirmwareVariant::INAV:
        var = "INAV";
        break;
    case FirmwareVariant::RCFL:
        var = "RCFL";
        break;
    default:
        var = "NONE";
    }
    return var;
}

}  // namespace msp
#endif
