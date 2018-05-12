#ifndef VARIANTS_HPP
#define VARIANTS_HPP

#include <map>

namespace msp {

enum class FirmwareVariant : int {
    MWII = 1,
    BAFL = 2,
    BTFL = 3,
    CLFL = 4,
    INAV = 5,
    RCFL = 6
};

std::map<std::string,FirmwareVariant> variant_map = 
{
    {"MWII", FirmwareVariant::MWII}, 
    {"BAFL", FirmwareVariant::BAFL}, 
    {"BTFL", FirmwareVariant::BTFL}, 
    {"CLFL", FirmwareVariant::CLFL}, 
    {"INAV", FirmwareVariant::INAV}, 
    {"RCFL", FirmwareVariant::RCFL}
};
    
}
#endif
