#ifndef VARIANTS_HPP
#define VARIANTS_HPP

#include <map>
#include <string>

namespace msp {

enum class FirmwareVariant : int {
    NONE = 0,
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

std::string firmwareVariantToString (FirmwareVariant variant)
{
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
    
}
#endif
