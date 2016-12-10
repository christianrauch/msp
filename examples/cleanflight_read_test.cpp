#include <string>
#include <iostream>

#include <MSP.hpp>
#include <msp_msg.hpp>
#include <msg_print.hpp>

int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    msp::MSP msp(device);

    // wait for established connection
    {
    msp::Ident ident;
    msp.request_wait(ident, 100);
    }

    std::cout << "MSP ready..." << std::endl;

    msp::ApiVersion api_version;
    if(msp.request_block(api_version))
        std::cout << api_version << std::endl;
    else
        std::cerr << "Could not determine Cleanflight API version." << std::endl;

    msp::FcVariant fc_variant;
    if(msp.request_block(fc_variant))
        std::cout << fc_variant << std::endl;

    msp::FcVersion fc_version;
    if(msp.request_block(fc_version))
        std::cout << fc_version << std::endl;

    msp::BoardInfo board_info;
    if(msp.request_block(board_info))
        std::cout << board_info << std::endl;

    msp::BuildInfo build_info;
    if(msp.request_block(build_info))
        std::cout << build_info << std::endl;
}
