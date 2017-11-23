#include <string>
#include <iostream>

#include <MSP.hpp>
#include <msp_msg.hpp>
#include <msg_print.hpp>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    msp::MSP msp(device, baudrate);

    // wait for established connection
    {
    msp::msg::Ident ident;
    msp.request_wait(ident, 100);
    }

    std::cout << "MSP ready..." << std::endl;

    msp::msg::ApiVersion api_version;
    if(msp.request_block(api_version))
        std::cout << api_version << std::endl;
    else
        std::cerr << "Could not determine Cleanflight API version." << std::endl;

    msp::msg::FcVariant fc_variant;
    if(msp.request_block(fc_variant))
        std::cout << fc_variant << std::endl;

    msp::msg::FcVersion fc_version;
    if(msp.request_block(fc_version))
        std::cout << fc_version << std::endl;

    msp::msg::BoardInfo board_info;
    if(msp.request_block(board_info))
        std::cout << board_info << std::endl;

    msp::msg::BuildInfo build_info;
    if(msp.request_block(build_info))
        std::cout << build_info << std::endl;

    msp::msg::RxConfig rx_config;
    msp.request_block(rx_config);

    msp::msg::Feature feature;
    if(msp.request_block(feature))
        std::cout<<feature<<std::endl;

    msp::msg::RxMap rx_map;
    if(msp.request_block(rx_map))
        std::cout<<rx_map<<std::endl;

    return 0;
}
