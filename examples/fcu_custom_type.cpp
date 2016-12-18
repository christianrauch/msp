#include <FlightController.hpp>

#include <iostream>

struct MyIdent : public msp::Request {
    msp::ID id() const { return msp::ID::MSP_IDENT; }

    msp::ByteVector raw_data;

    void decode(const msp::ByteVector &data) {
        raw_data = data;
    }

};

struct Callbacks {
    void onIdent(const MyIdent &ident) {
        std::cout << "Raw Ident data: ";
        for(auto d : ident.raw_data) {
            std::cout << int(d) << ",";
        }
        std::cout << std::endl;
    }
};

int main(int argc, char *argv[]) {
    std::string device;
    if(argc>1)
        device = std::string(argv[1]);
    else
        device = "/dev/ttyUSB0";

    Callbacks cbs;
    fcu::FlightController fcu(device);
    fcu.initialise();

    // subscribe with costum type
    fcu.subscribe(&Callbacks::onIdent, &cbs);

    while(true) {
        fcu.handle_batch();
    }
}
