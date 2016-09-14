#include <iostream>
#include <MSP.hpp>

#include <unistd.h>
#include <chrono>

int main(int argc, char *argv[]) {
    std::cout << "Hello World!" << std::endl;

    msp::MSP msp("/dev/ttyUSB0");
    msp.setWait(1);

    //sleep(10);

    std::cout<<"ready"<<std::endl;

    bool success;

    // try connecting until first package is received
    msp::Ident ident;
    do {
        try {
            success = msp.request(ident);
            std::cout<<"success: "<<success<<std::endl;
        }
        catch(boost::system::system_error &e) {
            //std::cerr<<"Cannot reach MSP yet. "<<e.what()<<std::endl;
        }

    } while(success==false);

    std::cout<<"version: "<<(int)ident.version<<std::endl;

    msp::RcTuning rc_tuning;
    do {
        success = msp.request(rc_tuning);
    } while(success==false);
    std::cout<<"success: "<<success<<std::endl;
    std::cout<<"thr expo: "<<(int)rc_tuning.Throttle_EXPO<<std::endl;

    msp::SetRcTuning set_rc_tuning(rc_tuning);
    set_rc_tuning.Throttle_EXPO = 73;
    do {
        success = msp.respond(set_rc_tuning);
    } while(success==false);
    std::cout<<"success: "<<success<<std::endl;

    msp::RcTuning rc_tuning2;
    do {
        success = msp.request(rc_tuning2);
    } while(success==false);
    std::cout<<"success: "<<success<<std::endl;
    std::cout<<"thr expo: "<<(int)rc_tuning2.Throttle_EXPO<<std::endl;


//    auto start = std::chrono::steady_clock::now();
//    unsigned int n_msg = 0;
//    while(n_msg!=500) {
//        //msp::Status status;
//        msp::RawImu status;
//        try {
//            success = msp.request(status);
//        }
//        catch(boost::system::system_error &e) {
//            //std::cerr<<"No data. "<<e.what()<<std::endl;
//        }

//        if(success) {
//            //std::cout<<"time: "<<status.time<<" us"<<std::endl;
//            success = false;
//            n_msg++;
//        }
//    }
//    auto end = std::chrono::steady_clock::now();

//    std::cout<<"duration: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<" ms"<<std::endl;

    return 0;
}
