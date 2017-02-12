#include "MSP.hpp"

#include <future>
#include <iostream>
#include <type_traits>

namespace msp {

MSP::MSP() : wait(10) { }

MSP::MSP(const std::string &device, const uint baudrate) : wait(10) {
    connect(device, baudrate);
}

bool MSP::connect(const std::string &device, const uint baudrate) {
    sp.connect(device, baudrate);
    std::cout<<"Connected to: "<<device<<std::endl;
    return true;
}

bool MSP::request(msp::Request &request) {
    if(!sendData(request.id()))
        return false;

    std::this_thread::sleep_for(std::chrono::microseconds(wait));

    try {
        const DataID pkg = receiveData();
        if(pkg.id==uint8_t(request.id()))
            request.decode(pkg.data);
        return pkg.id==uint8_t(request.id());
    }
    catch(const MalformedHeader &e) {
        std::cerr<<e.what()<<std::endl;
        return false;
    }
    catch(const WrongCRC &e) {
        std::cerr<<e.what()<<std::endl;
        return false;
    }
    catch(const UnknownMsgId &e) {
        if(e.getInvalidID()==uint8_t(request.id())) {
            std::cerr<<e.what()<<std::endl;
        }
        return false;
    }
    catch(msp::NoData) { return false; }
    catch(boost::system::system_error) { return false; }
}

bool MSP::request_block(msp::Request &request) {
    bool success = false;
    while(success==false) {
        // write ID
        if(!sendData(request.id())) {
            success = false;
            continue;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(wait));

        try {
            const DataID pkg = receiveData();
            success = (pkg.id==uint8_t(request.id()));
            if(success)
                request.decode(pkg.data);
        }
        catch(const MalformedHeader &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(const WrongCRC &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(const UnknownMsgId &e) {
            if(e.getInvalidID()==uint8_t(request.id())) {
                std::cerr<<e.what()<<std::endl;
                return false;
            }
            success = false;
        }
        catch(msp::NoData) { success = false; }
        catch(boost::system::system_error) { success = false; }
    }

    return true;
}

bool MSP::request_wait(msp::Request &request, const uint wait_ms, const uint min_payload_size) {
    const std::chrono::milliseconds wait(wait_ms);

    bool success = false;
    while(success==false) {
        // send ID
        while(sendData(request.id())!=true);

        std::this_thread::sleep_for(wait);

        try {
            if(sp.hasData()>=int(FRAME_SIZE+min_payload_size)) {
                DataID pkg = receiveData();
                success = (pkg.id==uint8_t(request.id()));
                if(success)
                    request.decode(pkg.data);
            }
        }
        catch(const MalformedHeader &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(const WrongCRC &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(const UnknownMsgId &e) {
            if(e.getInvalidID()==uint8_t(request.id())) {
                std::cerr<<e.what()<<std::endl;
                return false;
            }
            success = false;
        }
        catch(msp::NoData) { success = false; }
        catch(boost::system::system_error) { success = false; }
    }

    return true;
}

bool MSP::respond(const msp::Response &response) {
    if(!sendData(response.id(), response.encode()))
        return false;

    std::this_thread::sleep_for(std::chrono::microseconds(wait));

    try {
        const DataID pkg = receiveData();
        return (pkg.id==uint8_t(response.id()) && pkg.data.size()==0);
    }
    catch(const MalformedHeader &e) {
        std::cerr<<e.what()<<std::endl;
        return false;
    }
    catch(boost::system::system_error) { return false; }
}

bool MSP::respond_block(const msp::Response &response) {
    bool success = false;
    while(success==false) {
        // write ID and data and skip to write again if error occurred
        if(!sendData(response.id(), response.encode())) {
            success = false;
            continue;
        }

        try {
            const DataID pkg = receiveData();
            success = (pkg.id==uint8_t(response.id()) && pkg.data.size()==0);
        }
        catch(const MalformedHeader &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(msp::NoData) { success = false; }
        catch(boost::system::system_error) { success = false; }
    }

    return true;
}

bool MSP::sendData(const uint8_t id, const ByteVector &data) {
    ByteVector msg;
    msg.reserve(6+data.size());

    msg.push_back('$');
    msg.push_back('M');
    msg.push_back('<');
    msg.push_back(uint8_t(data.size()));                // data size
    msg.push_back(id);                                  // message_id
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crc(id, data) );                     // crc

    return sp.write(msg);
}

DataID MSP::receiveData() {
    // wait for correct preamble start
    if(sp.hasData()<1) {
        throw NoData();
    }

    while( char(sp.read()) != '$');

    const char hdr = char(sp.read());
    if(hdr != 'M')
        throw MalformedHeader('M', uint8_t(hdr));

    const char com_state = char(sp.read());
    if(com_state != '>') {
        switch(com_state) {
        case '!': {
            // the sent message ID is unknown to the FC
            sp.read(); // ignore data size
            const uint8_t id = sp.read(); // get faulty ID
            throw UnknownMsgId(id);
        }
        default:
            throw MalformedHeader('>', uint8_t(com_state));
        }
    }

    // read data size
    const uint8_t data_size = sp.read();

    // get ID of msg
    const uint8_t id = sp.read();

    // read payload data
    const ByteVector data = sp.read(data_size);

    // check CRC
    const uint8_t rcv_crc = sp.read();
    const uint8_t exp_crc = crc(id, data);

    if(rcv_crc!=exp_crc)
        throw WrongCRC(id, exp_crc, rcv_crc);

    return DataID(data,id);
}

uint8_t MSP::crc(const uint8_t id, const ByteVector &data) {
    uint8_t crc = uint8_t(data.size())^id;

    for(uint8_t d : data)
        crc = crc^d;

    return crc;
}

} // namespace msp
