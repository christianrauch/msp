#include "MSP.hpp"

#include <iostream>

namespace msp {

void printData(const ByteVector &data) {
    std::cout<<"data("<<data.size()<<") START>>>"<<std::endl;
    for(auto d : data)
        std::cout<<(int)d<<std::endl;
    std::cout<<"<<<data("<<data.size()<<") END"<<std::endl;
}

MSP::MSP(const std::string &device) : sp(device), wait(10) { }

bool MSP::request(msp::Request &request) {
    if(!sendData(request.id()))
        return false;

    usleep(wait);

    try {
        const DataID pkg = receiveData();
        if(pkg.id==request.id())
            request.decode(pkg.data);
        return pkg.id==request.id();
    }
    catch(const MalformedHeader &e) {
        std::cerr<<e.what()<<std::endl;
        return false;
    }
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

        try {
            const DataID pkg = receiveData();
            success = (pkg.id==request.id());
            if(success)
                request.decode(pkg.data);
        }
        catch(const MalformedHeader &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(boost::system::system_error) { success = false; }
    }

    return true;
}

bool MSP::respond(msp::Response &response) {
    if(!sendData(response.id(), response.encode()))
        return false;

    usleep(wait);

    try {
        const DataID pkg = receiveData();
        return (pkg.id==response.id() && pkg.data.size()==0);
    }
    catch(const MalformedHeader &e) {
        std::cerr<<e.what()<<std::endl;
        return false;
    }
    catch(boost::system::system_error) { return false; }
}

bool MSP::respond_block(msp::Response &response) {
    bool success = false;
    while(success==false) {
        // write ID and data and skip to write again if error occurred
        if(!sendData(response.id(), response.encode())) {
            success = false;
            continue;
        }

        try {
            const DataID pkg = receiveData();
            success = (pkg.id==response.id() && pkg.data.size()==0);
        }
        catch(const MalformedHeader &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
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
    msg.push_back(data.size());                         // data size
    msg.push_back(id);                                  // message_id
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crc(id, data) );                     // crc

    return sp.write(msg);
}

DataID MSP::receiveData() {
    // wait for correct preamble start
    while( (char)sp.read() != '$');

    if( (char)sp.read() != 'M')
        throw MalformedHeader();

    if( (char)sp.read() != '>')
        throw MalformedHeader();

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
        throw WrongCRC(exp_crc, rcv_crc);

    return DataID(data,id);
}

uint8_t MSP::crc(const uint8_t id, const ByteVector &data) {
    uint8_t crc = data.size()^id;

    for(uint8_t d : data)
        crc = crc^d;

    return crc;
}

} // namespace msp
