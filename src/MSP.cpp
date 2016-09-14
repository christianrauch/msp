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

    std::cout<<"Requested "<<(int)request.id()<<std::endl;

    usleep(wait);

    try {
        const DataID pkg = receiveData();
        if(pkg.id==request.id()) {
            std::cout<<"accept id: "<<(int)pkg.id<<std::endl;
            request.decode(pkg.data);
        }
        else {
            std::cout<<"wrong id: "<<(int)pkg.id<<" (expected:"<<(int)request.id()<<")"<<std::endl;
        }
        return pkg.id==request.id();
    }
    catch(MalformedHeader) {
        std::cerr<<"Malformed header"<<std::endl;
        return false;
    }
    catch(boost::system::system_error) { return false; }
}

bool MSP::respond(msp::Response &response) {
    if(!sendData(response.id(), response.encode()))
        return false;

    std::cout<<"Responded "<<(int)response.id()<<std::endl;
    //printData(response.encode());

    usleep(wait);

    try {
        const DataID pkg = receiveData();
        return (pkg.id==response.id() && pkg.data.size()==0);
    }
    catch(MalformedHeader) { return false; }
    catch(boost::system::system_error) { return false; }
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
    std::cout<<"received id: "<<(int)id<<std::endl;

    //std::cout<<"going to read "<<(int)data_size<<" data bytes"<<std::endl;

    const ByteVector data = sp.read(data_size);

    printData(data);

    const uint8_t rcv_crc = sp.read();
    if(rcv_crc!=crc(id, data)) {
        //std::cerr<<"ignoring package with wrong check sum"<<std::endl;
        throw WrongCRC();
    }

    return DataID(data,id);
}

uint8_t MSP::crc(const uint8_t id, const ByteVector &data) {
    uint8_t crc = data.size()^id;

    for(uint8_t d : data)
        crc = crc^d;

    return crc;
}

} // namespace msp
