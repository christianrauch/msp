#include "MSP.hpp"

namespace msp {

MSP::MSP(const std::string &device) : sp(device), wait(10) { }

bool MSP::request(msp::Request &request) {
    if(!sendData(request.id))
        return false;

    usleep(wait);

    try {
        const ByteVector data = receiveData(request.id);
        request.decode(data);
    }
    catch(NoData) { return false; }
    catch(MalformedHeader) { return false; }
    catch(WrongMessageType) { return false; }

    return true;
}

bool MSP::respond(msp::Response &response) {
    if(!sendData(response.id, response.encode()))
        return false;

    usleep(wait);

    try {
        receiveData(response.id);
    }
    catch(NoData) { return true; }  // ACK, expect no data
    catch(MalformedHeader) { return false; }
    catch(WrongMessageType) { return false; }
}

bool MSP::sendData(const uint8_t id, const ByteVector &data) {
    //std::cout<<"sending data size: "<<data.size()<<std::endl;

    ByteVector msg;
    msg.push_back('$');
    msg.push_back('M');
    msg.push_back('<');
    msg.push_back(data.size());                         // data size
    msg.push_back(id);                                  // message_id
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crc(id, data) );                     // crc

    return sp.write(msg);
}

MSP::ByteVector MSP::receiveData(const uint8_t id) {
    // wait for correct preamble start
    while( (char)sp.read() != '$');

    if( (char)sp.read() != 'M')
        throw MalformedHeader();

    if( (char)sp.read() != '>')
        throw MalformedHeader();

    // read data size
    const uint8_t data_size = sp.read();

    // get ID of msg
    const uint8_t id_rcv = sp.read();
    //std::cout<<"id: "<<(int)id_rcv<<std::endl;

    // return if ID does not fit
    if(id!=id_rcv) {
        //std::cout<<"id: "<<(int)id_rcv<<" (expected: "<<(int)id<<")"<<std::endl;
        throw WrongMessageType();
    }

    if(data_size==0)
        throw NoData();

    //std::cout<<"going to read "<<(int)data_size<<" data bytes"<<std::endl;

    const ByteVector data = sp.read(data_size);

    const uint8_t rcv_crc = sp.read();
    if(rcv_crc!=crc(id, data)) {
        //std::cerr<<"ignoring package with wrong check sum"<<std::endl;
        throw WrongCRC();
    }

    return data;
}

uint8_t MSP::crc(const uint8_t id, const ByteVector &data) {
    uint8_t crc = data.size()^id;

    for(uint8_t d : data)
        crc = crc^d;

    return crc;
}

} // namespace msp
