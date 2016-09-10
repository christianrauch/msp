#include "MSP.hpp"

MSP::MSP(const std::string &device) : sp(device) { }

//MSP::ByteVector MSP::compileRequest(const uint8_t request_id) {
//    ByteVector data;
//    data.push_back('$');
//    data.push_back('M');
//    data.push_back('<');
//    data.push_back(0);              // data size
//    data.push_back(request_id);     // message_id
//    data.push_back(0^request_id);   // crc
//    return data;
//}

//bool MSP::sendRequest(const uint8_t request_id) {
//    return sp.write(compileRequest(request_id));
//}

bool MSP::sendData(const uint8_t id, const ByteVector &data) {
    std::cout<<"data size: "<<data.size()<<std::endl;

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
        return ByteVector(0);

    if( (char)sp.read() != '>')
        return ByteVector(0);

    // read data size
    const uint8_t data_size = sp.read();

    std::cout<<"going to read "<<(int)data_size<<" data bytes"<<std::endl;

    // get ID of msg
    const uint8_t id_rcv = sp.read();
    std::cout<<"id: "<<(int)id_rcv<<std::endl;

    // return if ID does not fit
    if(id!=id_rcv)
        return ByteVector(0);

    const ByteVector data = sp.read(data_size);

    const uint8_t rcv_crc = sp.read();
    if(rcv_crc!=crc(id, data)) {
        std::cerr<<"ignoring package with wrong check sum"<<std::endl;
        return ByteVector(0);
    }

    return data;
}

uint8_t MSP::crc(const uint8_t id, const ByteVector &data) {
    uint8_t crc = data.size()^id;

    for(uint8_t d : data)
        crc = crc^d;

    return crc;
}
