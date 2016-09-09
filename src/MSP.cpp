#include "MSP.hpp"

MSP::MSP(const std::string &device) : sp(device) { }

MSP::ByteVector MSP::compileRequest(const uint8_t request_id) {
    ByteVector data;
    data.push_back('$');
    data.push_back('M');
    data.push_back('<');
    data.push_back(0);              // data size
    data.push_back(request_id);     // message_id
    data.push_back(0^request_id);   // crc
    return data;
}

bool MSP::sendRequest(const uint8_t request_id) {
    return sp.write(compileRequest(request_id));
}

std::tuple<MSP::ByteVector, uint8_t> MSP::readData() {
    ByteVector data;
    uint8_t id = 0;

    // wait for correct preamble start
    while( (char)sp.read() != '$');

    if( (char)sp.read() != 'M')
        return std::make_tuple(data, id);

    if( (char)sp.read() != '>')
        return std::make_tuple(data, id);

    // read data size
    const uint8_t data_size = sp.read();

    std::cout<<"going to read "<<(int)data_size<<" data bytes"<<std::endl;

    id = sp.read();

    std::cout<<"id: "<<(int)id<<std::endl;

    data = sp.read(data_size);

    // TODO: check sum
    const uint8_t crc = sp.read();

    return std::make_tuple(data, id);
}
