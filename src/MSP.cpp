#include "MSP.hpp"

#include <future>
#include <iostream>
#include <type_traits>

namespace msp {

MSP::MSP() : port(io), wait(10) { }

MSP::MSP(const std::string &device, const uint baudrate) : port(io), wait(10) {
    connect(device, baudrate);
}

bool MSP::connect(const std::string &device, const uint baudrate) {
    this->device = device;
    try {
        port.open(device);
    }
    catch(const asio::system_error &e) {
        throw NoConnection(device, e.what());
    }

    port.set_option(asio::serial_port::baud_rate(baudrate));
    port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));

    // clear buffer for new session
    clear();

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
    catch(asio::system_error) { return false; }
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
        catch(asio::system_error) { success = false; }
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
            if(hasData()>=int(FRAME_SIZE+min_payload_size)) {
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
        catch(asio::system_error) { success = false; }
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
    catch(asio::system_error) { return false; }
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
        catch(asio::system_error) { success = false; }
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

    return write(msg);
}

DataID MSP::receiveData() {
    // wait for correct preamble start
    if(hasData()<1) {
        throw NoData();
    }

    while( char(read()) != '$');

    const char hdr = char(read());
    if(hdr != 'M')
        throw MalformedHeader('M', uint8_t(hdr));

    const char com_state = char(read());
    if(com_state != '>') {
        switch(com_state) {
        case '!': {
            // the sent message ID is unknown to the FC
            read(); // ignore data size
            const uint8_t id = read(); // get faulty ID
            throw UnknownMsgId(id);
        }
        default:
            throw MalformedHeader('>', uint8_t(com_state));
        }
    }

    // read data size
    const uint8_t data_size = read();

    // get ID of msg
    const uint8_t id = read();

    // read payload data
    const ByteVector data = read(data_size);

    // check CRC
    const uint8_t rcv_crc = read();
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

bool MSP::write(const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(lock_write);
    try {
        const std::size_t bytes_written = asio::write(port, asio::buffer(data.data(), data.size()));
        return (bytes_written==data.size());
    }
    catch(const asio::system_error &e) {
        throw NoConnection(device, e.what());
    }
}

size_t MSP::read(std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(lock_read);
    return asio::read(port, asio::buffer(data.data(), data.size()));
}

std::vector<uint8_t> MSP::read(std::size_t n_bytes) {
    std::vector<uint8_t> data(n_bytes);
    const size_t nread = read(data);
    assert(nread==n_bytes);
    return data;
}

int MSP::hasData() {
#if __unix__ || __APPLE__
    int available_bytes;
    if(ioctl(port.native_handle(), FIONREAD, &available_bytes)!=-1) {
        return available_bytes;
    }
    else {
        return -1;
    }
#elif _WIN32
    COMSTAT comstat;
    if (ClearCommError(port.native_handle(), NULL, &comstat) == true) {
        return comstat.cbInQue;
    }
    else {
        return -1;
    }
#else
#warning "hasData() will be unimplemented"
#endif
}

void MSP::clear() {
#if __unix__ || __APPLE__
    tcflush(port.native_handle(),TCIOFLUSH);
#elif _WIN32
    PurgeComm(port.native_handle(), PURGE_TXCLEAR);
#else
#warning "clear() will be unimplemented"
#endif
}

} // namespace msp
