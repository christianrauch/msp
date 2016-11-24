#include "MSP.hpp"

#include <future>
#include <iostream>
#include <type_traits>

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
    catch(WrongCRC &e) {
        std::cerr<<e.what()<<std::endl;
        return false;
    }
    catch(UnknownMsgId &e) {
        std::cerr<<e.what()<<std::endl;
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
        catch(WrongCRC &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(UnknownMsgId &e) {
            std::cerr<<e.what()<<std::endl;
            return false;
        }
        catch(msp::NoData) { success = false; }
        catch(boost::system::system_error) { success = false; }
    }

    return true;
}

bool MSP::request_timeout(msp::Request &request, unsigned int timeout_ms) {

    const std::chrono::milliseconds timeout(timeout_ms);

    bool success = false;
    while(success==false) {
        try {
            std::future<DataID> proc = std::async(std::launch::async, &MSP::receiveData, this);

            // write ID while waiting for data
            while(proc.wait_for(timeout)==std::future_status::timeout)
                while(sendData(request.id())!=true);

            DataID pkg = proc.get();
            success = (pkg.id==request.id());
            if(success)
                request.decode(pkg.data);
        }
        catch(const MalformedHeader &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(WrongCRC &e) {
            std::cerr<<e.what()<<std::endl;
            success = false;
        }
        catch(UnknownMsgId &e) {
            std::cerr<<e.what()<<std::endl;
            return false;
        }
        catch(msp::NoData) { success = false; }
        catch(boost::system::system_error) { success = false; }
    }

    return true;
}

bool MSP::respond(const msp::Response &response) {
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

bool MSP::sendData(const ID id, const ByteVector &data) {
    ByteVector msg;
    msg.reserve(6+data.size());

    msg.push_back('$');
    msg.push_back('M');
    msg.push_back('<');
    msg.push_back(data.size());                         // data size
    msg.push_back((std::underlying_type<ID>::type)id);  // message_id
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crc(id, data) );                     // crc

    return sp.write(msg);
}

DataID MSP::receiveData() {
    // wait for correct preamble start
    if(sp.poll()==0) {
        throw NoData();
    }

    while( (char)sp.read() != '$');

    if( (char)sp.read() != 'M')
        throw MalformedHeader();

    const uint8_t com_state = sp.read();
    switch((char)com_state) {
    case '>':
        // expected char
        break;
    case '!': {
        // the send message ID is unknown to the FC
        sp.read(); // ignore data size
        const uint8_t id = sp.read(); // get faulty ID
        throw UnknownMsgId(id);
        break;
    }
    default:
        throw MalformedHeader();
        break;
    }

    if( (char)com_state != '>') {
        throw MalformedHeader();
    }

    // read data size
    const uint8_t data_size = sp.read();

    // get ID of msg
    const ID id = (ID)sp.read();

    // read payload data
    const ByteVector data = sp.read(data_size);

    // check CRC
    const uint8_t rcv_crc = sp.read();
    const uint8_t exp_crc = crc(id, data);

    if(rcv_crc!=exp_crc)
        throw WrongCRC(id, exp_crc, rcv_crc);

    return DataID(data,id);
}

uint8_t MSP::crc(const ID id, const ByteVector &data) {
    uint8_t crc = data.size()^(std::underlying_type<ID>::type)id;

    for(uint8_t d : data)
        crc = crc^d;

    return crc;
}

} // namespace msp
