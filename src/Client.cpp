#include <Client.hpp>
#include "SerialPortImpl.cpp"

#include <iostream>

namespace msp {

PeriodicTimer::PeriodicTimer(std::function<void()> funct, const double period_seconds)
    : funct(funct), running(false)
{
    period_us = std::chrono::duration<size_t, std::micro>(size_t(period_seconds*1e6));
}

void PeriodicTimer::start() {
    // only start thread if period is above 0
    if(!(period_us.count()>0))
        return;
    mutex_timer.lock();
    thread_ptr = std::shared_ptr<std::thread>(new std::thread(
    [this]{
        running = true;
        while(running) {
            // call function and wait until end of period or stop is called
            const auto tstart = std::chrono::high_resolution_clock::now();
            funct();
            if (mutex_timer.try_lock_until(tstart + period_us)) {
                mutex_timer.unlock();
            }
        } // while running
    }
    ));
}

void PeriodicTimer::stop() {
    running = false;
    mutex_timer.unlock();
    if(thread_ptr!=nullptr && thread_ptr->joinable()) {
        thread_ptr->join();
    }
}

void PeriodicTimer::setPeriod(const double period_seconds) {
    stop();
    period_us = std::chrono::duration<size_t, std::micro>(size_t(period_seconds*1e6));
    start();
}

} // namespace msp

namespace msp {
namespace client {

Client::Client() : pimpl(new SerialPortImpl), running(false), print_warnings(false), version(1) { }

Client::~Client() {
    for(const std::pair<msp::ID, msp::Message*> d : subscribed_requests)
        delete d.second;

    for(const std::pair<msp::ID, SubscriptionBase*> s : subscriptions)
        delete s.second;
}

void Client::connect(const std::string &device, const size_t baudrate) {
    pimpl->port.open(device);

    pimpl->port.set_option(asio::serial_port::baud_rate(baudrate));
    pimpl->port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    pimpl->port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    pimpl->port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
}

void Client::start() {
    thread = std::thread([this]{
        running = true;
        while(running) { processOneMessage(); }
    });
}

void Client::stop() {
    running = false;
    pimpl->io.stop();
    pimpl->port.close();
    thread.join();
}

uint8_t Client::read() {
    bool debug = false;
    if(pimpl->buffer.sgetc()==EOF) {
        if (debug) std::cout << "buffer returned EOF" << std::endl;
        asio::read(pimpl->port, pimpl->buffer, asio::transfer_exactly(1));
        if (debug) std::cout << "byte read" << std::endl;
    }
    if (debug) std::cout << "extracting char" <<std::endl;
    uint8_t rc = uint8_t(pimpl->buffer.sbumpc());
    if (debug) std::cout << "char extracted" <<std::endl;
    return rc;
}

int Client::request(msp::Message &request, const double timeout) {
    msp::ByteVector data;
    const int success = request_raw(uint8_t(request.id()), data, timeout);
    if(success==1) { request.decode(data); }
    return success;
}

int Client::request_raw(const uint8_t id, ByteVector &data, const double timeout) {
    // send request
    if(!sendRequest(id)) { std::cout << "sendRequest failed" << std::endl; return false; }

    // wait for thread to received message
    std::unique_lock<std::mutex> lock(mutex_cv_request);
    const auto predicate = [&]{
        mutex_request.lock();
        const bool received = (request_received.id==id);
        // unlock to wait for next message
        if(!received) { mutex_request.unlock(); }
        return received;
    };

    if(timeout>0) {
        if(!cv_request.wait_for(lock, std::chrono::milliseconds(size_t(timeout*1e3)), predicate))
            return -1;
    }
    else {
        cv_request.wait(lock, predicate);
    }

    // check message status and decode
    const bool success = request_received.status==OK;
    if(success) { data = request_received.data; }
    mutex_request.unlock();
    if (!success) std::cout << "request status not OK" << std::endl; 
    return success;
}

bool Client::respond(const msp::Message &response, const bool wait_ack) {
    return respond_raw(uint8_t(response.id()), response.encode(), wait_ack);
}

bool Client::respond_raw(const uint8_t id, const ByteVector &data, const bool wait_ack) {
    // send response
    if(!sendData(id, data)) { return false; }

    if(!wait_ack)
        return true;

    // wait for thread to received message
    std::unique_lock<std::mutex> lock(mutex_cv_ack);
    cv_ack.wait(lock, [&]{
        mutex_request.lock();
        const bool received = (request_received.id==id);
        // unlock to wait for next message
        if(!received) { mutex_request.unlock(); }
        return received;
    });

    // check status, expect ACK without payload
    const bool success = request_received.status==OK;
    mutex_request.unlock();
    return success;
}
/*
uint8_t Client::crc(const uint8_t id, const ByteVector &data) {
    switch (version) {
    case 1:
        return crcV1(id,data);
    case 2:
        return crcV2(data);
    }
}
*/
bool Client::sendData(const uint32_t id, const ByteVector &data) {
    switch (version) {
    case 1:
        return sendDataV1(uint8_t(id),data);
    case 2:
        return sendDataV2(uint16_t(id),data);
    }
    return false;
}

void Client::processOneMessage() {
    //std::cout << "processOneMessage" << std::endl;
    std::lock_guard<std::mutex> lck(mutex_buffer);
    asio::error_code ec;
    const std::size_t bytes_transferred = asio::read_until(pimpl->port, pimpl->buffer, "$", ec);
    if (ec == asio::error::operation_aborted) {
        //operation_aborted error probably means the client is being closed
        return;
    }
    // ignore and remove header bytes
    pimpl->buffer.consume(bytes_transferred);

    // message version
    int ver = 0;
    const uint8_t ver_marker = read();
    if (ver_marker == 'M') ver = 1;
    if (ver_marker == 'X') ver = 2;
    if (ver == 0) {
        std::cerr << "Message marker " << ver_marker << " is not recognised!" << std::endl;
    }
    
    ReceivedMessage recv_msg;
    if (ver == 2)
        recv_msg = processOneMessageV2();
    else
        recv_msg = processOneMessageV1();

    mutex_request.lock();
    request_received.reset(new ReceivedMessage(recv_msg));
    mutex_request.unlock();

    // notify waiting request methods
    cv_request.notify_one();
    // notify waiting respond methods
    cv_ack.notify_one();

    // check subscriptions
    mutex_callbacks.lock();
    if(request_received->status==OK && subscriptions.count(ID(request_received->id))) {
        // fetch message type, decode payload
        msp::Message *const req = subscribed_requests.at(ID(request_received->id));
        req->decode(request_received->data);
        // call callback
        subscriptions.at(ID(request_received->id))->call(*req);
    }
    mutex_callbacks.unlock();
}

ReceivedMessage Client::processOneMessageV1() {
    ReceivedMessage ret;
    
    ret.status = OK;
    
    // message direction
    const uint8_t dir = read();
    const bool ok_id = (dir!='!');
    if (!ok_id) std::cout << "id not recognized by FC" << std::endl; 
    
    // payload length
    uint8_t len;
    asio::read(pimpl->port, asio::buffer(&len,1));
    request_received.data.resize(len);

    // message ID
    ret.id = read();

    if(print_warnings && !ok_id) {
        std::cerr << "Message v1 with ID " << size_t(ret.id) << " is not recognised!" << std::endl;
    }
    
    // payload
    for(size_t i(0); i<len; i++) {
        ret.data.push_back(read());
    }

    // CRC
    const uint8_t rcv_crc = read();
    const uint8_t exp_crc = crcV1(ret.id,ret.data);
    const bool ok_crc = (rcv_crc==exp_crc);

    if(print_warnings && !ok_crc) {
        std::cerr << "Message v1 with ID " << size_t(ret.id) << " has wrong CRC! (expected: " << size_t(exp_crc) << ", received: "<< size_t(rcv_crc) << ")" << std::endl;
    }

    if(!ok_id) { ret.status = FAIL_ID; }
    else if(!ok_crc) { ret.status = FAIL_CRC; }
    
    if (ret.status != OK) std::cout << "v1 ret status " << ret.status << std::endl; 
    
    return ret;
}

ReceivedMessage Client::processOneMessageV2() {
    ReceivedMessage ret;
    
    ret.status = OK;

    uint8_t exp_crc = 0;
    
    // message direction
    const uint8_t dir = read();
    const bool ok_id = (dir!='!');
    
    // flag
    const uint8_t flag = read();
    exp_crc = crcV2(exp_crc, flag);
    
    // message ID
    const uint8_t id_low = read();
    const uint8_t id_high = read();
    ret.id = uint32_t(id_low) | (uint32_t(id_high) << 8);
    exp_crc = crcV2(exp_crc, id_low);
    exp_crc = crcV2(exp_crc, id_high);
    
    // payload length
    const uint8_t len_low = read();
    const uint8_t len_high = read();
    uint32_t len = uint32_t(len_low) | (uint32_t(len_high) << 8);
    exp_crc = crcV2(exp_crc, len_low);
    exp_crc = crcV2(exp_crc, len_high);

    if(print_warnings && !ok_id) {
        std::cerr << "Message v2 with ID " << size_t(ret.id) << " is not recognised!" << std::endl;
    }
    
    // payload
    ByteVector data;
    for(size_t i(0); i<len; i++) {
        ret.data.push_back(read());
    }
    
    exp_crc = crcV2(exp_crc,ret.data);

    // CRC
    const uint8_t rcv_crc = read();
    
    const bool ok_crc = (rcv_crc==exp_crc);

    if(print_warnings && !ok_crc) {
        std::cerr << "Message v2 with ID " << size_t(ret.id) << " has wrong CRC! (expected: " << size_t(exp_crc) << ", received: "<< size_t(rcv_crc) << ")" << std::endl;
    }

    if(!ok_id) { ret.status = FAIL_ID; }
    else if(!ok_crc) { ret.status = FAIL_CRC; }
    
    return ret;
    
}


bool Client::sendDataV1(const uint8_t id, const ByteVector &data) {
    std::lock_guard<std::mutex> lock(mutex_send);
    ByteVector msg;
    msg.push_back('$');                                 // preamble1
    msg.push_back('M');                                 // preamble2
    msg.push_back('<');                                 // direction
    msg.push_back(uint8_t(data.size()));                // data size
    msg.push_back(id);                                  // message_id
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crcV1(id, data) );                     // crc
    
    
    asio::error_code ec;
    const std::size_t bytes_written = asio::write(pimpl->port, asio::buffer(msg.data(), msg.size()), ec);
    if (ec == asio::error::operation_aborted) {
        //operation_aborted error probably means the client is being closed
        return false;
    }

    return (bytes_written==msg.size());
}

uint8_t Client::crcV1(const uint8_t id, const ByteVector &data) {
    uint8_t crc = uint8_t(data.size())^id;
    for(const uint8_t d : data) { crc = crc^d; }
    return crc;
}

bool Client::sendDataV2(const uint16_t id, const ByteVector &data) {
    std::lock_guard<std::mutex> lock(mutex_send);
    ByteVector msg;
    msg.push_back('$');                                 // preamble1
    msg.push_back('X');                                 // preamble2
    msg.push_back('<');                                 // direction
    msg.push_back(0);                                   // flag
    
    msg.push_back(uint8_t( id & 0xFF ));                // message_id low bits
    msg.push_back(uint8_t( id >> 8 ));                  // message_id high bits
    
    uint16_t size = (uint16_t)data.size();
    msg.push_back(uint8_t( size & 0xFF ));              // data size low bits
    msg.push_back(uint8_t( size >> 8 ));                // data size high bits
    
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crcV2( 0, ByteVector(msg.begin()+3,msg.end()) ) );                     // crc

    asio::error_code ec;
    const std::size_t bytes_written = asio::write(pimpl->port, asio::buffer(msg.data(), msg.size()), ec);
    if (ec == asio::error::operation_aborted) {
        //operation_aborted error probably means the client is being closed
        return false;
    }
    
    return (bytes_written==msg.size());
}

uint8_t Client::crcV2(uint8_t crc, const ByteVector &data) {
    //std::cout << "crc array len: " << data.size() << std::endl;
    for (const uint8_t& p : data) {
        crc = crcV2(crc,p);
    }
    return crc;
}


uint8_t Client::crcV2(uint8_t crc, const uint8_t& b) {
    crc ^= b;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}



} // namespace client
} // namespace msp
