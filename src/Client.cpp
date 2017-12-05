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

Client::Client() : pimpl(new SerialPortImpl), running(false), print_warnings(false) { }

Client::~Client() {
    for(const std::pair<msp::ID, msp::Request*> d : subscribed_requests)
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
    if(pimpl->buffer.sgetc()==EOF) {
        asio::read(pimpl->port, pimpl->buffer, asio::transfer_exactly(1));
    }

    return uint8_t(pimpl->buffer.sbumpc());
}

bool Client::sendData(const uint8_t id, const ByteVector &data) {
    std::lock_guard<std::mutex> lock(mutex_send);
    ByteVector msg;
    msg.push_back('$');                                 // preamble1
    msg.push_back('M');                                 // preamble2
    msg.push_back('<');                                 // direction
    msg.push_back(uint8_t(data.size()));                // data size
    msg.push_back(id);                                  // message_id
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crc(id, data) );                     // crc

    asio::error_code ec;
    const std::size_t bytes_written = asio::write(pimpl->port, asio::buffer(msg.data(), msg.size()), ec);
    if (ec == asio::error::operation_aborted) {
        //operation_aborted error probably means the client is being closed
        return false;
    }

    return (bytes_written==msg.size());
}

int Client::request(msp::Request &request, const double timeout) {
    msp::ByteVector data;
    const int success = request_raw(uint8_t(request.id()), data, timeout);
    if(success==1) { request.decode(data); }
    return success;
}

int Client::request_raw(const uint8_t id, ByteVector &data, const double timeout) {
    // send request
    if(!sendRequest(id)) { return false; }

    // wait for thread to received message
    std::unique_lock<std::mutex> lock(mutex_cv_request);
    const auto predicate = [&]{
        mutex_request.lock();
        const bool received = (request_received!=NULL) && (request_received->id==id);
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
    const bool success = request_received->status==OK;
    if(success) { data = request_received->data; }
    mutex_request.unlock();
    return success;
}

bool Client::respond(const msp::Response &response, const bool wait_ack) {
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
        const bool received = (request_received!=NULL) && (request_received->id==id);
        // unlock to wait for next message
        if(!received) { mutex_request.unlock(); }
        return received;
    });

    // check status, expect ACK without payload
    const bool success = request_received->status==OK;
    mutex_request.unlock();
    return success;
}

uint8_t Client::crc(const uint8_t id, const ByteVector &data) {
    uint8_t crc = uint8_t(data.size())^id;
    for(const uint8_t d : data) { crc = crc^d; }
    return crc;
}

void Client::processOneMessage() {
    std::lock_guard<std::mutex> lck(mutex_buffer);
    asio::error_code ec;
    const std::size_t bytes_transferred = asio::read_until(pimpl->port, pimpl->buffer, "$M", ec);
    if (ec == asio::error::operation_aborted) {
        //operation_aborted error probably means the client is being closed
        return;
    }
    // ignore and remove header bytes
    pimpl->buffer.consume(bytes_transferred);

    MessageStatus status = OK;

    // message direction
    const uint8_t dir = read();
    const bool ok_id = (dir!='!');

    // payload length
    const uint8_t len = read();

    // message ID
    const uint8_t id = read();

    if(print_warnings && !ok_id) {
        std::cerr << "Message with ID " << size_t(id) << " is not recognised!" << std::endl;
    }

    // payload
    std::vector<uint8_t> data;
    for(size_t i(0); i<len; i++) {
        data.push_back(read());
    }

    // CRC
    const uint8_t rcv_crc = read();
    const uint8_t exp_crc = crc(id,data);
    const bool ok_crc = (rcv_crc==exp_crc);

    if(print_warnings && !ok_crc) {
        std::cerr << "Message with ID " << size_t(id) << " has wrong CRC! (expected: " << size_t(exp_crc) << ", received: "<< size_t(rcv_crc) << ")" << std::endl;
    }

    if(!ok_id) { status = FAIL_ID; }
    else if(!ok_crc) { status = FAIL_CRC; }

    mutex_request.lock();
    request_received.reset(new ReceivedMessage());
    request_received->id = id;
    request_received->data = data;
    request_received->status = status;
    mutex_request.unlock();

    // notify waiting request methods
    cv_request.notify_one();
    // notify waiting respond methods
    cv_ack.notify_one();

    // check subscriptions
    mutex_callbacks.lock();
    if(status==OK && subscriptions.count(ID(id))) {
        // fetch message type, decode payload
        msp::Request *const req = subscribed_requests.at(ID(id));
        req->decode(data);
        // call callback
        subscriptions.at(ID(id))->call(*req);
    }
    mutex_callbacks.unlock();
}

} // namespace client
} // namespace msp
