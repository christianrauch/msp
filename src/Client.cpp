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

Client::Client() : pimpl(new SerialPortImpl), running(false), print_warnings(false) {
    request_received.data.reserve(256);
}

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

bool Client::sendData(const uint8_t id, const ByteVector &data) {
    std::lock_guard<std::mutex> lock(mutex_send);

    try {
        asio::write(pimpl->port, asio::buffer("$M<",3));              // header
        asio::write(pimpl->port, asio::buffer({uint8_t(data.size())})); // data size
        asio::write(pimpl->port, asio::buffer({uint8_t(id)}));          // message id
        asio::write(pimpl->port, asio::buffer(data));                   // data
        asio::write(pimpl->port, asio::buffer({crc(id, data)}));        // crc
    } catch (const asio::system_error &ec) {
        if (ec.code() == asio::error::operation_aborted) {
            //operation_aborted error probably means the client is being closed
            return false;
        }
    }

    return true;
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

uint8_t Client::crc(const uint8_t id, const ByteVector &data) {
    uint8_t crc = uint8_t(data.size())^id;
    for(const uint8_t d : data) { crc = crc^d; }
    return crc;
}

void Client::processOneMessage() {
    uint8_t c;
    // find start of header
    while(true) {
        // find '$'
        for(c=0; c!='$'; asio::read(pimpl->port, asio::buffer(&c,1)));
        // check 'M'
        asio::read(pimpl->port, asio::buffer(&c,1));
        if(c=='M') { break; }
    }

    // message direction
    asio::read(pimpl->port, asio::buffer(&c,1));
    const bool ok_id = (c!='!');

    // payload length
    uint8_t len;
    asio::read(pimpl->port, asio::buffer(&len,1));
    request_received.data.resize(len);

    // message ID
    mutex_request.lock();
    asio::read(pimpl->port, asio::buffer(&request_received.id,1));
    mutex_request.unlock();

    if(print_warnings && !ok_id) {
        std::cerr << "Message with ID " << size_t(request_received.id) << " is not recognised!" << std::endl;
    }

    // payload
    asio::read(pimpl->port, asio::buffer(request_received.data));

    // CRC
    uint8_t rcv_crc;
    asio::read(pimpl->port, asio::buffer(&rcv_crc,1));
    mutex_request.lock();
    const uint8_t exp_crc = crc(request_received.id, request_received.data);
    mutex_request.unlock();
    const bool ok_crc = (rcv_crc==exp_crc);

    if(print_warnings && !ok_crc) {
        std::cerr << "Message with ID " << size_t(request_received.id) << " has wrong CRC! (expected: " << size_t(exp_crc) << ", received: "<< size_t(rcv_crc) << ")" << std::endl;
    }

    mutex_request.lock();
    request_received.status = !ok_id ? FAIL_ID : (!ok_crc ? FAIL_CRC : OK) ;
    mutex_request.unlock();

    // notify waiting request methods
    cv_request.notify_one();
    // notify waiting respond methods
    cv_ack.notify_one();

    // check subscriptions
    mutex_callbacks.lock();
    mutex_request.lock();
    if(request_received.status==OK && subscriptions.count(ID(request_received.id))) {
        // fetch message type, decode payload
        msp::Request *const req = subscribed_requests.at(ID(request_received.id));
        req->decode(request_received.data);
        mutex_request.unlock();
        // call callback
        subscriptions.at(ID(request_received.id))->call(*req);
    }
    mutex_request.unlock();
    mutex_callbacks.unlock();
}

} // namespace client
} // namespace msp
