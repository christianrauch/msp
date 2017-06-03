#include <Client.hpp>

#include <iostream>

namespace msp {
namespace client {

Client::Client() : port(io), running(false), print_warnings(false) { }

Client::~Client() {
    for(const std::pair<msp::ID, msp::Request*> d : subscribed_requests)
        delete d.second;

    for(const std::pair<msp::ID, SubscriptionBase*> s : subscriptions)
        delete s.second;
}

void Client::connect(const std::string &device, const uint baudrate) {
    port.open(device);

    port.set_option(asio::serial_port::baud_rate(baudrate));
    port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
}

void Client::waitForOneMessage() {
    // register handler for incomming messages
    mutex_buffer.lock();
    asio::async_read_until(port, buffer, "$M", std::bind(&Client::onHeaderStart, this, std::placeholders::_1, std::placeholders::_2));
    // wait for incomming data
    io.run();
    io.reset();
    mutex_buffer.unlock();
}

void Client::start() {
    thread = std::thread([this]{
        running = true;
        while(running) { waitForOneMessage(); }
    });
}

void Client::stop() {
    running = false;
    io.stop();
    thread.join();
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

    const std::size_t bytes_written = asio::write(port, asio::buffer(msg.data(), msg.size()));

    return (bytes_written==msg.size());
}

bool Client::request(msp::Request &request) {
    msp::ByteVector data;
    const bool success = request_raw(uint8_t(request.id()), data);
    if(success) { request.decode(data); }
    return success;
}

bool Client::request_raw(const uint8_t id, ByteVector &data) {
    // send request
    if(!sendRequest(id)) { return false; }

    // wait for thread to received message
    std::unique_lock<std::mutex> lock(mutex_cv_request);
    cv_request.wait(lock, [&]{
        mutex_request.lock();
        const bool received = (request_received!=NULL) && (request_received->id==id);
        // unlock to wait for next message
        if(!received) { mutex_request.unlock(); }
        return received;
    });

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

void Client::onHeaderStart(const asio::error_code& error, const std::size_t bytes_transferred) {
    if(error) { return; }

    // ignore and remove header bytes
    buffer.consume(bytes_transferred);

    MessageStatus status = OK;

    // message direction
    const uint8_t dir = uint8_t(buffer.sbumpc());
    if(dir=='!') { status = FAIL_ID; }

    // payload length
    const uint8_t len = uint8_t(buffer.sbumpc());

    // message ID
    const uint8_t id = uint8_t(buffer.sbumpc());

    // payload
    std::vector<uint8_t> data;
    for(uint i(0); i<len; i++) { data.push_back(uint8_t(buffer.sbumpc())); }

    // CRC
    const uint8_t rcv_crc = uint8_t(buffer.sbumpc());
    if(rcv_crc!=crc(id,data)) { status = FAIL_CRC; }

    if(print_warnings) {
        switch (status) {
        case OK: break;
        case FAIL_ID:
            std::cerr << "Message with ID " << uint(id) << " is not recognised!" << std::endl;
            break;
        case FAIL_CRC:
            std::cerr << "Message with ID " << uint(id) << " has wrong CRC!" << std::endl;
            break;
        }
    }

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
