#include <Client.hpp>
#include <cstdlib>
#include <iostream>

typedef unsigned int uint;

namespace msp {
namespace client {

Client::Client() :
    port(io),
    log_level_(SILENT),
    msp_ver_(1),
    fw_variant(FirmwareVariant::INAV) {}

Client::~Client() {}

void Client::setLoggingLevel(const LoggingLevel& level) { log_level_ = level; }

bool Client::setVersion(const int& ver) {
    if(ver == 1 || ver == 2) {
        msp_ver_ = ver;
        return true;
    }
    return false;
}

int Client::getVersion() const { return msp_ver_; }

void Client::setVariant(const FirmwareVariant& v) { fw_variant = v; }

FirmwareVariant Client::getVariant() const { return fw_variant; }

bool Client::start(const std::string& device, const size_t baudrate) {
    return connectPort(device, baudrate) && startReadThread() &&
           startSubscriptions();
}

bool Client::stop() {
    return disconnectPort() && stopReadThread() && stopSubscriptions();
}

bool Client::connectPort(const std::string& device, const size_t baudrate) {
    try {
        port.open(device);
        port.set_option(asio::serial_port::baud_rate(uint(baudrate)));
        port.set_option(
            asio::serial_port::parity(asio::serial_port::parity::none));
        port.set_option(asio::serial_port::character_size(
            asio::serial_port::character_size(8)));
        port.set_option(
            asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
    }
    catch(const std::system_error& e) {
        const int ecode = e.code().value();
        throw std::runtime_error("Error when opening '" + device +
                                 "': " + e.code().category().message(ecode) +
                                 " (error code: " + std::to_string(ecode) +
                                 ")");
    }
    return isConnected();
}

bool Client::disconnectPort() {
    asio::error_code ec;
    port.close(ec);
    if(ec) return false;
    return true;
}

bool Client::isConnected() const { return port.is_open(); }

bool Client::startReadThread() {
    // no point reading if we arent connected to anything
    if(!isConnected()) return false;
    // can't start if we are already running
    if(running_.test_and_set()) return false;
    // hit it!
    thread = std::thread([this] {
        asio::async_read_until(port,
                               buffer,
                               std::bind(&Client::messageReady,
                                         this,
                                         std::placeholders::_1,
                                         std::placeholders::_2),
                               std::bind(&Client::processOneMessage,
                                         this,
                                         std::placeholders::_1,
                                         std::placeholders::_2));
        io.run();
    });
    return true;
}

bool Client::stopReadThread() {
    bool rc = false;
    if(running_.test_and_set()) {
        io.stop();
        thread.join();
        io.reset();
        rc = true;
    }
    running_.clear();
    return rc;
}

bool Client::startSubscriptions() {
    bool rc = true;
    for(const auto& sub : subscriptions) {
        rc &= sub.second->start();
    }
    return rc;
}

bool Client::stopSubscriptions() {
    bool rc = true;
    for(const auto& sub : subscriptions) {
        rc &= sub.second->stop();
    }
    return rc;
}

bool Client::sendMessage(msp::Message& message, const double& timeout) {
    if(log_level_ >= DEBUG)
        std::cout << "sending message - ID " << size_t(message.id())
                  << std::endl;
    if(!sendData(message.id(), message.encode())) {
        if(log_level_ >= WARNING)
            std::cerr << "message failed to send" << std::endl;
        return false;
    }
    // prepare the condition check
    std::unique_lock<std::mutex> lock(cv_response_mtx);
    const auto predicate = [&] {
        mutex_response.lock();
        const bool received = (request_received != nullptr) &&
                              (request_received->id == message.id());
        // unlock to wait for next message
        if(!received) {
            mutex_response.unlock();
        }
        return received;
    };
    // depending on the timeout, we may wait a fixed amount of time, or
    // indefinitely
    if(timeout > 0) {
        if(!cv_response.wait_for(
               lock,
               std::chrono::milliseconds(size_t(timeout * 1e3)),
               predicate)) {
            if(log_level_ >= INFO)
                std::cout << "timed out waiting for response to message ID "
                          << size_t(message.id()) << std::endl;
            return false;
        }
    }
    else {
        cv_response.wait(lock, predicate);
    }
    // check status
    const bool recv_success = request_received->status == OK;
    ByteVector data;
    if(recv_success) {
        // make local copy of the data so that the read thread can keep moving
        data = request_received->payload;
    }
    mutex_response.unlock();
    // decode the local copy of the payload
    const bool decode_success = data.size() == 0 ? true : message.decode(data);
    return recv_success && decode_success;
}

bool Client::sendMessageNoWait(const msp::Message& message) {
    if(log_level_ >= DEBUG)
        std::cout << "async sending message - ID " << size_t(message.id())
                  << std::endl;
    if(!sendData(message.id(), message.encode())) {
        if(log_level_ >= WARNING)
            std::cerr << "async sendData failed" << std::endl;
        return false;
    }
    return true;
}

uint8_t Client::extractChar() {
    if(buffer.sgetc() == EOF) {
        if(log_level_ >= WARNING)
            std::cerr << "buffer returned EOF; reading char directly from port"
                      << std::endl;
        asio::read(port, buffer, asio::transfer_exactly(1));
    }
    return uint8_t(buffer.sbumpc());
}

bool Client::sendData(const msp::ID id, const ByteVector& data) {
    if(log_level_ >= DEBUG)
        std::cout << "sending: " << size_t(id) << " | " << data;
    ByteVector msg;
    if(msp_ver_ == 2) {
        msg = packMessageV2(id, data);
    }
    else {
        msg = packMessageV1(id, data);
    }
    if(log_level_ >= DEBUG) std::cout << "packed: " << msg;
    asio::error_code ec;
    std::size_t bytes_written;
    {
        std::lock_guard<std::mutex> lock(mutex_send);
        bytes_written =
            asio::write(port, asio::buffer(msg.data(), msg.size()), ec);
    }
    if(ec == asio::error::operation_aborted && log_level_ >= WARNING) {
        // operation_aborted error probably means the client is being closed
        std::cerr << "------------------> WRITE FAILED <--------------------"
                  << std::endl;
        return false;
    }
    if(log_level_ >= DEBUG)
        std::cout << "write complete: " << bytes_written << " vs " << msg.size()
                  << std::endl;
    return (bytes_written == msg.size());
}

ByteVector Client::packMessageV1(const msp::ID id,
                                 const ByteVector& data) const {
    ByteVector msg;
    msg.push_back('$');                               // preamble1
    msg.push_back('M');                               // preamble2
    msg.push_back('<');                               // direction
    msg.push_back(uint8_t(data.size()));              // data size
    msg.push_back(uint8_t(id));                       // message_id
    msg.insert(msg.end(), data.begin(), data.end());  // data
    msg.push_back(crcV1(uint8_t(id), data));          // crc
    return msg;
}

uint8_t Client::crcV1(const uint8_t id, const ByteVector& data) const {
    uint8_t crc = uint8_t(data.size()) ^ id;
    for(const uint8_t d : data) {
        crc = crc ^ d;
    }
    return crc;
}

ByteVector Client::packMessageV2(const msp::ID id,
                                 const ByteVector& data) const {
    ByteVector msg;
    msg.push_back('$');                           // preamble1
    msg.push_back('X');                           // preamble2
    msg.push_back('<');                           // direction
    msg.push_back(0);                             // flag
    msg.push_back(uint8_t(uint16_t(id) & 0xFF));  // message_id low bits
    msg.push_back(uint8_t(uint16_t(id) >> 8));    // message_id high bits

    const uint16_t size = uint16_t(data.size());
    msg.push_back(uint8_t(size & 0xFF));  // data size low bits
    msg.push_back(uint8_t(size >> 8));    // data size high bits

    msg.insert(msg.end(), data.begin(), data.end());                  // data
    msg.push_back(crcV2(0, ByteVector(msg.begin() + 3, msg.end())));  // crc

    return msg;
}

uint8_t Client::crcV2(uint8_t crc, const ByteVector& data) const {
    for(const uint8_t& p : data) {
        crc = crcV2(crc, p);
    }
    return crc;
}

uint8_t Client::crcV2(uint8_t crc, const uint8_t& b) const {
    crc ^= b;
    for(int ii = 0; ii < 8; ++ii) {
        if(crc & 0x80) {
            crc = uint8_t(crc << 1) ^ 0xD5;
        }
        else {
            crc = uint8_t(crc << 1);
        }
    }
    return crc;
}

void Client::processOneMessage(const asio::error_code& ec,
                               const std::size_t& bytes_transferred) {
    if(log_level_ >= DEBUG)
        std::cout << "processOneMessage on " << bytes_transferred << " bytes"
                  << std::endl;

    if(ec == asio::error::operation_aborted) {
        // operation_aborted error probably means the client is being closed
        // notify waiting request methods
        cv_response.notify_all();
        return;
    }

    // ignore and remove header bytes
    const uint8_t msg_marker = extractChar();
    if(msg_marker != '$')
        std::cerr << "Message marker " << size_t(msg_marker)
                  << " is not recognised!" << std::endl;

    // message version
    int ver                  = 0;
    const uint8_t ver_marker = extractChar();
    if(ver_marker == 'M') ver = 1;
    if(ver_marker == 'X') ver = 2;
    if(ver == 0) {
        std::cerr << "Version marker " << size_t(ver_marker)
                  << " is not recognised!" << std::endl;
    }

    ReceivedMessage recv_msg;
    if(ver == 2)
        recv_msg = processOneMessageV2();
    else
        recv_msg = processOneMessageV1();

    {
        std::lock_guard<std::mutex> lock2(cv_response_mtx);
        std::lock_guard<std::mutex> lock(mutex_response);
        request_received.reset(new ReceivedMessage(recv_msg));
    }
    // notify waiting request methods
    cv_response.notify_all();

    // check subscriptions
    {
        std::lock_guard<std::mutex> lock(mutex_subscriptions);
        std::lock_guard<std::mutex> lock2(mutex_response);
        if(request_received->status == OK &&
           subscriptions.count(ID(request_received->id))) {
            subscriptions.at(ID(request_received->id))
                ->decode(request_received->payload);
        }
    }

    asio::async_read_until(port,
                           buffer,
                           std::bind(&Client::messageReady,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2),
                           std::bind(&Client::processOneMessage,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2));

    if(log_level_ >= DEBUG)
        std::cout << "processOneMessage finished" << std::endl;
}

std::pair<iterator, bool> Client::messageReady(iterator begin,
                                               iterator end) const {
    iterator i             = begin;
    const size_t available = size_t(std::distance(begin, end));

    if(available < 2) return std::make_pair(begin, false);

    if(*i == '$' && *(i + 1) == 'M') {
        // not even enough data for a header
        if(available < 6) return std::make_pair(begin, false);

        const uint8_t payload_size = uint8_t(*(i + 3));
        // incomplete xfer
        if(available < size_t(5 + payload_size + 1))
            return std::make_pair(begin, false);

        std::advance(i, 5 + payload_size + 1);
    }
    else if(*i == '$' && *(i + 1) == 'X') {
        // not even enough data for a header
        if(available < 9) return std::make_pair(begin, false);

        const uint16_t payload_size =
            uint8_t(*(i + 6)) | uint8_t(*(i + 7) << 8);

        // incomplete xfer
        if(available < size_t(8 + payload_size + 1))
            return std::make_pair(begin, false);

        std::advance(i, 8 + payload_size + 1);
    }
    else {
        for(; i != end; ++i) {
            if(*i == '$') break;
        }
        // implicitly consume all if $ not found
    }

    return std::make_pair(i, true);
}

ReceivedMessage Client::processOneMessageV1() {
    ReceivedMessage ret;

    ret.status = OK;

    // message direction
    const uint8_t dir = extractChar();
    const bool ok_id  = (dir != '!');

    // payload length
    const uint8_t len = extractChar();

    // message ID
    uint8_t id = extractChar();
    ret.id     = msp::ID(id);

    if(log_level_ >= WARNING && !ok_id) {
        std::cerr << "Message v1 with ID " << size_t(ret.id)
                  << " is not recognised!" << std::endl;
    }

    // payload
    for(size_t i(0); i < len; i++) {
        ret.payload.push_back(extractChar());
    }

    // CRC
    const uint8_t rcv_crc = extractChar();
    const uint8_t exp_crc = crcV1(id, ret.payload);
    const bool ok_crc     = (rcv_crc == exp_crc);

    if(log_level_ >= WARNING && !ok_crc) {
        std::cerr << "Message v1 with ID " << size_t(ret.id)
                  << " has wrong CRC! (expected: " << size_t(exp_crc)
                  << ", received: " << size_t(rcv_crc) << ")" << std::endl;
    }

    if(!ok_id) {
        ret.status = FAIL_ID;
    }
    else if(!ok_crc) {
        ret.status = FAIL_CRC;
    }

    return ret;
}

ReceivedMessage Client::processOneMessageV2() {
    ReceivedMessage ret;

    ret.status = OK;

    uint8_t exp_crc = 0;

    // message direction
    const uint8_t dir = extractChar();
    if(log_level_ >= DEBUG) std::cout << "dir: " << size_t(dir) << std::endl;
    const bool ok_id = (dir != '!');

    // flag
    const uint8_t flag = extractChar();
    if(log_level_ >= DEBUG) std::cout << "flag: " << size_t(flag) << std::endl;
    exp_crc = crcV2(exp_crc, flag);

    // message ID
    const uint8_t id_low  = extractChar();
    const uint8_t id_high = extractChar();
    uint16_t id           = uint16_t(id_low) | uint16_t(id_high << 8);
    ret.id                = msp::ID(id);
    if(log_level_ >= DEBUG) std::cout << "id: " << size_t(id) << std::endl;
    exp_crc = crcV2(exp_crc, id_low);
    exp_crc = crcV2(exp_crc, id_high);

    // payload length
    const uint8_t len_low  = extractChar();
    const uint8_t len_high = extractChar();
    uint32_t len           = uint32_t(len_low) | (uint32_t(len_high) << 8);
    exp_crc                = crcV2(exp_crc, len_low);
    exp_crc                = crcV2(exp_crc, len_high);
    if(log_level_ >= DEBUG) std::cout << "len: " << len << std::endl;

    if(log_level_ >= WARNING && !ok_id) {
        std::cerr << "Message v2 with ID " << size_t(ret.id)
                  << " is not recognised!" << std::endl;
    }

    // payload
    ByteVector data;
    for(size_t i(0); i < len; i++) {
        ret.payload.push_back(extractChar());
    }

    exp_crc = crcV2(exp_crc, ret.payload);

    // CRC
    const uint8_t rcv_crc = extractChar();

    const bool ok_crc = (rcv_crc == exp_crc);

    if(log_level_ >= WARNING && !ok_crc) {
        std::cerr << "Message v2 with ID " << size_t(ret.id)
                  << " has wrong CRC! (expected: " << size_t(exp_crc)
                  << ", received: " << size_t(rcv_crc) << ")" << std::endl;
    }

    if(!ok_id) {
        ret.status = FAIL_ID;
    }
    else if(!ok_crc) {
        ret.status = FAIL_CRC;
    }

    return ret;
}

}  // namespace client
}  // namespace msp
