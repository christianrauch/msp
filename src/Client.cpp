#include <Client.hpp>
//#include "SerialPortImpl.cpp"

#include <iostream>
#include <cstdlib>


namespace msp {
namespace client {

Client::Client(const std::string &device, const size_t baudrate) : 
    device_name_(device), baud_rate_(baudrate), port(io), running_(ATOMIC_FLAG_INIT), print_warnings_(false), 
    print_debug_(false), msp_ver_(1), fw_variant(FirmwareVariant::INAV)
{ }

Client::~Client() 
{ }

void Client::setDevice(const std::string& device)
{
    device_name_ = device;
}

std::string Client::getDevice()
{
    return device_name_;
}

void Client::setBaudRate(const size_t& baud)
{
    baud_rate_= baud;
}

size_t Client::getBaudRate()
{
    return baud_rate_;
}
    
    
void Client::printWarnings(const bool& val)
{
    print_warnings_ = val;
}

void Client::printDebug(const bool& val)
{
    print_debug_ = val;
}

bool Client::setVersion(int ver) 
{
    if (ver == 1 || ver == 2) {
        msp_ver_ = ver;
        return true;
    }
    return false;
}

int Client::getVersion()
{
    return msp_ver_;
}
    

void Client::setVariant(FirmwareVariant v) 
{
    fw_variant = v;
}

FirmwareVariant Client::getVariant() 
{
    return fw_variant;
}


bool Client::connect() 
{
    port.open(device_name_);
    port.set_option(asio::serial_port::baud_rate(baud_rate_));
    port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
    return start();
    
}

bool Client::disconnect()
{
    port.close();
    return stop();
}


bool Client::start() 
{
    if (!port.is_open()) return false;
    if (running_.test_and_set()) return false;
    thread = std::thread([this]{
        asio::async_read_until(port, buffer, 
            std::bind(&Client::messageReady,this,std::placeholders::_1,std::placeholders::_2),
            std::bind(&Client::processOneMessage,this,std::placeholders::_1,std::placeholders::_2));
        io.run();
        
    });
    return true;
}

bool Client::stop() 
{
    bool rc = false;
    if (running_.test_and_set()) {
        for (const auto& sub : subscriptions) {
            sub.second->stop();
        }
        
        io.stop();
        thread.join();
        io.reset();
        rc = true;
    }
    running_.clear();
    return rc;
}


bool Client::sendMessage(msp::Message& message, const double timeout)
{
    if (print_debug_) std::cout << "sending message - ID " << (uint32_t)message.id() << std::endl;
    if(!sendData(message.id(), message.encode())) { 
        std::cout << "sendData failed" << std::endl;
        return false; 
    }

    // wait for thread to received message
    std::unique_lock<std::mutex> lock(cv_response_mtx);

    
    const auto predicate = [&]{
        mutex_response.lock();
        const bool received = (request_received!=NULL) && (request_received->id==(uint32_t)message.id());
        // unlock to wait for next message
        if(!received) { mutex_response.unlock(); }
        return received;
    };

    if(timeout>0) {
        //std::cout << "waiting for response with timeout" << std::endl;
        if(!cv_response.wait_for(lock, std::chrono::milliseconds(size_t(timeout*1e3)), predicate))
            return false;
        //std::cout << "response received" << std::endl;
    }
    else {
        cv_response.wait(lock, predicate);
    }

    // check status
    const bool success = request_received->status==OK;
    ByteVector data;
    if(success) { data = request_received->data; }
    mutex_response.unlock();
    message.decode(data);
    
    if (!success) std::cout << "request status not OK" << std::endl; 
    return success;
    
}


bool Client::asyncSendMessage(msp::Message& message)
{
    if (print_debug_) std::cout << "async sending message - ID " << (uint32_t)message.id() << std::endl;
    if(!sendData(message.id(), message.encode())) { 
        std::cout << "async sendData failed" << std::endl;
        return false; 
    }
    return true;
}


uint8_t Client::extractChar() {
    if(buffer.sgetc()==EOF) {
        std::cout << "buffer returned EOF; reading char directly from port" << std::endl;
        asio::read(port, buffer, asio::transfer_exactly(1));
    }
    uint8_t rc = uint8_t(buffer.sbumpc());
    return rc;
}


bool Client::sendData(const msp::ID id, const ByteVector &data) 
{
    if (print_debug_) std::cout << "sending: " << uint32_t(id) << " | " << data;
    ByteVector msg;
    if (msp_ver_ == 2) {
        msg = packMessageV2(id,data);
    } else {
        msg = packMessageV1(id,data);
    }
    if (print_debug_) std::cout << "packed: " << msg;
    asio::error_code ec;
    std::size_t bytes_written;
    {
        std::lock_guard<std::mutex> lock(mutex_send);
        bytes_written = asio::write(port, asio::buffer(msg.data(), msg.size()), ec);
    }
    if (ec == asio::error::operation_aborted) {
        //operation_aborted error probably means the client is being closed
        std::cout << "------------------> WRITE FAILED <--------------------" <<std::endl;
        return false;
    }
    if (print_debug_) std::cout << "write complete: " << bytes_written << " vs " << msg.size() << std::endl;
    return (bytes_written==msg.size());
    
}


//TODO: if id is > 254, pack using V2 over V1 protocol
ByteVector Client::packMessageV1(const msp::ID id, const ByteVector &data) 
{
    
    ByteVector msg;
    msg.push_back('$');                                 // preamble1
    msg.push_back('M');                                 // preamble2
    msg.push_back('<');                                 // direction
    msg.push_back(uint8_t(data.size()));                // data size
    msg.push_back(uint8_t(id));                                  // message_id
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crcV1(uint8_t(id), data) );                     // crc
    return msg;
}

uint8_t Client::crcV1(const uint8_t id, const ByteVector &data) {
    uint8_t crc = uint8_t(data.size())^id;
    for(const uint8_t d : data) { crc = crc^d; }
    return crc;
}

ByteVector Client::packMessageV2(const msp::ID id, const ByteVector &data) 
{
    
    ByteVector msg;
    msg.push_back('$');                                 // preamble1
    msg.push_back('X');                                 // preamble2
    msg.push_back('<');                                 // direction
    msg.push_back(0);                                   // flag
    msg.push_back(uint8_t( uint16_t(id) & 0xFF ));                // message_id low bits
    msg.push_back(uint8_t( uint16_t(id) >> 8 ));                  // message_id high bits
    
    uint16_t size = (uint16_t)data.size();
    msg.push_back(uint8_t( size & 0xFF ));              // data size low bits
    msg.push_back(uint8_t( size >> 8 ));                // data size high bits
    
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crcV2( 0, ByteVector(msg.begin()+3,msg.end()) ) );                     // crc
    
    return msg;

}

uint8_t Client::crcV2(uint8_t crc, const ByteVector &data) 
{
    for (const uint8_t& p : data) {
        crc = crcV2(crc,p);
    }
    return crc;
}


uint8_t Client::crcV2(uint8_t crc, const uint8_t& b) 
{
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



void Client::processOneMessage(const asio::error_code& ec,std::size_t bytes_transferred) {
    if (print_debug_) std::cout << "processOneMessage on " << bytes_transferred << " bytes" << std::endl;

    if (ec == asio::error::operation_aborted) {
        //operation_aborted error probably means the client is being closed
        // notify waiting request methods
        cv_response.notify_all();
        return;
    } 
    
    // ignore and remove header bytes
    //buffer.consume(bytes_transferred);
    const uint8_t msg_marker = extractChar();
    if (msg_marker != '$') std::cerr << "Message marker " << msg_marker << " is not recognised!" << std::endl;
    
    // message version
    int ver = 0;
    const uint8_t ver_marker = extractChar();
    if (ver_marker == 'M') ver = 1;
    if (ver_marker == 'X') ver = 2;
    if (ver == 0) {
        std::cerr << "Version marker " << ver_marker << " is not recognised!" << std::endl;
    }
    
    ReceivedMessage recv_msg;
    if (ver == 2)
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
        if(request_received->status==OK && subscriptions.count(ID(request_received->id))) {
            subscriptions.at(ID(request_received->id))->decode(request_received->data);
        }
    }
    
    
    asio::async_read_until(port, buffer, std::bind(&Client::messageReady,this,std::placeholders::_1,std::placeholders::_2),
        std::bind(&Client::processOneMessage,this,std::placeholders::_1,std::placeholders::_2));
        
    if (print_debug_) std::cout << "processOneMessage finished" << std::endl;
}

std::pair<iterator, bool> Client::messageReady(iterator begin, iterator end)
{
    
    iterator i = begin;
    size_t available = std::distance(begin,end);
    
    if (available < 2) return std::make_pair(begin,false);
    
    if (*i == '$' && *(i+1) == 'M') {
        //not even enough data for a header
        if (available < 6) return std::make_pair(begin,false);

        uint8_t payload_size = *(i+3);
        //incomplete xfer
        if (available < size_t(5 + payload_size + 1)) return std::make_pair(begin,false);
        
        std::advance(i, 5+payload_size+1);
        
    } else if (*i == '$' && *(i+1) == 'X') {
        //not even enough data for a header
        if (available < 9) return std::make_pair(begin,false);
        
        
        uint16_t payload_size = uint8_t(*(i+6)) | uint8_t(*(i+7))<<8;
        
        //incomplete xfer
        if (available < size_t(8 + payload_size + 1)) return std::make_pair(begin,false);
        
        std::advance(i, 8+payload_size+1);
        
    } else {
        for (; i != end; ++i) {
            if (*i == '$') break;
        }
        //implicitly consume all if $ not found
    }
    
    return std::make_pair(i, true);
}


ReceivedMessage Client::processOneMessageV1() {
    ReceivedMessage ret;
    
    ret.status = OK;
    
    // message direction
    const uint8_t dir = extractChar();
    const bool ok_id = (dir!='!');
    if (!ok_id) std::cout << "id not recognized by FC" << std::endl; 
    
    // payload length
    const uint8_t len = extractChar();

    // message ID
    ret.id = extractChar();

    if(print_warnings_ && !ok_id) {
        std::cerr << "Message v1 with ID " << size_t(ret.id) << " is not recognised!" << std::endl;
    }
    
    // payload
    for(size_t i(0); i<len; i++) {
        ret.data.push_back(extractChar());
    }

    // CRC
    const uint8_t rcv_crc = extractChar();
    const uint8_t exp_crc = crcV1(ret.id,ret.data);
    const bool ok_crc = (rcv_crc==exp_crc);

    if(print_warnings_ && !ok_crc) {
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
    const uint8_t dir = extractChar();
    if (print_debug_) std::cout << "dir: " << dir << std::endl;
    const bool ok_id = (dir!='!');
    
    // flag
    const uint8_t flag = extractChar();
    if (print_debug_) std::cout << "flag: " << flag << std::endl;
    exp_crc = crcV2(exp_crc, flag);
    
    // message ID
    const uint8_t id_low = extractChar();
    const uint8_t id_high = extractChar();
    ret.id = uint32_t(id_low) | (uint32_t(id_high) << 8);
    if (print_debug_) std::cout << "id: " << ret.id << std::endl;
    exp_crc = crcV2(exp_crc, id_low);
    exp_crc = crcV2(exp_crc, id_high);
    
    // payload length
    const uint8_t len_low = extractChar();
    const uint8_t len_high = extractChar();
    uint32_t len = uint32_t(len_low) | (uint32_t(len_high) << 8);
    exp_crc = crcV2(exp_crc, len_low);
    exp_crc = crcV2(exp_crc, len_high);
    if (print_debug_) std::cout << "len: " << len << std::endl;

    if(print_warnings_ && !ok_id) {
        std::cerr << "Message v2 with ID " << size_t(ret.id) << " is not recognised!" << std::endl;
    }
    
    // payload
    ByteVector data;
    for(size_t i(0); i<len; i++) {
        ret.data.push_back(extractChar());
    }
    
    exp_crc = crcV2(exp_crc,ret.data);

    // CRC
    const uint8_t rcv_crc = extractChar();
    
    const bool ok_crc = (rcv_crc==exp_crc);

    if(print_warnings_ && !ok_crc) {
        std::cerr << "Message v2 with ID " << size_t(ret.id) << " has wrong CRC! (expected: " << size_t(exp_crc) << ", received: "<< size_t(rcv_crc) << ")" << std::endl;
    }

    if(!ok_id) { ret.status = FAIL_ID; }
    else if(!ok_crc) { ret.status = FAIL_CRC; }
    
    return ret;
    
}




} // namespace client
} // namespace msp
