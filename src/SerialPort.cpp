#include <SerialPort.hpp>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <iostream>

using namespace boost::asio;

SerialPort::SerialPort(const std::string &device) : port(io), timer(io) {
    port.open(device);

    port.set_option(serial_port::baud_rate(115200));
    port.set_option(serial_port::parity(serial_port::parity::none));
    port.set_option(serial_port::character_size(serial_port::character_size(8)));
    port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
//    port.set_option(serial_port::flow_control(serial_port::flow_control::none));

    io.run();
}

SerialPort::~SerialPort() {
    port.close();
}

bool SerialPort::write(const std::vector<uint8_t> &data) {
    const std::size_t bytes_written = boost::asio::write(port, boost::asio::buffer(data.data(), data.size()));
    return (bytes_written==data.size());
}

size_t SerialPort::read(std::vector<uint8_t> &data) {
    return boost::asio::read(port, boost::asio::buffer(data.data(), data.size()));
}

std::vector<uint8_t> SerialPort::read(std::size_t n_bytes) {
    std::vector<uint8_t> data(n_bytes);
    read(data);
    return data;
}

uint8_t SerialPort::read() {
    return read(1).front();
}

size_t SerialPort::read(std::vector<uint8_t> &data, uint64_t time_ms) {
    io.reset();
    //timer.cancel();
    timer.expires_from_now(boost::posix_time::milliseconds(time_ms));
    timer.async_wait(boost::bind(&SerialPort::timer_handler, this, boost::asio::placeholders::error));

    size_t bt = 0;
    bool success = false;

    boost::asio::async_read(port, boost::asio::buffer(data.data(), data.size()), boost::bind(&SerialPort::read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred, std::ref(bt), std::ref(success)));


    std::cout<<"run done"<<std::endl;

    while(success==false) {}
    std::cout<<"done"<<std::endl;

    return bt;
}

bool SerialPort::isOpen() {
    return port.is_open();
}

void SerialPort::setDTR(bool flag) {
    int fd = port.native_handle();

    int data = TIOCM_DTR;
    if (!flag)
        ioctl(fd, TIOCMBIC, &data);
    else
        ioctl(fd, TIOCMBIS, &data);
}

void SerialPort::timer_handler(const boost::system::error_code& error) {
    std::cout<<"time error: "<<error<<std::endl;

    if(error)
        std::cout<<"error"<<std::endl;

    if(error == boost::asio::error::operation_aborted) {
        std::cout<<"timer cancled"<<std::endl;
    }
    else {
        std::cout<<"time passed"<<std::endl;
        port.cancel();
    }
}

void SerialPort::read_handler(const boost::system::error_code& error, std::size_t bytes_transferred, size_t &bt, bool &success) {
    std::cout<<"read error: "<<error<<std::endl;
    std::cout<<"bytes transferred: "<<bytes_transferred<<std::endl;

    if(error == boost::asio::error::operation_aborted) {
        std::cout<<"aborted"<<std::endl;
    }

    if(error==0 && bytes_transferred>0)
        timer.cancel();

    success = true;
    bt = bytes_transferred;
}
