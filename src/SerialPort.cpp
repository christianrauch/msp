#include <SerialPort.hpp>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <iostream>

#include <poll.h>

using namespace boost::asio;

SerialPort::SerialPort(const std::string &device) : port(io) {
    port.open(device);

    port.set_option(serial_port::baud_rate(115200));
    port.set_option(serial_port::parity(serial_port::parity::none));
    port.set_option(serial_port::character_size(serial_port::character_size(8)));
    port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
//    port.set_option(serial_port::flow_control(serial_port::flow_control::none));
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

int SerialPort::poll(int timeout) {
    pollfd fd = {.fd = port.native_handle(),
                 .events = POLLIN,
                 .revents = 0};
    return ::poll(&fd, 1, timeout);
}
