#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <boost/asio.hpp>

class SerialPort {
public:
    SerialPort(const std::string &device);
    ~SerialPort();

    bool write(const std::vector<uint8_t> &data);

    size_t read(std::vector<uint8_t> &data);

    std::vector<uint8_t> read(std::size_t n_bytes);

    uint8_t read();

    size_t read(std::vector<uint8_t> &data, uint64_t time_ms);

    bool isOpen();

    void setDTR(bool flag);

private:
    boost::asio::io_service io;
    boost::asio::serial_port port;
    boost::asio::deadline_timer timer;

    void timer_handler(const boost::system::error_code& error);

    void read_handler(const boost::system::error_code& error, std::size_t bytes_transferred, size_t &bt, bool &success);
};

#endif // SERIALPORT_HPP
