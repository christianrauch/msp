#include <asio.hpp>

struct msp::SerialPortImpl {
    SerialPortImpl() : port(io) { }

    asio::io_service io;     ///<! io service
    asio::serial_port port;  ///<! port for serial device
    asio::streambuf buffer;
};
