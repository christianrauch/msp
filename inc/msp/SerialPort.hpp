#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <boost/asio.hpp>

class SerialPort {
public:
    /**
     * @brief SerialPort
     * @param device path to serial device
     */
    SerialPort(const std::string &device);

    ~SerialPort();

    /**
     * @brief write write data vector to device
     * @param data raw data vector
     * @return true on success
     * @return false on failure
     */
    bool write(const std::vector<uint8_t> &data);

    /**
     * @brief read read data vector from device
     * @param data raw data vector in which read data will be stored
     * @return number of read bytes
     */
    size_t read(std::vector<uint8_t> &data);

    /**
     * @brief read read given amount of bytes from device
     * @param n_bytes number of bytes to read
     * @return data vector with read bytes
     */
    std::vector<uint8_t> read(std::size_t n_bytes);

    /**
     * @brief read read a single byte from device
     * @return single byte
     */
    uint8_t read();

private:
    boost::asio::io_service io;     ///<! io service
    boost::asio::serial_port port;  ///<! port for serial device
};

#endif // SERIALPORT_HPP
