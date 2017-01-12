#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <boost/asio.hpp>
#include <mutex>

typedef unsigned int uint;

class NoConnection : public std::runtime_error {
public:
    NoConnection(const std::string &device, const std::string &msg)
        : runtime_error("Device not available: "+device+" ("+msg+")")
    { }
};

class SerialPort {
public:
    SerialPort();

    ~SerialPort();

    /**
     * @brief connect establish connection to serial device
     * @param device path or name of serial device
     * @param baudrate serial baudrate
     * @return true on success
     */
    bool connect(const std::string &device, const uint baudrate);

    /**
     * @brief getDevice obtain serial device
     * @return path or name of serial device
     */
    const std::string &getDevice() const;

    /**
     * @brief isOpen
     * @return true if device is open
     * @return false if device is not open
     */
    bool isOpen();

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

    /**
     * @brief hasData check if data is available
     * @return >0 amount of bytes ready to read
     * @return -1 on error
     */
    int hasData();

    /**
     * @brief clear flush the serial buffer to remove old data
     */
    void clear();

private:
    std::string device;
    boost::asio::io_service io;     ///<! io service
    boost::asio::serial_port port;  ///<! port for serial device
    std::mutex lock_write;
    std::mutex lock_read;
};

#endif // SERIALPORT_HPP
