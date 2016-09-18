# MultiWii Communication Library (C++)

This library implements the MultiWii Serial Protocoll ([MSP](http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol)) for communicating with a MultiWii flight controller (FC) over a serial device.
It currently implements the sending and reading from a serial device and defines some of the messages and encoding/decoding functions for raw data.

The communication has been tested with MultiWii 2.4 on an Arduino Nano 3.0 where it can achieve a update rate of approximately 300Hz (for a FC cycle time of 2.8ms / 357Hz).

## Installation and Test
- install boost: `sudo apt install libboost-system-dev`
- check out the source code and use cmake to compile: `mkdir build && cd build && cmake ..`
- run the example programm given the path to the serial device, e.g.: `./get_msp_info /dev/ttyUSB0`

## How to use the library

You first need to instantiate the driver with the path to the device:
```
msp::MSP msp(path_to_device);
```

### Sending and Receiving Raw Data
The library contains methods for sending data to and for receiving data from the flight controller (FC). These methods communicate once and indicate success by return values and exceptions.

Sending raw data to the FC:
```
bool sendData(const uint8_t id, const ByteVector &data)
```

Receiving data from the FC:
```
DataID receiveData()
```
`struct DataID` contains the id and data of the received message.

### Communication Pattern
The communication with the FC follows the remote procedure call pattern where we can either request data from the FC or respond with data to the FC.

This process is automated using two types of message: `Request` (receive and decode data) and `Response` (encode and send data).

#### Request data from FC
For requesting data from the FC (FC →), a message with the id of that command is send to the FC and then we need to wait for a message with the same id containing the requested payload.

Instantiate any message that inherits from `Request` and pass it to:
```
bool request(msp::Request &request)
```
This method returns after the forst try to read and encode any MSP message. Therefore, there exist alternative implementations of this communication pattern, for example to block until a valid package with correct id has been received:
```
bool request_block(msp::Request &request)
```
or to timeout while reading and resending the request:
```
bool request_timeout(msp::Request &request, unsigned int timeout_ms)
```
which can be useful to block until the FC is available.

E.g. the Arduino Nano 3.0 is reset when openning the serial device and needs some time to boot until it will respond to messages. `request_timeout` will return when MultiWii is booted and able to respond to messages. From there on, `request_block` can be used to wait for data.

#### Respond with data to FC
For sending data to the FC (→ FC) a message containing the id and the payload is send to the FC and confirmed by a acknowledge message which only contains the id with no data.

Instantiate any message that inherits from `Response` and pass it to:
```
bool respond(msp::Response &response)
```

Messages and the encoding/decoding methods are defined in `msp_msg.hpp`.

### Examples

#### Get MultiWii version and multi-copter type

We will use the command MSP_IDENT (100) to get the version, multi-copter type and it capabilities.

Instantiate the driver:
```
msp::MSP msp(path_to_device);
```
create request message and send the request to the FC:
```
msp::Ident ident;
msp.request_block(ident);
```
When the call to `request_block` returns, the values of structure `ident` will be populated can be accessed:
```
std::cout<<"MSP version "<<(int)ident.version<<std::endl;
```

## TODO
- implement remaining request and response messages
- create high-level API that uses the message to extract further information (e.g. hasBaro(), arm(), disarm()) and reports SI units instead of sensor depending values
