# MultiWii / Cleanflight Communication Library (C++)

This library implements the MultiWii Serial Protocol ([MSP](http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol)) for communicating with a MultiWii or Cleanflight flight controller (FC) over a serial device.
It defines a **low-level API** for sending+encoding and receiving+decoding MSP messages, and a **high-level API** with a subscriber pattern to periodically request data from the FC and call callback functions as soon as a message is received.

The communication has been tested with MultiWii 2.4 on an Arduino Nano 3.0 where it can achieve a update rate of approximately 340Hz (for a FC cycle time of 2.8ms / 357Hz).

## Installation and Test
### Linux (Ubuntu / Debian)
- install asio: `sudo apt install --no-install-recommends libasio-dev`
- check out the source code and use cmake to compile: `mkdir build && cd build && cmake ..&& make -j`
- run the example program given the path to the serial device, e.g.: `./msp_read_test /dev/ttyUSB0`

### Windows
#### Requirements
- CMake
- asio: download [asio-1.10.8.zip](https://sourceforge.net/projects/asio/files/latest/download?source=files) and extract the header files, e.g. to `C:\asio-1.10.8`
- Visual C++ Build Tools: http://landinghub.visualstudio.com/visual-cpp-build-tools

#### Build
- open the Developer Command Prompt for Visual Studio
- change to the directory where you checked out msp
- `mkdir build`, `cd build`
- `cmake -G"NMake Makefiles" -DASIO_HEADER_PATH=C:\asio-1.10.8\include ..`
- build: `nmake`

#### Test
- `msp_read_test.exe COM3`

## Hardware Setup

You can connect to Arduino or Naze32 boards either by (1) using a built-in USB-to-Serial connector or (2) by direct serial connection (RX->TX, TX->RX, GND->GND, VCC->VCC). The first option is preferable for high transfer rates, as the direct connection is more exposed to transmission errors.

### Baseflight / Cleanflight until 1.11:
- the MSP update rate is determined by the looptime
- change the looptime in the CLI to e.g. 1000us (1000Hz): `set looptime=1000`, then `save`

### Cleanflight from 1.12 / Betaflight until 3.0.1
- the MSP update rate is fixed to 100Hz

### Betaflight 3.1
- change the update rate for the serial task in the range 100 ... 2000Hz
    - e.g. to 2000Hz: `set serial_update_rate_hz=2000`, then `save`
- it might be necessary to increase the serial baudrate in the Ports configuration tab
- test communication with higher baudrate: `./get_msp_info /dev/ttyUSB0 1000000`

### Sending and Receiving RC commands
- activate feature `RX_MSP`
    - via GUI: `Configuration` -> `Receiver` -> `Receiver Mode` -> `MSP RX input`
    - via CLI: execute `feature RX_MSP`
    - via API: `FlightController::enableRxMSP()` and reboot

Beginning with Cleanflight 2 and Betaflight 3, `MSP_SET_RAW_RC` messages are ignored on some targets with insufficient flash memory (like the naze32). You can verify this, if `MSP_RC` messages return an empty list of channels. To activate `MSP_SET_RAW_RC` messages on these targets, you need to add `#define USE_RX_MSP` to your `target.h`, e.g. `src/main/target/NAZE/target.h`.

## How to use the library (low-level API)

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
This method returns after the first try to read and encode any MSP message. Therefore, there exist alternative implementations of this communication pattern, for example to block until a valid package with correct id has been received:
```
bool request_block(msp::Request &request)
```
or to retry sending requests until a response is received:
```
bool request_wait(msp::Request &request, uint wait_ms)
```
which can be useful to block until the FC is available and responds to messages.

E.g. the Arduino Nano 3.0 is reseted when opening the serial device and needs some time to boot until it will respond to messages. `request_wait` will return when MultiWii is booted and able to respond to messages. From there on, `request_block` can be used to fetch data from the FC.

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

## High-level API

The high-level API allows to periodically request messages from the FCU.

### Instantiation
Instantiate and setup the `FlightController` class:
```
#include <FlightController.hpp>

fcu::FlightController fcu("/dev/ttyUSB0", 115200);

// wait for connection and setup
fcu.initialise();
```

### Periodic request for messages

Define a class that holds callback functions to process information of received message:
```
class App {
public:
    void onStatus(const msp::Status& status) {
        std::cout<<status;
    }

    void onImu(const msp::Imu& imu) {
        std::cout<<imu;
    }
}
```

Instantiate class with callbacks and register them to the FCU with the desired update rate in seconds:
```
App app;

fcu.subscribe(&App::onStatus, &app, 0.1);
fcu.subscribe(&App::onImu, &app, 0.01);
```

Requests are sent to and processed by the flight controller as fast as possible. It is important to note that the MultiWii FCU only processed a single message per cycle. All subscribed messages therefore share the effective bandwidth of 1/(2800 us) = 357 messages per second.

### Request and Send Messages
Additional messages that are not requested periodically can be requested by the method
```
bool request(msp::Request &request, const double timeout = 0)
```
You need to instantiate a message of type `msp::Request` and provide it to the method with an optional timeout.

Response messages are sent by
```
bool respond(const msp::Response &response, const bool wait_ack=true)
```
where the method will block until an acknowledge is received if `wait_ack=true` (default).
