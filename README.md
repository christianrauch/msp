# MultiWii / Cleanflight / Baseflight Communication Library (C++)

This library implements the MultiWii Serial Protocol ([MSP](http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol)) for communicating with a MultiWii or Cleanflight flight controller (FC) over a serial device.
It defines a **low-level API** for sending+encoding and receiving+decoding MSP messages, and a **high-level API** with a subscriber pattern to periodically request data from the FC and call callback functions as soon as a message is received.

The communication has been tested with MultiWii 2.4 on an Arduino Nano 3.0 where it can achieve a update rate of approximately 340Hz (for a FC cycle time of 2.8ms / 357Hz) and Betaflight on a Naze32 Rev6 with update rates of max. 1200Hz.

## Installation and Test
### Linux (Ubuntu / Debian)
- install asio and ninja:
  ```sh
  sudo apt install -y --no-install-recommends ninja-build libasio-dev
  ```
- check out the source code and use cmake to compile:
  ```sh
  cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release
  cmake --build build
  ```
- run the example program given the path to the serial device:
  ```sh
  ./msp_read_test /dev/ttyUSB0
  ```

### Windows
#### Requirements
- CMake
- asio: download [asio-1.20.0.zip](https://sourceforge.net/projects/asio/files/latest/download?source=files) and extract the header files, e.g. to `C:\asio-1.20.0`
- Visual C++ Build Tools: https://visualstudio.microsoft.com/de/downloads/

#### Build and Test
- open the Developer Command Prompt for Visual Studio
- change to the directory where you checked out msp and run:
  ```sh
  cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release -DASIO_ROOT=C:\asio-1.20.0
  cmake --build build
  ```
- test with serial device:
  ```sh
  msp_read_test.exe COM3
  ```

## Hardware Setup

You can connect to Arduino or Naze32 boards either by (1) using a built-in USB-to-Serial connector or (2) by direct serial connection (RX->TX, TX->RX, GND->GND, VCC->VCC). The first option is preferable for high transfer rates, as the direct connection is more exposed to transmission errors.

### MultiWii
- the MSP update rate is determined by variable `LOOP_TIME` in `config.h`
    - e.g. `#define LOOP_TIME 2800` sets the loop time to 2800µs and the update rate to 1/0.0028 s = 357.14 Hz
- Some Arduino boards with an integrated USB-to-serial converter auto-reset themselves when a serial connection is established. If you send messages to the FC right after opening a connection the FC will miss your request and will not send a response. You can prevent this by either a delay between opening a connection and sending messages, or by disabling the Data Terminal Ready (DTR) line, e.g. via `stty -F /dev/ttyUSB0 -hupcl`.

### Cleanflight / Betaflight
- change the update rate for the serial task in the range 100 ... 2000Hz
    - e.g. to 2000Hz: `set serial_update_rate_hz=2000`, then `save`
- it might be necessary to increase the serial baudrate in the Ports configuration tab

### Sending and Receiving RC commands
- activate feature `RX_MSP`
    - via GUI: `Configuration` -> `Receiver` -> `Receiver Mode` -> `MSP RX input`
    - via CLI: execute `feature RX_MSP`

Beginning with Cleanflight 2 and Betaflight 3, `MSP_SET_RAW_RC` messages are ignored on some targets with insufficient flash memory (like the naze32). You can verify this, if `MSP_RC` messages return an empty list of channels. To activate `MSP_SET_RAW_RC` messages on these targets, you need to add `#define USE_RX_MSP` to your `target.h`, e.g. `src/main/target/NAZE/target.h`.

## FlightContoller API

The FlightContoller API allows the user to send/receive messages from the flight controller. Messages may be queried periodically, or on demand.

### Instantiation
Instantiate and setup the `FlightController` class:
```C++
#include <FlightController.hpp>

fcu::FlightController fcu;

// do connection and setup
fcu.connect("/dev/ttyUSB0", 115200);
```

### Periodic request for messages
Messages that need to be queried periodically can be handled automatically using a subscription. This can be done with a class method or a stand-alone function.


#### Class method pattern
Define a class that holds callback functions to process information of received message. The function signature is `void <callback_name>(const <child_of_msp::Message>& )`.
```C++
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
```C++
App app;

fcu.subscribe(&App::onStatus, &app, 0.1);
fcu.subscribe(&App::onImu, &app, 0.01);
```

Requests are sent to and processed by the flight controller as fast as possible. It is important to note that the MultiWii FCU only processed a single message per cycle. All subscribed messages therefore share the effective bandwidth of 1/(2800 us) = 357 messages per second.

#### Lambda pattern

The `FlightController::subscribe` method is restricted to callbacks which return `void` and take an argument of `const msp::Message&`. In order to call a method that doesn't match that signature (maybe it needs additional information), it sometimes is useful to wrap the non-compliant method in a lambda that matches the expected signature.

```C++
auto callback = [](const msp::msg::RawImu& imu){
    // ImuSI is not a subclass of msp::Message, so it's not suitable for use as a callback argument
    std::cout << msp::msg::ImuSI(imu, 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);
}

fcu.subscribe<msp::msg::RawImu>(callback, 0.1);
```


### Manual sending of messages
Additional messages that are not sent periodically can be dispatched by the method
```C++
bool sendMessage(msp::Message &message, const double timeout = 0)
```
You need to instantiate an instance of the object matching the message you want to send and provide it to the method with an optional timeout. If the timeout paramter is 0, then the method will wait (possibly forever) until a response is received from the flight controller. A strictly positive value will limit the waiting to the specified interval.

```C++
msp::Status status;
if (fcu.sendMessage(status) ) {
    // status will contain the values returned by the flight controller
}

msp::SetCalibrationData calibration;
calibration.acc_zero_x = 0;
calibration.acc_zero_y = 0;
calibration.acc_zero_x = 0;
calibration.acc_gain_x = 1;
calibration.acc_gain_y = 1;
calibration.acc_gain_z = 1;
if (fcu.sendMessage(calibration) ) {
    // calibration data has been sent
    // since this message does not have any return data, the values are unchanged
}
```

If the message is of a type that has a data response from the flight controller, the instance of the message provided to the `sendMessage` call will contain the values unpacked from the flight controller's response.
