# MultiWii Communication Library (C++)

This library implements the MultiWii Serial Protocoll ([MSP](http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol)) for communicating with a MultiWii flight controller (FC) over a serial device.
It currently implements the sending and reading from a serial device and defines some of the messages and encoding/decoding functions for raw data.

## Installation and Test
- install boost: `sudo apt install libboost-system-dev`
- check out the source code and use cmake to compile: `mkdir build && cd build && cmake ..`
- run the example programm given the path to the serial device, e.g.: `./get_msp_info /dev/ttyUSB0`
