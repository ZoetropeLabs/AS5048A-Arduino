AS5048A-Arduino
===============

A simple SPI library to interface with Austria Microsystem's AS5048A angle sensor with an Arduino.

The sensor should be connected to the hardware SPI pins (MISO, MOSI, SCK). The CS pin can be connected to any GPIO pin but should be passed to the constructor.
The second argument in the constructor is the time the library should wait for the sensor to have the data available in order to read it. It is not usually a problem
using ATmega microcontrollers but it can be a problem using fast microcontrollers such as the ESP32. For these cases, a value of around 50 would do, but should be
customized.

The angle sensor is described in more detail [here](zoetrope.io/AS5048)
