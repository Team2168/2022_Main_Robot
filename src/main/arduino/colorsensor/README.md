# Color Sensor

This is arduino code for the color sensor.

Wiring map:
![map 4 0](https://user-images.githubusercontent.com/74201229/155244337-c20f173f-a93a-4156-8f16-831797d96b06.png)


## Installation

### Install the Arduino IDE

The Arduino IDE is used for all development of arduino code.  Download and install from [here](https://www.arduino.cc/en/software)

### Install Teensy support software

For this sensor, we used the Teensy 3.5 microcontroller.  It is compatible with Arduino software, but requires additional software to function.

Download and install teensyduino from [here](https://www.pjrc.com/teensy/td_download.html)

### Install Color Sensor Headers

These headers are installed as a git submodule for ease of use.  To get these, navigate to the root of this project and run `git submodule init`.  This will automatically clone the headers where they need to be.