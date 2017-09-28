# Rocket Stove Temperature Controller Firmware
This is a C++ project designed to run on a [Teensy microcontroller](https://www.pjrc.com/teensy/), for the purpose of controlling the temperature of a rocket stove.

It was developed in [Xcode](https://developer.apple.com/xcode/) with the [embedXcode](http://embedxcode.weebly.com/) template. Xcode+embedXcode is a great OS X alternative to the Arduino editor.

This system is designed to take commands via the USB serial port. The [Rocket Stove Temp Controller GUI](https://github.com/johnvs/Rocket_Stove_Temp_Control_GUI) is a related project that acts as an interface for the microcontroller hardware.

## Usage
In addition to the Teensy microcontroller, the hardware required to fully implement this project consists of:
- a motor controller
- a stepper motor (with a home sensor)
- a 12VDC fan motor
- two thermocouples (and their interfaces)
- a potentiometer (used to manually control the fan speed)
- a 12VDC/5A power supply

It can also be used as an example of how to implement this type of system, one consisting of an embedded microcontroller (with a USB port) and a computer based GUI.

## License
[MIT](https://github.com/johnvs/Rocket_Stove_Temp_Control/blob/master/LICENSE)
