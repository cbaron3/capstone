# Capstone
Source code for Mechatronic Systems Engineering Capstone project

# Requirements

## Teensyduino

Firmware is written for a Teensy 3.2 and makes use of Teensy features such as PWMServo and elapsedMicros. 

[Teensyduino](https://www.pjrc.com/teensy/teensyduino.html)

## Barometer

Library to interact with the on-board MPL3115A2. 

[MPL3115A2](https://github.com/sparkfun/MPL3115A2_Breakout/tree/master/Libraries/Arduino)

## GPS

Libraries to interact with the on-board Adafruit GPS module. Two were selected: one interrupt-based and one polling-based. The interrupt-based library is what we use.

[Adafruit_GPS - Interrupt based](https://github.com/adafruit/Adafruit_GPS)

[TinyGPS - Polling based](https://github.com/mikalhart/TinyGPS)

## IMU

Library to interact with the on-board MPU9250.

[MPU9250](https://github.com/bolderflight/MPU9250)

## AHRS 

Library to filter our raw IMU data to produce accurate roll/pitch/yaw.

[uNavAHR](https://github.com/bolderflight/uNavAHRS)

[Eigen - Library Dependency](https://github.com/bolderflight/Eigen)
