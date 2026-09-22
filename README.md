# Arm Rover

<img width="2688" height="1512" alt="PXL_20260922_145024456" src="https://github.com/user-attachments/assets/6812098f-1e48-4b06-a70b-41c21931fbd9" />

## Features
For now it can drive to specified positions on its map.

## Map

<img width="1235" height="1027" alt="screenshot-2026-09-22_17-20-10" src="https://github.com/user-attachments/assets/fcb21f94-dda6-4142-bf32-3836938ea11f" />


## Hardware

The Chassis is build with wood bars and a lot of hot glue

- RaspberryPi 4
- Arduino Uno
- 2D Lidar ld d500
- imu: mpu6500
- 5v 2A UBEC, 5v 8A UBEC
- DC Motor controller build from spare parts: Mosfet Transistor, logic Transistor and Relay
- Geard TT-Motors with wheels
- Hall Sensors 49E
- Battery: 2200mAh LiPo

## Wheel Encoders
Custom designed and 3d Printed magnetic encoder using 4 2x5mm Neodym magnets, aranged in a 4 Pole encoder.

## Bumper Sensor
the lidar is mounted relatively high, so there is a danger of bumping into stuff thats below the line of sight of the lidar sensor.
The Bumper sensors are the cardboard sheets in the front of back which are mounted with foam at the lower end, so it compresses, and using hall sensors this movement can be detected.  

## Software

all of this glue code is mostly written by LLMs, \
my contribution is the Integration, and actually making it work.

## In Progress: Robot Arm

<img width="2688" height="1512" alt="PXL_20260922_145351409" src="https://github.com/user-attachments/assets/3d88d30b-e167-42c9-944f-3eee5537592d" />
