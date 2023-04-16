# pico-gas-meter
A minimalistic solution to monitor the rotation of a gas meter with the raspberry pi pico and a three axis magnetometer.

## Overview
pico-gas-meter is a script written for the Raspberry PI Pico and the Micropython firmware. It is used to log the gas consumption of a gas meter. "You can't optimise what you can't measure", which is why this project came about. The rotating magnet in a gas meter is detected by a triaxial magnetic sensor. The magnetic sensor is a Bosch BMM150 connected to a Raspberry Pi Pico W microcontroller. The microcontroller sends a data packet to a server via UDP after each rotation. Everything that arrives on the UDP port is currently written to a file on the server side.
