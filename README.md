# UPCR TEC Module
RS-422 enabled module for bidirectional control of up to 2 TEC modules.
Refer to individual directories README for more details.

## Features
* 4 NTC temperature sensors
* 2 biderectional TEC drivers (12V input)
* Isolated RS-422 Bus
* STM32F030F4 on board microcontroller

## Hardware
This repository contains 
* Altium Designer files
* BOM
* Schematic PDF
Gerber files are available in releases

## Firmware
The code in this repository handles the general workflow of the module. 
* STM32 Cube MX project 
* Makefile
* Source code
* Python script to generate temperature lookup table
> To use this code you will need to define the communication protocol between devices (number of bytes, message structure) as well as tune the PID loop for temperature control.

