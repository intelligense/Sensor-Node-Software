Sensor-Node-Software
====================
This repository contains the microcontroller code, BLE module configuration, hardware schematics, and PCB design files.
The sensor node is designed to perform inertial measurements, do some basic data processing, and transmit the results to a remote device over BLE.


#Software
The project is developed on an example project provided by TI.
##Files
1. compdcm_mpu9150.c file is main file of the project.
2. startup_ccs.c file contains configurations of interrupt service routine for the project.
3. utils.c and utils.h are the utility files that handles UART communication, allowing the project to print messages to PC via ICDI as well as communicate with the BLE module.
4. cmd_def.c, cmd_def.h, and apitypes.h are BGLib library files which handles communication with BLE module.
5. stub.c, stores unused BGLib library methods.


#Components
There are 3 major components: IMU, microcontroller, and BLE module.
All components cost around CAD $70 from Digikey.ca.
##IMU: InvenSense MPU9150
The MPU9150 is a 9-axis sensor with good accuracy for its accelerometer and gyroscope. 
The magnetometer has large zero biases and is practically unusable without calibration. 
The sensor has mutliple ranges for sensors and 16-bit ADC for digital output. 
The MPU9150 communicates with microcontroller over I2C.
##Microcontroller: Texas Instrument TM4C123G
The TM4C123G is a 32-bit Cortex-M4 processor with floating-point unit extension.
The microcontroller has powerful floating-point (single precision) computation capability, ideal for processing sensor data real-time.
##BLE module: Bluegiga BLE113
The BLE113 is a directly usable BLE module.
With a little bit configuration, the BLE113 can handle data transmissions to any BLE capable device.
The BLE113 communicates with the microcontroller over UART.


#Schematics and PCB design files
The schematics as well as the PCB design files are saved in the /schematic folder.
The hardware system design follows examples provided by TI and Bluegiga.
With current design, the coin battery is not usable.
