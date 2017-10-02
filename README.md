# 3D-Printed Quadcopter (Drone) - Controller

This Controller for our self-designed 3D-printed Quadcopter is based on an ATmega2560 microprocessor.
The controller program and the different modules are written in C in Atmel Studio.

# Modules

  - ESCControl - Controls the 4 motor-controllers (ESC) with PWM signals
  - RCReader - Reads the incoming PWM signals from the RC-receiver
  - I2C - Implements the TWI/I2C communication, used by BNOCOM
  - UART0 - Implements the UART0 communication, used by UARTCOM
  - UARTCOM (WIP) - Creates an interface for PC-Drone communication with a custom protocol
  - BNOCOM (TBD) - Creates an interface to read data from the BNO055 sensor
  - more modules might come...
  - 
 

Powered by...
--
[![Atmel Studio 7](http://www.atmel.com/Images/Studio7__HomePage_980x352.jpg)](http://www.atmel.com/microsite/atmel-studio/)
![Atmel](https://micrium.com/wp-content/uploads/2012/08/Atmel-Logo.png)