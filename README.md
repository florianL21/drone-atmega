# 3D-Printed Quadcopter (Drone) - Controller

This Controller is for our self-designed 3D-printed Quadcopter which is based on an ATSAM3X8E microprocessor.
The Controller program and the different modules are written in C in Atmel Studio, with performance in mind.
The Controller is programmed to use Interrupts and DMA Controllers wherever possible to reduce processor load.
The update frequency of the PID Controllers reaches 250Hz and the IMU MEasurement frequency takes 4ms on average 
resulting in a 200Hz refresh frequency

# Modules

  - BNO055 			- Creates an interface for easy access of the bno msurements an other functions
  - ErrorHandling 	- Manages the custom error handling system used in this project
  - ESCControl 		- Controls the 4 motor-controllers (ESC) with PWM signals
  - FlashStorage 	- Creates an interface for easily accessing the flash storage on the ATSAM3x8e (Modified version of https://github.com/sebnil/DueFlashStorage)
  - GPT				- A module for easy access to general purpose timers and delay functions
  - HelperFunctions	- Diverse Functions for queues, mapping and other misc. functions
  - PID				- A Module that provises a PID Controller implementation (Modified version of https://github.com/geekfactory/PID)
  - RCReader 		- Reads the incoming PWM signals from the RC-receiver
  - SerialCOM		- Creates an interrupt based interface for PC-to-Drone communication with a custom protocol
  - UART0			- Provides functions for easy access to the UART0 control registers and DMA Controllers based on interrupts
  - USART0			- Provides functions for easy access to the USART0 control registers and DMA Controllers based on interrupts
  - WDT				- Functions for managing the WDT
  - more modules might come...
 

Powered by...
--
[![Atmel Studio 7](http://www.atmel.com/Images/Studio7__HomePage_980x352.jpg)](http://www.atmel.com/microsite/atmel-studio/)
![Atmel](https://micrium.com/wp-content/uploads/2012/08/Atmel-Logo.png)