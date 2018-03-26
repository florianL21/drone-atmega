/*
 * BNO055.h
 *
 * Created: 23.11.2017 21:44:00
 *  Author: flola
 */ 


#ifndef BNO055_H_
#define BNO055_H_

#include "sam.h"
#include <stdlib.h>
#include <stdbool.h>
#include "config.h"
#include "BNOCOM.h"
#include "BNO055_reg_table.h"
#include <stdio.h>
#include "SerialCOM.h"
#include "ErrorHandling.h"

/*Predefined bno constants*/
#define BNO055_ID				0xA0
#define BNO_CONFIG_MODE			0x00
#define BNO_PWR_MODE_NORMAL		0x00
#define BNO_PAGE_ID0			0x00
#define BNO_PAGE_ID1			0x01
#define BNO_INTERNAL_OSC		0x00
//fusion modes:
#define FUSION_MODE_NDOF		0x0C
//non fusion modes:
#define NONFUSION_MODE_ACCONLY	0x01
#define NONFUSION_MODE_MAGONLY	0x02
#define NONFUSION_MODE_GYROONLY	0x03
#define NONFUSION_MODE_ACCMAG	0x04
#define NONFUSION_MODE_ACCGYRO	0x05
#define NONFUSION_MODE_MAGGYRO	0x06
#define NONFUSION_MODE_AMG		0x07
//data pointers:
#define BNO_REG_ACC_DATA		BNO_REG_ACC_DATA_X
#define BNO_REG_MAG_DATA		BNO_REG_MAG_DATA_X
#define BNO_REG_GYR_DATA		BNO_REG_GYR_DATA_X
#define BNO_REG_LIA_DATA		BNO_REG_LIA_DATA_X
#define BNO_REG_GRV_DATA		BNO_REG_GRV_DATA_X


struct BNO055_Data  
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}; 
typedef struct BNO055_Data BNO055_Data;


typedef void (*BNO055_ERROR_CALLBACK)(BNO_STATUS_BYTES Error, ErrorCode Transmit_error_code);
typedef void (*BNO055_DATA_READY_CALLBACK)(void);

/*
	Init the bno055 module for fusion mode
	This sets up the USART0 for communication with the BNO, sets all callbacks correspondingly
	and sets up all registers inside of the sensor for fusion mode.
	WARNING: If the bno is not connected or not correctly wired calling this function WILL cause an endless loop to occur!
	Parameters:
		- calibrationNeeded:	If true starts the bno calibration routine and exits only when the sensor is fully calibrated.
									IMPORTANT: the watchdog timer should not be started before calling the calibration routine
								If false the calibration offsets are set from the values saved in the flash memmory
	Returns:
		Any error that might have occurred.
*/
ErrorCode BNO055_init_fusion_mode(bool calibrationNeeded);

/*
	Init the bno055 module for non fusion mode
	This sets up the USART0 for communication with the BNO, sets all callbacks correspondingly
	and sets up all registers inside of the sensor for fusion mode.
	WARNING: If the bno is not connected or not correctly wired calling this function WILL cause an endless loop to occur!
	Parameters:
		- bno_mode_register:	This sets the mode register of the BNO. Valid values are all defines which start with "NONFUSION_MODE_"
								The valid numeric values range from 0x01 to 0x07
	Returns:
		Any error that might have occurred.
*/
ErrorCode BNO055_init_non_fusion_mode(uint8_t bno_mode_register);

/*
	This function returns the calibration status for the gyro, accelerometer, magnetometer and the system calibration as pointers.
	Requirements:
		- BNO055_init_fusion_mode or BNO055_init_non_fusion_mode
	Parameters:
		- bno_mode_register:	This sets the mode register of the BNO. Valid values are all defines which start with "NONFUSION_MODE_"
								The valid numeric values range from 0x01 to 0x07
	Returns:
		Any error that might have occurred.
*/
ErrorCode BNO055_get_calibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

/*
	This function returns the last measured data.
	Requirements:
		- BNO055_init_fusion_mode or BNO055_init_non_fusion_mode
	Returns:
		BNO055_Data structure with the measured values
*/
BNO055_Data BNO055_get_measurement_data();

/*
	This function starts a BNO Measurement.
	Requirements:
		- BNO055_init_fusion_mode or BNO055_init_non_fusion_mode
	Parameters:
		- measureContinous:		If true this will start a new measurement as soon as the one before it is done.
		- triggerCallback:		If true this will trigger the BNO055_data_ready_callback as soon as a measurement is done.
		- bnoMeasurementType:	Defines which measurement should be performed. Valid values are all BNO_REG_*_DATA defines.
	Returns:
		Any error that might have occurred.				
*/
ErrorCode BNO055_start_measurement(bool measureContinous, bool triggerCallback, uint8_t bnoMeasurementType);

/*
	This function stops the continuous measurement. All measurements which were requested before calling this function will be performed anyways.
	Requirements:
		- BNO055_init_fusion_mode or BNO055_init_non_fusion_mode
	Returns:
		Any error that might have occurred.				
*/
ErrorCode BNO055_stop_continuous_measurement();

/*
	This function registers an error callback which gets called when an error in the interrupt routine occurs.
	Parameters:
		- callback:	function pointer to the callback
	Returns:
		Any error that might have occurred.				
*/
ErrorCode BNO055_register_error_callback(BNO055_ERROR_CALLBACK callback);

/*
	This function registers an data ready callback which gets called when a measurement is complete and the trigger data_ready_callbeck is set to true.
	Parameters:
		- callback:	function pointer to the callback
	Returns:
		Any error that might have occurred.				
*/
ErrorCode BNO055_register_data_ready_callback(BNO055_DATA_READY_CALLBACK callback);

/*
	This function starts the calibration process of the BNO. 
	WARNING: this is a BLOCKING function. it contains a while loop which lasts until the BNO is calibrated correctly. WDT timers are likely to trigger when calling this function.
	Returns:
		Any error that might have occurred.				
*/
ErrorCode BNO055_calibrate();

/*
	This function returns true if the BNO is busy
	Returns:
		true when busy, false if not.			
*/
bool BNO055_is_busy();

/*
	This function returns true if the BNO is setup correctly and ready to use. This does NOT check for BNO055_is_busy()!!!!
	Returns:
		true when busy, false if not.			
*/
bool BNO055_is_connected();
//TODO: safety check for bno is busy

#endif /* BNO055_H_ */