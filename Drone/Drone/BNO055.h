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

struct BNO055_eulerData  
{
	float role;
	float pitch;
	float heading;
}; 
typedef struct BNO055_eulerData BNO055_eulerData;


typedef void (*BNO055_ERROR_CALLBACK)(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code);
typedef void (*BNO055_DATA_READY_CALLBACK)(void);

StatusCode BNO055_init(bool calibrationNeeded);
StatusCode BNO055_get_calibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
BNO055_eulerData BNO055_get_euler_measurement_data();
StatusCode BNO055_start_euler_measurement(bool measureContinous, bool triggerCallback);
StatusCode BNO055_stop_continous_measurement();
StatusCode BNO055_register_error_callback(BNO055_ERROR_CALLBACK callback);
StatusCode BNO055_register_data_ready_callback(BNO055_DATA_READY_CALLBACK callback);
StatusCode BNO055_calibrate();
bool BNO055_is_busy();
//TODO: safety check for bno is busy

#endif /* BNO055_H_ */