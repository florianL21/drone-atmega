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


typedef void (*BNOCOM_ERROR_CALLBACK)(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code);
typedef void (*BNOCOM_SUCCESS_CALLBACK)(uint8_t* Data, uint8_t Length);

StatusCode BNO055_Setup(bool calibrationNeeded);
StatusCode BNO055_calculate_calibration(uint8_t CalibrationData, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
StatusCode BNO055_get_euler_data(bool measureContinous);

#endif /* BNO055_H_ */