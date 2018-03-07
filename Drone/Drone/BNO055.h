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

ErrorCode BNO055_init_fusion_mode(bool calibrationNeeded);
ErrorCode BNO055_init_non_fusion_mode(uint8_t bno_mode_register);
ErrorCode BNO055_get_calibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
BNO055_Data BNO055_get_measurement_data();
ErrorCode BNO055_start_measurement(bool measureContinous, bool triggerCallback, uint8_t bnoMeasurementType);
ErrorCode BNO055_stop_continous_measurement();
ErrorCode BNO055_register_error_callback(BNO055_ERROR_CALLBACK callback);
ErrorCode BNO055_register_data_ready_callback(BNO055_DATA_READY_CALLBACK callback);
ErrorCode BNO055_calibrate();
bool BNO055_is_busy();
//TODO: safety check for bno is busy

#endif /* BNO055_H_ */