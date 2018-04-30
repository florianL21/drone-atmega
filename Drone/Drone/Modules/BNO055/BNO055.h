/*
 * BNO_055.h
 *
 * Created: 27.03.2018 18:47:37
 *  Author: flola
 */ 


#ifndef BNO_055_H_
#define BNO_055_H_

#include "BNO055_reg_table.h"
#include "../UART/USART0.h"
#include "../ErrorHandling/ErrorHandling.h"
#include "../HelperFunctions/HelperFunctions.h"
#include <math.h>
#include "../WDT/WDT.h"
#include "../GPT/GPT.h"

//TODO: Remove after testing done:
#include "../SerialCOM/SerialCOM.h"

struct BNO055_Quat
{
	double x;
	double y;
	double z;
	double w;
};
typedef struct BNO055_Quat BNO055_Quat;

struct BNO055_Data
{
	float Roll;
	float Pitch;
	float Yaw;
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
};
typedef struct BNO055_Data BNO055_Data;

typedef enum {Calibrate_if_necessary, Do_not_calibrate, Force_calibration}BNO_INIT_CALIB;
	
typedef void (*BNO055_DATA_READY_CALLBACK)(uint8_t Data[], uint8_t Length);
typedef void (*BNO055_ERROR_CALLBACK)(ErrorCode Transmit_error_code);


ErrorCode BNO055_init(BNO_INIT_CALIB PerformCalib);

bool BNO055_IsReady();

bool BNO055_IsCalibrating();

ErrorCode BNO055_calibrate();

//WARNING: dataToRead must be a preallocated array in the exact same length as DataLength!!!!
ErrorCode BNO055_read_blocking(uint8_t RegisterAddress, uint8_t dataToRead[], uint8_t DataLength);

ErrorCode BNO055_write_blocking(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength);

ErrorCode BNO055_write(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength);

ErrorCode BNO055_read(uint8_t RegisterAddress, uint8_t DataLength);

ErrorCode BNO055_register_data_ready_callback(BNO055_DATA_READY_CALLBACK callback);

ErrorCode BNO055_register_error_callback(BNO055_ERROR_CALLBACK callback);

//the start pointer has to point to a preinitialised field with a minimum length of 8,
//where the quaternion data begins at the defined pointer position!
BNO055_Data BNO055_ConvertQuaToYPR(uint8_t* startPtr);

BNO055_Quat BNO055_GetQuat(uint8_t* startPtr);

BNO055_Data ConvertQuaToYPR(BNO055_Quat quat);


#endif /* BNO_055_H_ */