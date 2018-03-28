/*
 * BNO_055.h
 *
 * Created: 27.03.2018 18:47:37
 *  Author: flola
 */ 


#ifndef BNO_055_H_
#define BNO_055_H_

#include "BNO055_reg_table.h"
#include "USART0.h"
#include "ErrorHandling.h"
#include "HelperFunctions.h"

//TODO: Remove after testing done:
#include "SerialCOM.h"

struct BNO055_Data
{
	int16_t X;
	int16_t Y;
	int16_t Z;
};
typedef struct BNO055_Data BNO055_Data;

typedef enum {Calibrate_if_necessary, Do_not_calibrate, Force_calibration}BNO_INIT_CALIB;
	
typedef void (*BNO055_DATA_READY_CALLBACK)(uint8_t Data[], uint8_t Length);
typedef void (*BNO055_ERROR_CALLBACK)(ErrorCode Transmit_error_code);

ErrorCode BNO055_init(BNO_INIT_CALIB PerformCalib);

//WARNING: dataToRead must be a preallocated array in the exact same length as DataLength!!!!
ErrorCode BNO055_read_blocking(uint8_t RegisterAddress, uint8_t dataToRead[], uint8_t DataLength);

ErrorCode BNO055_write_blocking(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength);

ErrorCode BNO055_write(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength);

ErrorCode BNO055_read(uint8_t RegisterAddress, uint8_t DataLength);

ErrorCode BNO055_register_data_ready_callback(BNO055_DATA_READY_CALLBACK callback);

ErrorCode BNO055_register_error_callback(BNO055_ERROR_CALLBACK callback);


#endif /* BNO_055_H_ */