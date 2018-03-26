/*
 * BNO055.c
 *
 * Created: 23.11.2017 21:44:11
 *  Author: flola
 */ 

#include "BNO055.h"






/***************************************************************************************************
* BNO Top Level interaction:
****************************************************************************************************/
ErrorCode bno_calculate_calibration(uint8_t CalibrationData, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);


void bno_runtime_error(BNO_STATUS_BYTES Error, ErrorCode Transmit_error_code);
void bno_runtime_success(uint8_t* Data, uint8_t Length);

typedef enum {
	bno_none = 0,
	bno_euler_measurement = 1,
	bno_calibration_measurement = 2,
	bno_temp_measurement = 3,
	bno_calibration_in_progress = 4
} bno_requested_measurement;

BNO055_DATA_READY_CALLBACK bno_data_ready_callback = NULL;
BNO055_ERROR_CALLBACK bno_error_callback = NULL;
bool bno_measureContinous = false;
bool bno_triggerCallback = false;
BNO055_Data lastMeasuredData;
bool bno_is_busy = false;
bool bno_is_connected = false;
uint8_t bnoReadRegister = 0;




ErrorCode BNO055_init_fusion_mode(bool calibrationNeeded)
{
	bno_is_busy = true;
	
	DEFAULT_ERROR_HANDLER(BNOCOM_Init(bno_runtime_success), MODULE_BNO055, FUNCTION_init_fusion_mode);
	
	DEFAULT_ERROR_HANDLER(BNOCOM_register_error_callback(bno_runtime_error), MODULE_BNO055, FUNCTION_init_fusion_mode);
	//Wait for USART0 to clear its backlog if there is any
	
	while(!USART0_is_idle())
	{
		
	}
	
	//Check for right device
	//Read Chip-id 
	uint8_t Data = 0;
	
	DEFAULT_ERROR_HANDLER(BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CHIP_ID, 0, &Data), MODULE_BNO055, FUNCTION_init_fusion_mode);
	if(Data != BNO055_ID)
		return MODULE_BNO055 | FUNCTION_init_fusion_mode | ERROR_WRONG_DEVICE_ID;
	//sensor defaults to OPR_MODE -> config mode
	DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), MODULE_BNO055, FUNCTION_init_fusion_mode);
	
	//sensor defaults to PWR_MODE -> normal mode
	DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PWR_MODE, 0, BNO_PWR_MODE_NORMAL), MODULE_BNO055, FUNCTION_init_fusion_mode);
	
	//sensor defaults to PAGE_ID -> PAGE0
	DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PAGE_ID, 0, BNO_PAGE_ID0), MODULE_BNO055, FUNCTION_init_fusion_mode);
	
	//Set output units:
	Data =	(0<<7) | //Format = Windows
			(0<<4) | //Temperature = Celsius
			(0<<2) | //Euler = Degrees
			(1<<1) | //Gyro = Rad/s
			(0<<0);  //Accelerometer = m/s^2
	_Delay(1000000);
	DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_UNIT_SEL, 0, Data), MODULE_BNO055, FUNCTION_init_fusion_mode);
	_Delay(10000);
	//sensor defaults to SYS_TRIGGER -> Internal oscillator
	DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_SYS_TRIGGER, 0, BNO_INTERNAL_OSC), MODULE_BNO055, FUNCTION_init_fusion_mode);
	
	if(calibrationNeeded)
	{
		//Switch to Fusion Mode
		DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, FUSION_MODE_NDOF), MODULE_BNO055, FUNCTION_init_fusion_mode);
		
		//Calibrating		
		BNO055_calibrate();
		//Switch to Config Mode
		DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), MODULE_BNO055, FUNCTION_init_fusion_mode);
		
				
		//TODO: write Data to EEPROM
			
		
	}else{
		//TODO: read calibration data from EEPROM
	}
	
	//Set Operation Mode to NDOF (nine degrees of freedom)
	DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, FUSION_MODE_NDOF), MODULE_BNO055, FUNCTION_init_fusion_mode);
	//Initialize data struct to zero
	lastMeasuredData.X = 0;
	lastMeasuredData.Y = 0;
	lastMeasuredData.Z = 0;
	bno_is_busy = false;
	//Initialization finished
	bno_is_connected = true;
	return SUCCESS;
}

ErrorCode BNO055_init_non_fusion_mode(uint8_t bno_mode_register)
{
	if(bno_mode_register >= 0x01 && bno_mode_register <= 0x07)
	{
		bno_is_busy = true;
		
		DEFAULT_ERROR_HANDLER(BNOCOM_Init(bno_runtime_success), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		
		DEFAULT_ERROR_HANDLER(BNOCOM_register_error_callback(bno_runtime_error), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		
		//Wait for USART0 to clear its backlog if there is any
		while(!USART0_is_idle());
		
		//Check for right device
		//Read Chip-id
		uint8_t Data = 0;
		
		DEFAULT_ERROR_HANDLER(BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CHIP_ID, 0, &Data), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		if(Data != BNO055_ID)
			return MODULE_BNO055 | FUNCTION_init_fusion_mode | ERROR_WRONG_DEVICE_ID;
		
		//sensor defaults to OPR_MODE -> config mode
		DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		
		//sensor defaults to PWR_MODE -> normal mode
		DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PWR_MODE, 0, BNO_PWR_MODE_NORMAL), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		
		//sensor defaults to PAGE_ID -> PAGE0
		DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PAGE_ID, 0, BNO_PAGE_ID0), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		
		//Set output units:
		Data =	(0<<7) | //Format = Windows
		(0<<4) | //Temperature = Celsius
		(0<<2) | //Euler = Degrees
		(0<<1) | //Gyro = Degrees/s
		(0<<0);  //Accelerometer = m/s^2
		_Delay(1000000);
		DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_UNIT_SEL, 0, Data), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		_Delay(10000);
		//sensor defaults to SYS_TRIGGER -> Internal oscillator
		DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_SYS_TRIGGER, 0, BNO_INTERNAL_OSC), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		
		//Set Operation Mode to NDOF (nine degrees of freedom)
		DEFAULT_ERROR_HANDLER(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, bno_mode_register), MODULE_BNO055, FUNCTION_init_non_fusion_mode);
		//Initialize data struct to zero
		lastMeasuredData.X = 0;
		lastMeasuredData.Y = 0;
		lastMeasuredData.Z = 0;
		bno_is_busy = false;
		//Initialization finished
		bno_is_connected = true;
		return SUCCESS;
	}
	return MODULE_BNO055 | FUNCTION_init_non_fusion_mode | ERROR_ARGUMENT_OUT_OF_RANGE;
}

ErrorCode BNO055_calibrate()
{
	uint8_t Data = 0;
	uint8_t sys = 0;
	uint8_t gyro = 0;
	uint8_t accel = 0;
	uint8_t mag = 0;
	bno_is_busy = true;
	while(sys != 3)
	{
		char buffer[20] = "";
		sprintf(buffer,"Sys: %d", sys);
		SerialCOM_put_debug(buffer);
		_Delay(252000);
		ErrorCode readReturn = BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CALIB_STAT, 0, &Data);
		while((readReturn & 0x0000FF) == ERROR_GENERIC && Data == BNO_STATUS_BUS_OVER_RUN_ERROR)
		{
			readReturn = BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CALIB_STAT, 0, &Data);
			SerialCOM_put_debug("Bus Error");
			_Delay(840000);
		}
		if(readReturn != SUCCESS && Data != BNO_STATUS_BUS_OVER_RUN_ERROR)
		{
			return ErrorHandling_set_top_level(readReturn, MODULE_BNO055, FUNCTION_calibrate);
		}
		DEFAULT_ERROR_HANDLER(bno_calculate_calibration(Data, &sys, &gyro, &accel, &mag), MODULE_BNO055, FUNCTION_calibrate);
	}
	bno_is_busy = false;
	return SUCCESS;
}

ErrorCode BNO055_get_calibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
	uint8_t Data = 0;
	bno_is_busy = true;
	DEFAULT_ERROR_HANDLER(BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CALIB_STAT, 0, &Data), MODULE_BNO055, FUNCTION_get_calibration);
	DEFAULT_ERROR_HANDLER(bno_calculate_calibration(Data, sys, gyro, accel, mag), MODULE_BNO055, FUNCTION_get_calibration);
	bno_is_busy = false;
	return SUCCESS;
}

ErrorCode bno_calculate_calibration(uint8_t CalibrationData, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
	if(sys == NULL || gyro == NULL || accel == NULL || mag == NULL)
		return MODULE_BNO055 | FUNCTION_calculate_calibration | ERROR_GOT_NULL_POINTER;
	//Write the data to the proper variables
	*sys = (CalibrationData >> 6) & 0x03;
	*gyro = (CalibrationData >> 4) & 0x03;
	*accel = (CalibrationData >> 2) & 0x03;
	*mag = CalibrationData & 0x03;
	return SUCCESS;
}

ErrorCode BNO055_register_data_ready_callback(BNO055_DATA_READY_CALLBACK callback)
{
	if(callback == NULL)
		return MODULE_BNO055 | FUNCTION_register_data_ready_callback | ERROR_GOT_NULL_POINTER;
	bno_data_ready_callback = callback;
	return SUCCESS;
}

ErrorCode BNO055_register_error_callback(BNO055_ERROR_CALLBACK callback)
{
	if(callback == NULL)
		return MODULE_BNO055 | FUNCTION_register_error_callback | ERROR_GOT_NULL_POINTER;
	bno_error_callback = callback;
	return SUCCESS;
}

void bno_runtime_success(uint8_t* sensorData, uint8_t Length)
{
	if((sensorData == NULL || Length == 0) && bno_error_callback != NULL)
		bno_error_callback(BNO_TRANSMIT_ERROR, MODULE_BNO055 | FUNCTION_runtime_success | ERROR_GOT_NULL_POINTER);
	if(Length == 6) // Data
	{ 
		lastMeasuredData.X = (((uint16_t)sensorData[1]) << 8) | sensorData[0];
		lastMeasuredData.Y = (((uint16_t)sensorData[3]) << 8) | sensorData[2];
		lastMeasuredData.Z = (((uint16_t)sensorData[5]) << 8) | sensorData[4];
		
	}

	if(bno_data_ready_callback != NULL && bno_triggerCallback == true)
		bno_data_ready_callback();
	if(bno_measureContinous == true)
	{
		ErrorHandling_throw(ErrorHandling_set_top_level(BNOCOM_register_read_by_table(bnoReadRegister, 0, 6), MODULE_BNO055, FUNCTION_runtime_success));
		bno_is_busy = true;
	}
	bno_is_busy = false;
}

bool BNO055_is_busy()
{
	return bno_is_busy;
}

bool BNO055_is_connected()
{
	return bno_is_connected;
}

void bno_runtime_error(BNO_STATUS_BYTES Error, ErrorCode Transmit_error_code)
{
	if(bno_error_callback != NULL)
		bno_error_callback(Error, Transmit_error_code);
}

ErrorCode BNO055_stop_continuous_measurement()
{
	bno_measureContinous = false;
	return SUCCESS;
}

BNO055_Data BNO055_get_measurement_data()
{
	return lastMeasuredData;
}

ErrorCode BNO055_start_measurement(bool measureContinous, bool triggerCallback, uint8_t bnoMeasurementType)
{
	if(bnoMeasurementType == BNO_REG_ACC_DATA_X || bnoMeasurementType == BNO_REG_MAG_DATA_X 
		|| bnoMeasurementType == BNO_REG_GYR_DATA_X || bnoMeasurementType == BNO_REG_EUL_DATA_X 
		|| bnoMeasurementType == BNO_REG_LIA_DATA_X || bnoMeasurementType == BNO_REG_GRV_DATA_X)
	{
		bno_is_busy = true;
		bno_measureContinous = measureContinous;
		bno_triggerCallback = triggerCallback;
		bnoReadRegister = bnoMeasurementType;
		return ErrorHandling_set_top_level(BNOCOM_register_read_by_table(bnoReadRegister, 0, 6), MODULE_BNO055, FUNCTION_start_measurement);
	}
	return MODULE_BNO055 | FUNCTION_start_measurement | ERROR_ARGUMENT_OUT_OF_RANGE;
}