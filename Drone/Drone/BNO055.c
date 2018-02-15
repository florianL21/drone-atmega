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
StatusCode bno_calculate_calibration(uint8_t CalibrationData, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);


void bno_runtime_error(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code);
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
uint8_t bnoReadRegister = 0;




StatusCode BNO055_init_fusion_mode(bool calibrationNeeded)
{
	bno_is_busy = true;
	StatusCode error_return;
	
	DEFUALT_ERROR_HANDLER1(BNOCOM_Init(bno_runtime_success), error_return);
	
	DEFUALT_ERROR_HANDLER1(BNOCOM_register_error_callback(bno_runtime_error), error_return);
	//Wait for USART0 to clear its backlog if there is any
	
	while(!USART0_is_idle());
	
	//Check for right device
	//Read Chip-id 
	uint8_t Data = 0;
	
	DEFUALT_ERROR_HANDLER1(BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CHIP_ID, 0, &Data), error_return);
	if(Data != BNO055_ID)
		return BNO055_ERROR_WRONG_DEVICE_ID;
	//sensor defaults to OPR_MODE -> config mode
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), error_return);
	
	//sensor defaults to PWR_MODE -> normal mode
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PWR_MODE, 0, BNO_PWR_MODE_NORMAL), error_return);
	
	//sensor defaults to PAGE_ID -> PAGE0
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PAGE_ID, 0, BNO_PAGE_ID0), error_return);
	
	//Set output units:
	Data =	(0<<7) | //Format = Windows
			(0<<4) | //Temperature = Celsius
			(0<<2) | //Euler = Degrees
			(1<<1) | //Gyro = Rad/s
			(0<<0);  //Accelerometer = m/s^2
	_Delay(1000000);
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_UNIT_SEL, 0, Data), error_return);
	_Delay(10000);
	//sensor defaults to SYS_TRIGGER -> Internal oscillator
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_SYS_TRIGGER, 0, BNO_INTERNAL_OSC), error_return);
	
	if(calibrationNeeded)
	{
		//Switch to Fusion Mode
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, FUSION_MODE_NDOF), error_return);
		
		//Calibrating		
		BNO055_calibrate();
		//Switch to Config Mode
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), error_return);
		
				
		//TODO: write Data to EEPROM
			
		
	}else{
		//TODO: read calibration data from EEPROM
	}
	
	//Set Operation Mode to NDOF (nine degrees of freedom)
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, FUSION_MODE_NDOF), error_return);
	//Initialize data struct to zero
	lastMeasuredData.X = 0;
	lastMeasuredData.Y = 0;
	lastMeasuredData.Z = 0;
	bno_is_busy = false;
	//Initialization finished
	return SUCCESS;
}

StatusCode BNO055_init_non_fusion_mode(uint8_t bno_mode_register)
{
	if(bno_mode_register >= 0x01 && bno_mode_register <= 0x07)
	{
		bno_is_busy = true;
		StatusCode error_return;
		
		DEFUALT_ERROR_HANDLER1(BNOCOM_Init(bno_runtime_success), error_return);
		
		DEFUALT_ERROR_HANDLER1(BNOCOM_register_error_callback(bno_runtime_error), error_return);
		
		//Wait for USART0 to clear its backlog if there is any
		while(!USART0_is_idle());
		
		//Check for right device
		//Read Chip-id
		uint8_t Data = 0;
		
		DEFUALT_ERROR_HANDLER1(BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CHIP_ID, 0, &Data), error_return);
		if(Data != BNO055_ID)
		return BNO055_ERROR_WRONG_DEVICE_ID;
		
		//sensor defaults to OPR_MODE -> config mode
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), error_return);
		
		//sensor defaults to PWR_MODE -> normal mode
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PWR_MODE, 0, BNO_PWR_MODE_NORMAL), error_return);
		
		//sensor defaults to PAGE_ID -> PAGE0
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PAGE_ID, 0, BNO_PAGE_ID0), error_return);
		
		//Set output units:
		Data =	(0<<7) | //Format = Windows
		(0<<4) | //Temperature = Celsius
		(0<<2) | //Euler = Degrees
		(0<<1) | //Gyro = Degrees/s
		(0<<0);  //Accelerometer = m/s^2
		_Delay(1000000);
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_UNIT_SEL, 0, Data), error_return);
		_Delay(10000);
		//sensor defaults to SYS_TRIGGER -> Internal oscillator
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_SYS_TRIGGER, 0, BNO_INTERNAL_OSC), error_return);
		
		//Set Operation Mode to NDOF (nine degrees of freedom)
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, bno_mode_register), error_return);
		//Initialize data struct to zero
		lastMeasuredData.X = 0;
		lastMeasuredData.Y = 0;
		lastMeasuredData.Z = 0;
		bno_is_busy = false;
		//Initialization finished
		return SUCCESS;
	}
	return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
}

StatusCode BNO055_calibrate()
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
		StatusCode readReturn = BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CALIB_STAT, 0, &Data);
		while(readReturn == BNO055_ERROR && Data == BNO_STATUS_BUS_OVER_RUN_ERROR)
		{
			readReturn = BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CALIB_STAT, 0, &Data);
			SerialCOM_put_debug("Bus Error");
			_Delay(840000);
		}
		if(readReturn != SUCCESS && Data != BNO_STATUS_BUS_OVER_RUN_ERROR)
		{
			return readReturn;
		}
		DEFUALT_ERROR_HANDLER(bno_calculate_calibration(Data, &sys, &gyro, &accel, &mag), calib_return);
	}
	bno_is_busy = false;
	return SUCCESS;
}

StatusCode BNO055_get_calibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
	uint8_t Data = 0;
	bno_is_busy = true;
	DEFUALT_ERROR_HANDLER(BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CALIB_STAT, 0, &Data), error_return);
	DEFUALT_ERROR_HANDLER(bno_calculate_calibration(Data, sys, gyro, accel, mag), calib_return);
	bno_is_busy = false;
	return SUCCESS;
}

StatusCode bno_calculate_calibration(uint8_t CalibrationData, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
	if(sys == NULL || gyro == NULL || accel == NULL || mag == NULL)
		return BNO055_ERROR_GOT_NULL_POINTER;
	//Write the data to the proper variables
	*sys = (CalibrationData >> 6) & 0x03;
	*gyro = (CalibrationData >> 4) & 0x03;
	*accel = (CalibrationData >> 2) & 0x03;
	*mag = CalibrationData & 0x03;
	return SUCCESS;
}

StatusCode BNO055_register_data_ready_callback(BNO055_DATA_READY_CALLBACK callback)
{
	if(callback == NULL)
		return BNO055_ERROR_GOT_NULL_POINTER;
	bno_data_ready_callback = callback;
	return SUCCESS;
}

StatusCode BNO055_register_error_callback(BNO055_ERROR_CALLBACK callback)
{
	if(callback == NULL)
		return BNO055_ERROR_GOT_NULL_POINTER;
	bno_error_callback = callback;
	return SUCCESS;
}

void bno_runtime_success(uint8_t* sensorData, uint8_t Length)
{
	if((sensorData == NULL || Length == 0) && bno_error_callback != NULL)
		bno_error_callback(BNO_TRANSMIT_ERROR, BNO055_ERROR_GOT_NULL_POINTER);
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
		BNOCOM_register_read_by_table(bnoReadRegister, 0, 6);
		bno_is_busy = true;
	}
	bno_is_busy = false;
}

bool BNO055_is_busy()
{
	return bno_is_busy;
}

void bno_runtime_error(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code)
{
	if(bno_error_callback != NULL)
		bno_error_callback(Error, Transmit_error_code);
}

StatusCode BNO055_stop_continous_measurement()
{
	bno_measureContinous = false;
	return SUCCESS;
}

BNO055_Data BNO055_get_measurement_data()
{
	return lastMeasuredData;
}

StatusCode BNO055_start_measurement(bool measureContinous, bool triggerCallback, uint8_t bnoMeasurementType)
{
	if(bnoMeasurementType == BNO_REG_ACC_DATA_X || bnoMeasurementType == BNO_REG_MAG_DATA_X 
		|| bnoMeasurementType == BNO_REG_GYR_DATA_X || bnoMeasurementType == BNO_REG_EUL_DATA_X 
		|| bnoMeasurementType == BNO_REG_LIA_DATA_X || bnoMeasurementType == BNO_REG_GRV_DATA_X)
	{
		bno_is_busy = true;
		bno_measureContinous = measureContinous;
		bno_triggerCallback = triggerCallback;
		bnoReadRegister = bnoMeasurementType;
		return BNOCOM_register_read_by_table(bnoReadRegister, 0, 6);
	}
	return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
}