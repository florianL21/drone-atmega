/*
 * BNO055.c
 *
 * Created: 23.11.2017 21:44:11
 *  Author: flola
 */ 

#include "BNO055.h"


/*Predefined bno constants*/
#define BNO055_ID			0xA0
#define BNO_CONFIG_MODE		0x00
#define BNO_PWR_MODE_NORMAL	0x00
#define BNO_PAGE_ID0		0x00
#define BNO_INTERNAL_OSC	0x00
#define FUSION_MODE_NDOF	0x0C


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
BNO055_eulerData lastMeasuredData;
bool bno_is_busy = false;
bno_requested_measurement bno_last_requested_measurement = bno_none;




StatusCode BNO055_init(bool calibrationNeeded)
{
	bno_is_busy = true;
	StatusCode error_return;
	
	DEFUALT_ERROR_HANDLER1(BNOCOM_Init(bno_runtime_success), error_return);
	
	DEFUALT_ERROR_HANDLER1(BNOCOM_register_error_callback(bno_runtime_error), error_return);
	//UART0_puts("INIT\n\r");
	//Wait for USART0 to clear its backlog if there is any
	
	while(!USART0_is_idle());
	
	//Check for right device
	//Read Chip-id 
	uint8_t Data = 0;
	
	DEFUALT_ERROR_HANDLER1(BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CHIP_ID, 0, &Data), error_return);
	UART0_put_int(Data);
	//UART0_puts("BNO ID\n\r");
	if(Data != BNO055_ID)
		return BNO055_ERROR_WRONG_DEVICE_ID;
	/*//sensor defaults to OPR_MODE -> config mode
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), error_return);
	*/
	/*//sensor defaults to PWR_MODE -> normal mode
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PWR_MODE, 0, BNO_PWR_MODE_NORMAL), error_return);
	*/
	/*//sensor defaults to PAGE_ID -> PAGE0
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PAGE_ID, 0, BNO_PAGE_ID0), error_return);
	*/
	UART0_puts("1");
	//Set output units:
	Data =	(0<<7) | //Orientation = Windows
			(0<<4) | //Temperature = Celsius
			(0<<2) | //Euler = Degrees
			(1<<1) | //Gyro = Rads
			(0<<0);  //Accelerometer = m/s^2
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_UNIT_SEL, 0, Data), error_return);
	_Delay(10000);
	UART0_puts("2");
	/*//sensor defaults to SYS_TRIGGER -> Internal oscillator
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_SYS_TRIGGER, 0, BNO_INTERNAL_OSC), error_return);
	*/
	if(calibrationNeeded)
	{
		//Switch to Fusion Mode
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, FUSION_MODE_NDOF), error_return);
		
		//Calibrating		
		BNO055_calibrate();
		//Switch to Config Mode
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), error_return);
		
		/*//read Calibration data
		uint8_t dataLength = 22;
		uint8_t CalibData[22] = {0};
		DEFUALT_ERROR_HANDLER1(BNOCOM_read_and_wait_for_response(BNO_REG_ACC_OFFSET_X, 0, CalibData, &dataLength), error_return);
		
		//write Data to EEPROM
		for(int i=0; i<=(CALIB_DATA_END-CALIB_DATA_START); i++)
			EEPROM_write(CALIB_DATA_START+i, CalibData[i]);*/
			
		
	}else{
		/*//read calibration data from EEPROM
		uint8_t CalibData[22] = {0};
		for(int i=0; i<=(CALIB_DATA_END-CALIB_DATA_START); i++)
		{
			CalibData[i] = EEPROM_read(CALIB_DATA_START+i);
		}
	
		//switch to config mode
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE), error_return);
		
		//write calibration data
		DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response(BNO_REG_ACC_OFFSET_X, 0, CalibData, 22), error_return);
		*/
	}
	
	//Set Operation Mode to NDOF (nine degrees of freedom)
	DEFUALT_ERROR_HANDLER1(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_OPR_MODE, 0, FUSION_MODE_NDOF), error_return);
	UART0_puts("3");
	//Initialize data struct to zero
	lastMeasuredData.roll = 0;
	lastMeasuredData.pitch = 0;
	lastMeasuredData.heading = 0;
	bno_is_busy = false;
	//Initialization finished
	return SUCCESS;
}

StatusCode BNO055_calibrate()
{
	uint8_t Data = 0;
	uint8_t sys = 0;
	uint8_t gyro = 0;
	uint8_t accel = 0;
	uint8_t mag = 0;
	bno_is_busy = true;
	bno_last_requested_measurement = bno_calibration_in_progress;
	while(sys != 3 || gyro != 3 || accel != 3 || mag != 3)
	{
		DEFUALT_ERROR_HANDLER(BNOCOM_read_and_wait_for_response_1byte(BNO_REG_CALIB_STAT, 0, &Data), error_return);

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
	if(Length == 6 && bno_last_requested_measurement == bno_euler_measurement) // Euler Data
	{
		int16_t mPitch = 0;
		int16_t mRoll = 0;
		int16_t mHeading = 0;
		
		//Convert the uint8_t datasets to int16_t
		mHeading += (sensorData[1] << 8) + sensorData[0];
		mRoll += (sensorData[3] << 8) + sensorData[2];
		mPitch += (sensorData[5] << 8) + sensorData[4];
			
		//Convert to Degrees
		//1 Degree = 16 LSB	
		lastMeasuredData.heading = (float) mHeading / 16.0;
		lastMeasuredData.roll = (float) mRoll / 16.0;
		lastMeasuredData.pitch = (float) mPitch / 16.0;
	}

	if(bno_data_ready_callback != NULL && bno_triggerCallback == true)
		bno_data_ready_callback();
	if(bno_measureContinous == true)
		BNO055_start_euler_measurement(bno_measureContinous, bno_triggerCallback);
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

BNO055_eulerData BNO055_get_euler_measurement_data()
{
	return lastMeasuredData;
}

StatusCode BNO055_start_euler_measurement(bool measureContinous, bool triggerCallback)
{
	bno_is_busy = true;
	bno_last_requested_measurement = bno_euler_measurement;
	bno_measureContinous = measureContinous;
	bno_triggerCallback = triggerCallback;
	return BNOCOM_register_read_by_table(BNO_REG_EUL_DATA_X, 0, 6);
}