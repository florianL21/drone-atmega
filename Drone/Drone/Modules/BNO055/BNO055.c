/*
 * BNO_055.c
 *
 * Created: 27.03.2018 18:47:25
 *  Author: flola
 */ 

#include "BNO055.h"

#define BNO_TRANSMISSION_STARTBYTE					0xAA
#define BNO_TRANSMISSION_WRITE						0x00
#define BNO_TRANSMISSION_READ						0x01
#define BNO_TRANSMISSION_READ_SUCCESS_RESPONSE		0xBB
#define BNO_TRANSMISSION_ACK_RESPONSE				0xEE

#define BNO_TRANS_STATUS_WRITE_SUCCESS				0x01
#define BNO_TRANS_STATUS_READ_FAIL					0x02
#define BNO_TRANS_STATUS_WRITE_FAIL					0x03
#define BNO_TRANS_STATUS_REGMAP_INVALID_ADDRESS		0x04
#define BNO_TRANS_STATUS_REGMAP_WRITE_DISABLED		0x05
#define BNO_TRANS_STATUS_WRONG_START_BYTE			0x06
#define BNO_TRANS_STATUS_BUS_OVER_RUN_ERROR			0x07
#define BNO_TRANS_STATUS_MAX_LENGTH_ERROR			0x08
#define BNO_TRANS_STATUS_MIN_LENGTH_ERROR			0x09
#define BNO_TRANS_STATUS_RECEIVE_CHARACTER_TIMEOUT	0x0A

/*Predefined bno constants*/
#define BNO055_ID				0xA0
#define BNO_CONFIG_MODE			0x00
#define BNO_PWR_MODE_NORMAL		0x00
#define BNO_PAGE_ID0			0x00
#define BNO_PAGE_ID1			0x01
#define BNO_INTERNAL_OSC		0x00
#define FUSION_MODE_NDOF		0x0C

//various settings
#define NUM_OF_RETRYS_ON_ERROR	5
#define DELAY_BEFORE_RETRY		300

ErrorCode bno055_returnError = ERROR_GENERIC;
bool bno055_wait_for_response = false;
uint8_t bno055_continousMeasurementRegister = 0x00;
uint8_t bno055_continousMeasurementLength = 0;
uint8_t* bno055_ReadResponseDestPtr = NULL;
uint8_t bno055_RequestedReadLength = 0;
bool bno055_isReady = true;
bool bno055_is_calibrating = false;
BNO055_DATA_READY_CALLBACK bno_data_ready_callback = NULL;
BNO055_ERROR_CALLBACK bno_error_callback = NULL;

void bno055_data_received_callback(uint8_t* startPtr, uint16_t Length);

typedef struct Quaternion
{
	double w;
	double x;
	double y;
	double z;
}Quaternion;

ErrorCode BNO055_init(BNO_INIT_CALIB PerformCalib)
{
	DEFAULT_ERROR_HANDLER(USART0_init(115200,2), MODULE_BNO055, FUNCTION_init);
	DEFAULT_ERROR_HANDLER(USART0_register_received_callback(bno055_data_received_callback), MODULE_BNO055, FUNCTION_init);
	//start BNO initialisation:
	uint8_t Data = 0;
	DEFAULT_ERROR_HANDLER(BNO055_read_blocking(BNO_REG_CHIP_ID, &Data, 1), MODULE_BNO055, FUNCTION_init);
	//1. check for the right ID:
	if(Data != BNO055_ID)
		return MODULE_BNO055 | FUNCTION_init | ERROR_WRONG_DEVICE_ID;
	//2.sensor defaults to OPR_MODE -> config mode
	Data = BNO_CONFIG_MODE;
	DEFAULT_ERROR_HANDLER(BNO055_write_blocking(BNO_REG_OPR_MODE, &Data, 1), MODULE_BNO055, FUNCTION_init);
	//3. sensor defaults to PWR_MODE -> normal mode
	Data = BNO_PWR_MODE_NORMAL;
	DEFAULT_ERROR_HANDLER(BNO055_write_blocking(BNO_REG_PWR_MODE, &Data, 1), MODULE_BNO055, FUNCTION_init);
	//4. sensor defaults to PAGE_ID -> PAGE0
	Data = BNO_PAGE_ID0;
	DEFAULT_ERROR_HANDLER(BNO055_write_blocking(BNO_REG_PAGE_ID, &Data, 1), MODULE_BNO055, FUNCTION_init);
	
	//Set output units:
	Data =	(0<<7) | //Format = Windows
			(0<<4) | //Temperature = Celsius
			(0<<2) | //Euler = Degrees
			(1<<1) | //Gyro = Rad/s
			(0<<0);  //Accelerometer = m/s^2
	GPT_Delay(200);
	DEFAULT_ERROR_HANDLER(BNO055_write_blocking(BNO_REG_UNIT_SEL, &Data, 1), MODULE_BNO055, FUNCTION_init);
	GPT_Delay(100);
	//sensor defaults to SYS_TRIGGER -> Internal oscillator
	Data = BNO_INTERNAL_OSC;
	DEFAULT_ERROR_HANDLER(BNO055_write_blocking(BNO_REG_SYS_TRIGGER, &Data, 1), MODULE_BNO055, FUNCTION_init);
		
	switch (PerformCalib) //TODO: write a calib routine
	{
	case Force_calibration:
		//switch to fusion mode
		Data = FUSION_MODE_NDOF;
		DEFAULT_ERROR_HANDLER(BNO055_write_blocking(BNO_REG_OPR_MODE, &Data, 1), MODULE_BNO055, FUNCTION_init);
		//Calibrating
		DEFAULT_ERROR_HANDLER(BNO055_calibrate(),MODULE_BNO055,FUNCTION_init);
		//Switch to Config Mode
		Data = BNO_CONFIG_MODE;
		DEFAULT_ERROR_HANDLER(BNO055_write_blocking(BNO_REG_OPR_MODE, &Data, 1), MODULE_BNO055, FUNCTION_init);
	break;
	case Do_not_calibrate:
		
	break;
	case Calibrate_if_necessary:
		
	break;
	default:
		return ERROR_INVALID_ARGUMENT | MODULE_BNO055 | FUNCTION_init;
	break;
	}
	//Set Operation Mode to NDOF (nine degrees of freedom)
	Data = FUSION_MODE_NDOF;
	DEFAULT_ERROR_HANDLER(BNO055_write_blocking(BNO_REG_OPR_MODE, &Data, 1), MODULE_BNO055, FUNCTION_init);
	
	return SUCCESS;
}

bool BNO055_IsCalibrating()
{
	return bno055_is_calibrating;
}

ErrorCode BNO055_register_data_ready_callback(BNO055_DATA_READY_CALLBACK callback)
{
	if(callback == NULL)
		return ERROR_GOT_NULL_POINTER | MODULE_BNO055 | FUNCTION_register_data_ready_callback;
	bno_data_ready_callback = callback;
	return SUCCESS;
}

ErrorCode BNO055_register_error_callback(BNO055_ERROR_CALLBACK callback)
{
	if(callback == NULL)
		return ERROR_GOT_NULL_POINTER | MODULE_BNO055 | FUNCTION_register_error_callback;
	bno_error_callback = callback;
	return SUCCESS;
}

ErrorCode BNO055_calibrate()
{
	bno055_is_calibrating = true;
	DEFAULT_ERROR_HANDLER(BNO055_read(BNO_REG_CALIB_STAT, 1),MODULE_BNO055,FUNCTION_calibrate);
	while (bno055_is_calibrating == true)
	{
		WDT_restart();
		//ErrorHandling_throw(SerialCOM_put_debug("w"));
	}
	return SUCCESS;
}

bool BNO055_IsReady()
{
	return bno055_isReady;
}

BNO055_Data ConvertQuaToYPR(uint8_t* startPtr)
{
	BNO055_Data YPRData;
	int16_t x, y, z, w;
	x = y = z = w = 0;
	Quaternion quat;
	const double scale = (1.0 / (1<<14));
	//double rm[3][3];
	
	w = ((uint16_t)startPtr[0]) | (((uint16_t)startPtr[1]) << 8);
	x = ((uint16_t)startPtr[2]) | (((uint16_t)startPtr[3]) << 8);
	y = ((uint16_t)startPtr[4]) | (((uint16_t)startPtr[5]) << 8);
	z = ((uint16_t)startPtr[6]) | (((uint16_t)startPtr[7]) << 8);
	quat.w = w * scale;
	quat.x = x * scale;
	quat.y = y * scale;
	quat.z = z * scale;
	
	/* Create Roll Pitch Yaw Angles from Quaternions */
	double q2sqr = quat.y * quat.y;
	double t0 = -2.0 * (q2sqr + quat.z * quat.z) + 1.0;
	double t1 = +2.0 * (quat.x * quat.y + quat.w * quat.z);
	double t2 = -2.0 * (quat.x * quat.z - quat.w * quat.y);
	double t3 = +2.0 * (quat.y * quat.z + quat.w * quat.x);
	double t4 = -2.0 * (quat.x * quat.x + q2sqr) + 1.0;

	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;

	YPRData.Pitch = ((float) 57.2958 * asin(t2));
	YPRData.Roll = ((float) 57.2958 * atan2(t3, t4));
	YPRData.Yaw = ((float) 57.2958 * atan2(t1, t0));
	return YPRData;
}

ErrorCode BNO055_read_blocking(uint8_t RegisterAddress, uint8_t dataToRead[], uint8_t DataLength)
{
	static uint8_t numOfTries = 0;
	bno055_returnError = ERROR_GENERIC;
	if(dataToRead == NULL)
		return ERROR_GOT_NULL_POINTER | MODULE_BNO055 | FUNCTION_read_blocking;
	if(DataLength == 0)
		return ERROR_ARGUMENT_OUT_OF_RANGE | MODULE_BNO055 | FUNCTION_read_blocking;
	while(bno055_returnError != SUCCESS && numOfTries <= NUM_OF_RETRYS_ON_ERROR)
	{
		if(numOfTries != 0)//if this runs it is a retry:
		{
			//SerialCOM_put_error("read_retry");//TODO: proper error handling
		}
		bno055_wait_for_response = true;
		bno055_returnError = ERROR_GENERIC;
		bno055_ReadResponseDestPtr = dataToRead;
		DEFAULT_ERROR_HANDLER(BNO055_read(RegisterAddress, DataLength), MODULE_BNO055, FUNCTION_read_blocking);
		float startCommandTimestamp = GPT_GetPreciseTime();
		while (bno055_wait_for_response == true)
		{
			if(GPT_GetPreciseTime() - startCommandTimestamp >= BNO_TRANSMISSION_TIMEOUT_MS)
			{
				bno055_returnError = ERROR_BNO_READ_TIMEOUT | MODULE_BNO055 | FUNCTION_read_blocking;
				break;
			}
		}
		numOfTries++;
		GPT_Delay(DELAY_BEFORE_RETRY);
	}
	numOfTries = 0;
	//Data was copied in the callback.
	return ErrorHandling_set_top_level(bno055_returnError, MODULE_BNO055, FUNCTION_read_blocking);
}

ErrorCode BNO055_write_blocking(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength)
{
	static uint8_t numOfTries = 0;
	bno055_returnError = ERROR_GENERIC;
	while(bno055_returnError != SUCCESS && numOfTries <= NUM_OF_RETRYS_ON_ERROR)
	{
		if(numOfTries != 0)//if this runs it is a retry:
		{
			//SerialCOM_put_error("write_retry");//TODO: proper error handling
		}
		bno055_wait_for_response = true;
		bno055_returnError = ERROR_GENERIC;
		DEFAULT_ERROR_HANDLER(BNO055_write(RegisterAddress, dataToWrite, DataLength), MODULE_BNO055, FUNCTION_write_blocking);
		float startCommandTimestamp = GPT_GetPreciseTime();
		while (bno055_wait_for_response == true)
		{
			if(GPT_GetPreciseTime() - startCommandTimestamp >= BNO_TRANSMISSION_TIMEOUT_MS)
			{
				bno055_returnError = ERROR_BNO_WRITE_TIMEOUT | MODULE_BNO055 | FUNCTION_read_blocking;
				break;
			}
		}
		numOfTries++;
		GPT_Delay(DELAY_BEFORE_RETRY);
	}
	numOfTries = 0;
	return ErrorHandling_set_top_level(bno055_returnError,MODULE_BNO055,FUNCTION_write_blocking);
}

ErrorCode BNO055_write(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength)
{
	//checking for wrong arguments:
	if(BNO055_IsReady() != true)
		return ERROR_NOT_READY_FOR_OPERATION | MODULE_BNO055 | FUNCTION_write;
	if(DataLength == 0)
		return ERROR_ARGUMENT_OUT_OF_RANGE | MODULE_BNO055 | FUNCTION_write;
	if(dataToWrite == NULL)
		return ERROR_GOT_NULL_POINTER | MODULE_BNO055 | FUNCTION_write;
	uint8_t* sendBuffer = malloc((DataLength + 4) * sizeof(uint8_t));
	if(sendBuffer == NULL)
		return ERROR_MALLOC_RETURNED_NULL | MODULE_BNO055 | FUNCTION_write;
	//writing send message to buffer
	sendBuffer[0] = BNO_TRANSMISSION_STARTBYTE;
	sendBuffer[1] = BNO_TRANSMISSION_WRITE;
	sendBuffer[2] = RegisterAddress;
	sendBuffer[3] = DataLength;
	memcpy(&sendBuffer[4], dataToWrite, DataLength);
	//send the data
	DEFAULT_ERROR_HANDLER(USART0_put_data(sendBuffer, DataLength + 4), MODULE_BNO055, FUNCTION_write);
	bno055_isReady = false;
	free(sendBuffer);
	return SUCCESS;
}

ErrorCode BNO055_read(uint8_t RegisterAddress, uint8_t DataLength)
{
	//check if measurement in progress:
	if(BNO055_IsReady() != true)
		return ERROR_NOT_READY_FOR_OPERATION | MODULE_BNO055 | FUNCTION_write;
	//checking for wrong arguments:
	if(DataLength == 0)
		return ERROR_ARGUMENT_OUT_OF_RANGE | MODULE_BNO055 | FUNCTION_read;
	uint8_t readBuffer[4];
	//writing send message to buffer
	readBuffer[0] = BNO_TRANSMISSION_STARTBYTE;
	readBuffer[1] = BNO_TRANSMISSION_READ;
	readBuffer[2] = RegisterAddress;
	readBuffer[3] = DataLength;
	//send the data
	DEFAULT_ERROR_HANDLER(USART0_put_data(readBuffer, 4), MODULE_BNO055, FUNCTION_read);
	bno055_RequestedReadLength = DataLength;
	bno055_isReady = false;
	return SUCCESS;
}

ErrorCode bno055_translate_bno_transmission_errors(uint8_t bnoTransError)
{
	switch(bnoTransError)
	{
		case BNO_TRANS_STATUS_WRITE_SUCCESS:
			return SUCCESS;
		break;
		case BNO_TRANS_STATUS_READ_FAIL:
			return ERROR_READ_FAIL;
		break;
		case BNO_TRANS_STATUS_WRITE_FAIL:
			return ERROR_WRITE_FAIL;
		break;
		case BNO_TRANS_STATUS_REGMAP_INVALID_ADDRESS:
			return ERROR_REGMAP_INVALID_ADDRESS;
		break;
		case BNO_TRANS_STATUS_REGMAP_WRITE_DISABLED:
			return ERROR_REGMAP_WRITE_DISABLED;
		break;
		case BNO_TRANS_STATUS_WRONG_START_BYTE:
			return ERROR_WRONG_START_BYTE;
		break;
		case BNO_TRANS_STATUS_BUS_OVER_RUN_ERROR:
			return ERROR_BUS_OVER_RUN;
		break;
		case BNO_TRANS_STATUS_MAX_LENGTH_ERROR:
			return ERROR_MAX_LENGTH;
		break;
		case BNO_TRANS_STATUS_MIN_LENGTH_ERROR:
			return ERROR_MIN_LENGTH;
		break;
		case BNO_TRANS_STATUS_RECEIVE_CHARACTER_TIMEOUT:
			return ERROR_RECEIVE_CHARACTER_TIMEOUT;
		break;
		default:
			return ERROR_STATUS_BYTE_UNKNOWN;
		break;
	}
}

void bno055_data_received_callback(uint8_t* startPtr, uint16_t Length)
{
	static uint8_t recState = 0;
	if(Length == 2 && recState == 0 && startPtr[0] == BNO_TRANSMISSION_ACK_RESPONSE)
	{
		if(bno055_wait_for_response == true)		//write initialized by a blocking function
		{
			bno055_returnError = bno055_translate_bno_transmission_errors(startPtr[1]);
			bno055_wait_for_response = false;
		}else if(bno_error_callback != NULL && startPtr[1] != BNO_TRANS_STATUS_WRITE_SUCCESS) //write initialised by a start mesurement function
		{
			bno_error_callback(ErrorHandling_set_top_level(bno055_translate_bno_transmission_errors(startPtr[1]), MODULE_BNO055, FUNCTION_data_received_callback));
		}
	}else if(Length == 2 && recState == 0 && startPtr[0] == BNO_TRANSMISSION_READ_SUCCESS_RESPONSE)
	{
		recState = 1;
		ErrorCode Uart_return = USART0_set_receiver_length(startPtr[1]);
		if(bno055_wait_for_response == true && Uart_return != SUCCESS)
		{
			bno055_returnError = ErrorHandling_set_top_level(Uart_return, MODULE_BNO055, FUNCTION_data_received_callback);
		}else if(bno055_wait_for_response == false && Uart_return != SUCCESS && bno_error_callback != NULL)
		{
			bno_error_callback(ErrorHandling_set_top_level(Uart_return, MODULE_BNO055, FUNCTION_data_received_callback));
		}
	} else if(recState == 1)
	{
		
		if(bno055_wait_for_response == true)
		{
			if(Length != bno055_RequestedReadLength)
			{
				bno055_returnError = ERROR_LENGTH_MISSMATCH | MODULE_BNO055 | FUNCTION_data_received_callback;
				recState = 0;
				bno055_isReady = true;
				bno055_wait_for_response = false;
				return;
			}
			//copy data to its new dest.
			memcpy(bno055_ReadResponseDestPtr, startPtr, Length);
			//this frees the data --> thus should be called after copying!!
			ErrorCode Uart_return = USART0_set_receiver_length(2);
			if(Uart_return != SUCCESS)
			{
				bno055_returnError = ErrorHandling_set_top_level(Uart_return,MODULE_BNO055,FUNCTION_data_received_callback);
			}
			bno055_wait_for_response = false;
			bno055_returnError = SUCCESS;
		} else
		{
			if(Length != bno055_RequestedReadLength && bno_error_callback != NULL)
			{
				bno_error_callback(ERROR_LENGTH_MISSMATCH | MODULE_BNO055 | FUNCTION_data_received_callback);
				//make the BNO ready again and exit the function.
				recState = 0;
				bno055_isReady = true;
				return;
			}
			//trigger callback to process data
			if(bno_data_ready_callback != NULL)
				bno_data_ready_callback(startPtr, Length);
			//For calibration: check if fully calibrated, if not start the next measurement
			if(bno055_is_calibrating == true)
			{
				if(Length == 1)
				{
					if((startPtr[0] & 0xC0) != 0xC0)
					{
						ErrorCode BNO_return = BNO055_read(BNO_REG_CALIB_STAT,1);
						if(BNO_return != SUCCESS && bno_error_callback != NULL)
						{
							bno_error_callback(ErrorHandling_set_top_level(BNO_return,MODULE_BNO055,FUNCTION_data_received_callback));
						}
					} else
					{
						bno055_is_calibrating = false;
					}
				}else if(bno_error_callback != NULL)
				{
					bno_error_callback(ERROR_INVALID_ARGUMENT | MODULE_BNO055 | FUNCTION_data_received_callback);
				}
			}
			//this frees the data --> thus should be called after copying!!
			ErrorCode Uart_return = USART0_set_receiver_length(2);
			if(Uart_return != SUCCESS && bno_error_callback != NULL)
				bno_error_callback(ErrorHandling_set_top_level(Uart_return, MODULE_BNO055, FUNCTION_data_received_callback));
		}
		recState = 0;
	}
	bno055_isReady = true;
}