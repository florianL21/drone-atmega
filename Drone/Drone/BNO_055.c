/*
 * BNO_055.c
 *
 * Created: 27.03.2018 18:47:25
 *  Author: flola
 */ 

#include "BNO_055.h"

#define BNO_TRANSMISSION_STARTBYTE				0xAA
#define BNO_TRANSMISSION_WRITE					0x00
#define BNO_TRANSMISSION_READ					0x01
#define BNO_TRANSMISSION_SUCCESS_RESPONSE		0xBB
#define BNO_TRANSMISSION_ERROR_RESPONSE			0xEE

void bno055_data_received_callback(uint8_t* startPtr, uint16_t Length);

ErrorCode BNO055_init()
{
	DEFAULT_ERROR_HANDLER(USART0_init(115200,2), MODULE_BNO055, FUNCTION_Init);
	DEFAULT_ERROR_HANDLER(USART0_register_received_callback(bno055_data_received_callback), MODULE_BNO055, FUNCTION_Init);
	return SUCCESS;
}


ErrorCode BNO055_read_blocking(uint8_t RegisterAddress, uint8_t dataToRead[], uint8_t* DataLength)
{
	return SUCCESS;
}

ErrorCode BNO055_read(uint8_t RegisterAddress, uint8_t dataToRead[], uint8_t* DataLength)
{
	return SUCCESS;
}

ErrorCode BNO055_write_blocking(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength)
{
	DEFAULT_ERROR_HANDLER(BNO, MODULE_BNO055, FUNCTION_write_blocking);
	return SUCCESS;
}

ErrorCode BNO055_write(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength)
{
	//checking for wrong arguments:
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
	free(sendBuffer);
	return SUCCESS;
}

ErrorCode bno055_send_command(uint8_t RegisterAddress, uint8_t dataToWrite[], uint8_t DataLength)
{
	return SUCCESS;
}

void bno055_data_received_callback(uint8_t* startPtr, uint16_t Length)
{
	
}