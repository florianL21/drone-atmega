/*
 * SerialCOM.c
 *
 * Created: 12.02.2018 08:58:53
 *  Author: flola
 */ 

#include "SerialCOM.h"

SerialCOM_RECV_CALLBACK SerialCOM_reciveCallBack = NULL;

void message_received(uint8_t* message, uint16_t Length)
{	
	static uint8_t Type = 0;
	if(Length == 3 && message[0] == 0x02)
	{
		Type = message[1];
		UART0_set_receiver_length(message[2]+1);
	} else if(message[Length-1] == 0x03)
	{
		if(SerialCOM_reciveCallBack != NULL)
		{
			SerialCOM_reciveCallBack(message, Type);
		}
		UART0_set_receiver_length(3);
	} else
	{
		UART0_set_receiver_length(3);
		SerialCOM_put_debug("error");
	}
}

StatusCode SerialCOM_register_call_back(SerialCOM_RECV_CALLBACK callback)
{
	if(callback == NULL)
		return SerialCOM_ERROR_GOT_NULL_POINTER;
	SerialCOM_reciveCallBack = callback;
	return SUCCESS;
}

StatusCode SerialCOM_init()
{
	DEFUALT_ERROR_HANDLER(UART0_init(115200, 1), errorReturn);
	DEFUALT_ERROR_HANDLER1(UART0_set_receiver_length(3), errorReturn);
	DEFUALT_ERROR_HANDLER1(UART0_register_received_callback(message_received), errorReturn);
	return SUCCESS;
}

StatusCode SerialCOM_put_message(uint8_t message[], uint8_t Type, uint8_t Length)
{
	uint8_t *transmissionData = malloc((Length+4)*sizeof(uint8_t));
	if(transmissionData == NULL)
	{
		return SerialCOM_ERROR_MALLOC_RETURNED_NULL;
	}
	transmissionData[0] = 0x02;
	transmissionData[1] = Type;
	transmissionData[2] = Length;
	memcpy(&transmissionData[3], message, Length);
	transmissionData[Length+3] = 0x03;
	StatusCode errorReturn = UART0_put_data(transmissionData, Length+4);
	free(transmissionData);
	return errorReturn;
}

StatusCode SerialCOM_force_put_message(uint8_t message[], uint8_t Type, uint8_t Length)
{
	uint8_t *transmissionData = malloc((Length+4)*sizeof(uint8_t));
	if(transmissionData == NULL)
		return SerialCOM_ERROR_MALLOC_RETURNED_NULL;
	if(Length == 0)
		return SerialCOM_ERROR_INVALID_ARGUMENT;
	transmissionData[0] = 0x02;
	transmissionData[1] = Type;
	transmissionData[2] = Length;
	memcpy(&transmissionData[3],message, Length);
	transmissionData[Length+3] = 0x03;
	uart0_put_raw_data(transmissionData, Length+4);
	free(transmissionData);
	return SUCCESS;
}

StatusCode SerialCOM_put_debug(char Text[])
{
	return SerialCOM_put_message(((uint8_t*)Text), 0x00, strlen(Text));
}

StatusCode SerialCOM_put_debug_n(char Text[], uint8_t Length)
{
	return SerialCOM_put_message(((uint8_t*)Text), 0x00, Length);
}

StatusCode SerialCOM_put_error(char Text[])
{
	return SerialCOM_put_message(((uint8_t*)Text), 0x64, strlen(Text));
}

StatusCode SerialCOM_force_put_error(char Text[])
{
	return SerialCOM_force_put_message(((uint8_t*)Text), 0x64, strlen(Text));
}

uint8_t SerialCOM_get_free_space()
{
	return UART0_get_space();
}