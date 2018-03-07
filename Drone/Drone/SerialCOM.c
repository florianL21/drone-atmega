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
		SerialCOM_put_Command('A', 0x06);	//ACK
	} else
	{
		UART0_set_receiver_length(3);
		SerialCOM_put_debug("error");
		SerialCOM_put_Command('N', 0x06);	//NACK
	}
}

ErrorCode SerialCOM_register_call_back(SerialCOM_RECV_CALLBACK callback)
{
	if(callback == NULL)
		return MODULE_SERIALCOM | FUNCTION_register_call_back | ERROR_GOT_NULL_POINTER;
	SerialCOM_reciveCallBack = callback;
	return SUCCESS;
}

ErrorCode SerialCOM_init()
{
	DEFAULT_ERROR_HANDLER(UART0_init(115200, 1));
	DEFAULT_ERROR_HANDLER(UART0_set_receiver_length(3));
	DEFAULT_ERROR_HANDLER(UART0_register_received_callback(message_received));
	return SUCCESS;
}

ErrorCode SerialCOM_put_message(uint8_t message[], uint8_t Type, uint8_t Length)
{
	uint8_t *transmissionData = malloc((Length+4)*sizeof(uint8_t));
	if(transmissionData == NULL)
	{
		return MODULE_SERIALCOM | FUNCTION_put_message | ERROR_MALLOC_RETURNED_NULL;
	}
	transmissionData[0] = 0x02;
	transmissionData[1] = Type;
	transmissionData[2] = Length;
	memcpy(&transmissionData[3], message, Length);
	transmissionData[Length+3] = 0x03;
	ErrorCode errorReturn = UART0_put_data(transmissionData, Length+4);
	free(transmissionData);
	return errorReturn;
}

ErrorCode SerialCOM_force_put_message(uint8_t message[], uint8_t Type, uint8_t Length)
{
	uint8_t *transmissionData = malloc((Length+4)*sizeof(uint8_t));
	if(transmissionData == NULL)
		return MODULE_SERIALCOM | FUNCTION_force_put_message | ERROR_MALLOC_RETURNED_NULL;
	if(Length == 0)
		return MODULE_SERIALCOM | FUNCTION_force_put_message | ERROR_INVALID_ARGUMENT;
	transmissionData[0] = 0x02;
	transmissionData[1] = Type;
	transmissionData[2] = Length;
	memcpy(&transmissionData[3],message, Length);
	transmissionData[Length+3] = 0x03;
	uart0_put_raw_data(transmissionData, Length+4);
	free(transmissionData);
	return SUCCESS;
}

ErrorCode SerialCOM_put_Command(char CommandChar, uint8_t Type)
{
	return SerialCOM_put_message(((uint8_t*)&CommandChar), Type, 1);
}

ErrorCode SerialCOM_put_debug(char Text[])
{
	return SerialCOM_put_message(((uint8_t*)Text), 0x00, strlen(Text));
}

ErrorCode SerialCOM_put_debug_n(char Text[], uint8_t Length)
{
	return SerialCOM_put_message(((uint8_t*)Text), 0x00, Length);
}

ErrorCode SerialCOM_put_error(char Text[])
{
	return SerialCOM_put_message(((uint8_t*)Text), 0x64, strlen(Text));
}

ErrorCode SerialCOM_force_put_error(char Text[])
{
	return SerialCOM_force_put_message(((uint8_t*)Text), 0x64, strlen(Text));
}

uint8_t SerialCOM_get_free_space()
{
	return UART0_get_space();
}

ErrorCode SerialCOM_print_debug(const char *fmt, ...)
{
	char buffer[SERIALCOM_MAX_PRINT_CHARS];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);
	ErrorCode rt = SerialCOM_put_debug(buffer);
	return rt;
}

ErrorCode SerialCOM_print_error(const char *fmt, ...)
{
	char buffer[SERIALCOM_MAX_PRINT_CHARS];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);
	ErrorCode rt = SerialCOM_put_error(buffer);
	return rt;
}