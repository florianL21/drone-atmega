/*
 * BNO055.c
 *
 * Created: 19.11.2017 12:44:02
 *  Author: flola
 */ 

#include "BNO055.h"


/* BNO Status Bytes: */
#define START_BYTE			0xAA
#define WRITE_BYTE			0x00
#define READ_BYTE			0x01
#define ACK_OR_ERROR_BYTE	0xEE
#define	READ_SUCCESS_BYTE	0xBB


typedef enum {
	BNO_REC_STATE_IDLE			= 0,
	BNO_REC_STATE_READ_SUCCESS	= 1,
	BNO_REC_STATE_ERROR			= 2,
	
} BNO_recvStates;






bool bnoIsIdle = true;
uint8_t _rec_length = 0;
BNO_ERROR_CALLBACK _error_callback = NULL;
BNO_READ_SUCCESS_CALLBACK _read_success_callback = NULL;
BNO_recvStates _rec_states = BNO_REC_STATE_IDLE;

void _response_received(uint8_t* Data,uint16_t Length);

ERROR_CODE BNO055_Init(BNO_READ_SUCCESS_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return BNO055_ERROR_GOT_NULL_POINTER;
	}
	DEFUALT_ERROR_HANDLER(USART0_init(115200, 2),error_return1);
	DEFUALT_ERROR_HANDLER(USART0_register_received_callback(_response_received),error_return2);
	_read_success_callback = callBack;
	return SUCCESS;
}

ERROR_CODE BNO055_register_error_callback(BNO_ERROR_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return BNO055_ERROR_GOT_NULL_POINTER;
	}
	_error_callback = callBack;
	return SUCCESS;
}

void _response_received(uint8_t* Data, uint16_t Length)
{
	ERROR_CODE USART_return;
	switch(_rec_states)
	{
		case BNO_REC_STATE_IDLE:
			if(Length == 2 && Data[0] == ACK_OR_ERROR_BYTE)
			{
				if(Data[2] != BNO_STATUS_WRITE_SUCCESS && _error_callback != NULL)
				{
					_error_callback(Data[1], SUCCESS);
				}
				bnoIsIdle = true;
			}
			else if(Length == 2 && Data[0] == READ_SUCCESS_BYTE)
			{
				_rec_length = Data[1];
				USART_return = USART0_set_receiver_length(_rec_length);
				if(USART_return != SUCCESS)
				{
					_error_callback(BNO_TRANSMIT_ERROR, USART_return);
					return;
				}
				_rec_states = BNO_REC_STATE_READ_SUCCESS;
			}else
			{
				if(_error_callback != NULL)
				{
					_error_callback(BNO_TRANSMIT_ERROR, BNO055_ERROR_ARGUMENT_OUT_OF_RANGE);
				}
				bnoIsIdle = true;
			}
		break;
		
		case BNO_REC_STATE_READ_SUCCESS:
			if(_read_success_callback != NULL)
			{
				_read_success_callback(Data, _rec_length);
			}
			USART_return = USART0_set_receiver_length(2);
			if(USART_return != SUCCESS)
			{
				_error_callback(BNO_TRANSMIT_ERROR, USART_return);
				_rec_states = BNO_REC_STATE_IDLE;
				return;
			}
			bnoIsIdle = true;
			_rec_states = BNO_REC_STATE_IDLE;
		break;
		
		default:
			if(_error_callback != NULL)
			{
				_error_callback(BNO_TRANSMIT_ERROR, ERROR_FATAL);
			}
			bnoIsIdle = true;
		break;
	}
}

bool BNO055_is_idle()
{
	return bnoIsIdle;
}

ERROR_CODE BNO055_register_write(uint8_t Register, uint8_t Length, uint8_t* Data)
{
	if(!BNO055_is_idle() || !USART0_has_space())
	{
		return BNO055_ERROR_NOT_READY_FOR_OPERATION;
	}
	uint8_t* Message = malloc((Length+4)*sizeof(uint8_t));
	if(Message == NULL)
	{
		return BNO055_ERROR_MALLOC_RETURNED_NULL;
	}
	Message[0] = START_BYTE;
	Message[1] = WRITE_BYTE;
	Message[2] = Register;
	Message[3] = Length;
	for (uint8_t i = 0; i < Length; i++)
	{
		Message[i + 3] = Data[i];
	}
	ERROR_CODE USART_return = USART0_put_data(Message, Length + 4);
	if(USART_return == SUCCESS)
	{
		DEFUALT_ERROR_HANDLER(USART0_set_receiver_length(2),error_return);
		bnoIsIdle = false;
		free(Message);
		return SUCCESS;
	}else
	{
		free(Message);
		return USART_return;
	}
}

ERROR_CODE BNO055_register_read(uint8_t Register, uint8_t Length)
{
	if(!BNO055_is_idle() || !USART0_has_space())
	{
		return BNO055_ERROR_NOT_READY_FOR_OPERATION;
	}
	uint8_t Message[4];
	Message[0] = START_BYTE;
	Message[1] = READ_BYTE;
	Message[2] = Register;
	Message[3] = Length;
	DEFUALT_ERROR_HANDLER(USART0_put_data(Message, 4),error_return1);
	DEFUALT_ERROR_HANDLER(USART0_set_receiver_length(2),error_return2);
	bnoIsIdle = false;

	return SUCCESS;
}