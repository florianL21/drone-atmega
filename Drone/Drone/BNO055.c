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

void BNO055_Init(BNO_READ_SUCCESS_CALLBACK callBack)
{
	USART0_init(115200,2);
	USART0_register_received_callback(_response_received);
	//USART0_set_receiver_length(2);
	_read_success_callback = callBack;
}

void BNO055_register_error_callback(BNO_ERROR_CALLBACK callBack)
{
	_error_callback = callBack;
}

void _response_received(uint8_t* Data, uint16_t Length)
{
	uint8_t* recData;
	switch(_rec_states)
	{
		case BNO_REC_STATE_IDLE:
			if(Length == 2 && Data[0] == ACK_OR_ERROR_BYTE)
			{
				if(Data[2] != BNO_STATUS_WRITE_SUCCESS && _error_callback != NULL)
				{
					_error_callback(Data[1]);
				}
				bnoIsIdle = true;
			}
			else if(Length == 2 && Data[0] == READ_SUCCESS_BYTE)
			{
				_rec_length = Data[1];
				USART0_set_receiver_length(_rec_length);
				_rec_states = BNO_REC_STATE_READ_SUCCESS;
			}else
			{
				if(_error_callback != NULL)
				{
					_error_callback(BNO_TRANSMIT_ERROR);
				}
				bnoIsIdle = true;
			}
		break;
		
		case BNO_REC_STATE_READ_SUCCESS:
			if(_read_success_callback != NULL)
			{
				_read_success_callback(Data, _rec_length);
			}
			USART0_set_receiver_length(2);
			bnoIsIdle = true;
			_rec_states = BNO_REC_STATE_IDLE;
		break;
		
		default:
			if(_error_callback != NULL)
			{
				_error_callback(BNO_TRANSMIT_ERROR);
			}
			bnoIsIdle = true;
		break;
	}
}

bool BNO055_is_idle()
{
	return bnoIsIdle;
}

bool BNO055_register_write(uint8_t Register, uint8_t Length, uint8_t* Data)
{
	if(!bnoIsIdle)
	{
		return false;
	}
	if(!USART0_has_space())
	{
		return false;
	}
	uint8_t* Message = malloc((Length+4)*sizeof(uint8_t));
	if(Message == NULL)
	{
		return false;
	}
	Message[0] = START_BYTE;
	Message[1] = WRITE_BYTE;
	Message[2] = Register;
	Message[3] = Length;
	for (uint8_t i = 0; i < Length; i++)
	{
		Message[i + 3] = Data[i];
	}
	if(USART0_put_data(Message, Length + 4))
	{
		USART0_set_receiver_length(2);
		bnoIsIdle = false;
		free(Message);
		return true;
	}
	free(Message);
	return false;
}

bool BNO055_register_read(uint8_t Register, uint8_t Length)
{
	if(!bnoIsIdle)
	{
		return false;
	}
	if(!USART0_has_space())
	{
		return false;
	}
	uint8_t Message[4];
	Message[0] = START_BYTE;
	Message[1] = READ_BYTE;
	Message[2] = Register;
	Message[3] = Length;
	if(USART0_put_data(Message, 4))
	{
		USART0_set_receiver_length(2);
		bnoIsIdle = false;
		return true;
	}
	return false;
}