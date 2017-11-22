/*
 * BNO055.c
 *
 * Created: 19.11.2017 12:44:02
 *  Author: flola
 */ 

#include "BNO055.h"
#include "BNO055_reg_table.h"


/* BNO Status Bytes: */
#define START_BYTE			0xAA
#define WRITE_BYTE			0x00
#define READ_BYTE			0x01
#define ACK_OR_ERROR_BYTE	0xEE
#define	READ_SUCCESS_BYTE	0xBB

/*Predefined bno constants*/
#define BNO055_ID			0xA0


typedef enum {
	BNO_REC_STATE_IDLE			= 0,
	BNO_REC_STATE_READ_SUCCESS	= 1,
	BNO_REC_STATE_ERROR			= 2,
	
} BNO_recvStates;



bool bno_IsIdle = true;
uint8_t bno_rec_length = 0;
BNO_ERROR_CALLBACK bno_error_callback = NULL;
BNO_READ_SUCCESS_CALLBACK bno_read_success_callback = NULL;
BNO_recvStates bno_rec_states = BNO_REC_STATE_IDLE;

void bno_response_received(uint8_t* Data,uint16_t Length);
bool bno_register_validation_check(uint8_t Register, uint8_t Length, uint8_t read_write);

/***************************************************************************************************
* BNO Top Level interaction:
****************************************************************************************************/

uint8_t bno_init_response_value = 0x00;
bool bno_init_waiting_for_response = false;
StatusCode bno_init_response_status = SUCCESS;

void bno_init_success(uint8_t* Data, uint8_t Length)
{
	bno_init_waiting_for_response = false;
	bno_init_response_value = Data[0];
	bno_init_response_status = SUCCESS;
}

void bno_init_error(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code)
{
	bno_init_waiting_for_response = false;
	bno_init_response_status = BNO055_ERROR_INVALID_ARGUMENT;
}



StatusCode BNO055_Setup(BNO_READ_SUCCESS_CALLBACK callBack, bool Init_in_blocking_mode)
{
	DEFUALT_ERROR_HANDLER(BNO055_Init(bno_init_success), error_return);
	DEFUALT_ERROR_HANDLER(BNO055_register_error_callback(bno_init_error), error_return1);
	

	if(Init_in_blocking_mode)
	{
		while(!USART0_is_idle());
		//Check for right device
		//Read Chip-id 
		
		bno_init_waiting_for_response = true;
		BNO055_register_read(BNO055_reg_table0[BNO_REG][BNO_REG_CHIP_ID], BNO055_reg_table0[BNO_REG_LEN][BNO_REG_CHIP_ID]);
		while(bno_init_waiting_for_response);	//wait for transmission response
		if(bno_init_response_value != BNO055_ID)
		{
			return BNO055_ERROR_WRONG_DEVICE_ID;
		}
		//BNO055_register_write(BNO055_reg_table0[BNO_REG][BNO_REG_OPR_MODE], BNO055_reg_table0[BNO_REG_LEN][BNO_REG_OPR_MODE]);

	}




	DEFUALT_ERROR_HANDLER(BNO055_register_success_callback(callBack), error_returnX);
	return SUCCESS;
}









/***************************************************************************************************
* BNO Communication Level:
****************************************************************************************************/

bool bno055_register_is_valid(uint8_t Register, uint8_t Length, bool read_write)
{
	//check the PAGE1 registers first, because they are fewer and thus its faster
	for(uint8_t regCounter = 0; regCounter < BNO_NUM_REG_ADDRESSES1; regCounter++)
	{
		if(Register == BNO055_reg_table1[BNO_REG][regCounter] && Length == BNO055_reg_table1[BNO_REG_LEN][regCounter] && 
			(BNO055_reg_table1[BNO_REG_RW_SATUS][regCounter] == BNO_REG_READ_AND_WRITE || read_write == BNO055_reg_table1[BNO_REG_RW_SATUS][regCounter]))
		{
			return true;
		}
	}
	//check the PAGE0 registers second, because now that we know that the register isn't one of PAGE1 we can use a more efficient algorithm for the remaining registers
	for(uint8_t regCounter = 0; regCounter < BNO_NUM_REG_ADDRESSES0; regCounter++)
	{
		if(Register == BNO055_reg_table0[BNO_REG][regCounter])
		{
			if(Length != BNO055_reg_table0[BNO_REG_LEN][regCounter] || (read_write != BNO055_reg_table0[BNO_REG_RW_SATUS][regCounter] && BNO055_reg_table1[BNO_REG_RW_SATUS][regCounter] != BNO_REG_READ_AND_WRITE))
				return false;
			else 
				return true;
		}
	}
	return false;
}

StatusCode BNO055_Init(BNO_READ_SUCCESS_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return BNO055_ERROR_GOT_NULL_POINTER;
	}
	DEFUALT_ERROR_HANDLER(USART0_init(115200, 2),error_return1);
	DEFUALT_ERROR_HANDLER(USART0_register_received_callback(bno_response_received),error_return2);
	bno_read_success_callback = callBack;
	return SUCCESS;
}

StatusCode BNO055_register_error_callback(BNO_ERROR_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return BNO055_ERROR_GOT_NULL_POINTER;
	}
	bno_error_callback = callBack;
	return SUCCESS;
}

StatusCode BNO055_register_success_callback(BNO_READ_SUCCESS_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return BNO055_ERROR_GOT_NULL_POINTER;
	}
	bno_read_success_callback = callBack;
	return SUCCESS;
}

bool BNO055_is_idle()
{
	return bno_IsIdle;
}

StatusCode BNO055_register_read(uint8_t Register, uint8_t Length)
{
	if(!BNO055_is_idle() || !USART0_has_space())
	{
		return BNO055_ERROR_NOT_READY_FOR_OPERATION;
	}
	#ifdef BNO_USE_REGISTER_VALIDATION
	if(!bno055_register_is_valid(Register, Length, BNO_REG_READ_ONLY))
	{
		return BNO055_ERROR_INVALID_ARGUMENT;
	}
	#endif
	uint8_t Message[4];
	Message[0] = START_BYTE;
	Message[1] = READ_BYTE;
	Message[2] = Register;
	Message[3] = Length;
	DEFUALT_ERROR_HANDLER(USART0_put_data(Message, 4),error_return1);
	DEFUALT_ERROR_HANDLER(USART0_set_receiver_length(2),error_return2);
	bno_IsIdle = false;

	return SUCCESS;
}

StatusCode BNO055_register_write(uint8_t Register, uint8_t Length, uint8_t* Data)
{
	if(!BNO055_is_idle() || !USART0_has_space())
	{
		return BNO055_ERROR_NOT_READY_FOR_OPERATION;
	}
	#ifdef BNO_USE_REGISTER_VALIDATION
	if(!bno055_register_is_valid(Register, Length, BNO_REG_WRITE_ONLY))
	{
		return BNO055_ERROR_INVALID_ARGUMENT;
	}
	#endif
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
	StatusCode USART_return = USART0_put_data(Message, Length + 4);
	if(USART_return == SUCCESS)
	{
		DEFUALT_ERROR_HANDLER(USART0_set_receiver_length(2),error_return);
		bno_IsIdle = false;
		free(Message);
		return SUCCESS;
	}else
	{
		free(Message);
		return USART_return;
	}
}


void bno_response_received(uint8_t* Data, uint16_t Length)
{
	StatusCode USART_return;
	switch(bno_rec_states)
	{
		case BNO_REC_STATE_IDLE:
			if(Length == 2 && Data[0] == ACK_OR_ERROR_BYTE)
			{
				if(Data[2] != BNO_STATUS_WRITE_SUCCESS && bno_error_callback != NULL)
				{
					bno_error_callback(Data[1], SUCCESS);
				}
				bno_IsIdle = true;
			}
			else if(Length == 2 && Data[0] == READ_SUCCESS_BYTE)
			{
				bno_rec_length = Data[1];
				USART_return = USART0_set_receiver_length(bno_rec_length);
				if(USART_return != SUCCESS)
				{
					bno_error_callback(BNO_TRANSMIT_ERROR, USART_return);
					return;
				}
				bno_rec_states = BNO_REC_STATE_READ_SUCCESS;
			}else
			{
				if(bno_error_callback != NULL)
				{
					bno_error_callback(BNO_TRANSMIT_ERROR, BNO055_ERROR_ARGUMENT_OUT_OF_RANGE);
				}
				bno_IsIdle = true;
			}
		break;
		
		case BNO_REC_STATE_READ_SUCCESS:
			if(bno_read_success_callback != NULL)
			{
				bno_read_success_callback(Data, bno_rec_length);
			}
			USART_return = USART0_set_receiver_length(2);
			if(USART_return != SUCCESS)
			{
				bno_error_callback(BNO_TRANSMIT_ERROR, USART_return);
				bno_rec_states = BNO_REC_STATE_IDLE;
				return;
			}
			bno_IsIdle = true;
			bno_rec_states = BNO_REC_STATE_IDLE;
		break;
		
		default:
			if(bno_error_callback != NULL)
			{
				bno_error_callback(BNO_TRANSMIT_ERROR, ERROR_FATAL);
			}
			bno_IsIdle = true;
		break;
	}
}

