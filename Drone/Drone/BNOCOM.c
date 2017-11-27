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
	BNOCOM_REC_STATE_IDLE			= 0,
	BNOCOM_REC_STATE_READ_SUCCESS	= 1,
	BNOCOM_REC_STATE_ERROR			= 2,
	
} BNOCOM_recvStates;



bool bnocom_IsIdle = true;
uint8_t bnocom_rec_length = 0;
BNOCOM_ERROR_CALLBACK bnocom_error_callback = NULL;
//TODO: rename to bno_success_callback
BNOCOM_SUCCESS_CALLBACK bnocom_read_success_callback = NULL;
BNOCOM_recvStates bnocom_rec_states = BNOCOM_REC_STATE_IDLE;
void bnocom_response_received(uint8_t* Data,uint16_t Length);
bool bnocom_register_validation_check(uint8_t Register, uint8_t Length, uint8_t read_write);




/***************************************************************************************************
* BNO Blocking mode function for easier initialisation:
****************************************************************************************************/

uint8_t* bno_response_value;
uint8_t bno_response_length = 0;
bool bno_waiting_for_response = false;
StatusCode bno_response_status = SUCCESS;
BNO_STATUS_BYTES bno_response_error = BNO_STATUS_READ_SUCCESS;

void bno_success(uint8_t* Data, uint8_t Length)
{
	bno_waiting_for_response = false;
	bno_response_length = Length;
	bno_response_value = Data;
	bno_response_status = SUCCESS;
}

void bno_error(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code)
{
	bno_waiting_for_response = false;
	bno_response_error = Error;
	bno_response_status = BNO055_ERROR;
}

StatusCode BNOCOM_write_and_wait_for_response_1byte(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Data)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	return BNOCOM_write_and_wait_for_response(RegisterTableOffset, RegisterTablePage, &Data, 1);
}

StatusCode BNOCOM_read_and_wait_for_response_1byte(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* responseData)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	if(responseData == NULL)
		return BNO055_ERROR_GOT_NULL_POINTER;
	uint8_t responseLength = 0;
	DEFUALT_ERROR_HANDLER(BNOCOM_read_and_wait_for_response(RegisterTableOffset, RegisterTablePage, responseData, &responseLength), bno_read_status);
	if(responseLength != 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	return SUCCESS;
}

StatusCode BNOCOM_write_and_wait_for_response(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* Data, uint8_t Length)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1 || Length == 0)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	if(Data == NULL)
		return BNO055_ERROR_GOT_NULL_POINTER;
	StatusCode error_return = SUCCESS;
	//Store original callbacks
	BNOCOM_ERROR_CALLBACK SavedErrorCallback = bnocom_error_callback;
	BNOCOM_SUCCESS_CALLBACK SavedSuccessCallback = bnocom_read_success_callback;
	//register new callbacks
	BNOCOM_register_error_callback(bno_error);
	BNOCOM_register_success_callback(bno_success);

	bno_waiting_for_response = true;
	error_return = BNOCOM_register_write_by_table(RegisterTableOffset, RegisterTablePage, Data, Length);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_waiting_for_response);	//wait for transmission response
	//restore original callbacks
	BNOCOM_register_error_callback(SavedErrorCallback);
	BNOCOM_register_success_callback(SavedSuccessCallback);
	return bno_response_status;
}

StatusCode BNOCOM_read_and_wait_for_response(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t responseData[], uint8_t* responseLength)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	if(responseData == NULL || responseLength == NULL)
		return BNO055_ERROR_GOT_NULL_POINTER;
	StatusCode error_return = SUCCESS;
	//Store original callbacks
	BNOCOM_ERROR_CALLBACK SavedErrorCallback = bnocom_error_callback;
	BNOCOM_SUCCESS_CALLBACK SavedSuccessCallback = bnocom_read_success_callback;
	//register new callbacks
	BNOCOM_register_error_callback(bno_error);
	BNOCOM_register_success_callback(bno_success);
	uint8_t expectedLength = *responseLength; //store length for comparison
	bno_waiting_for_response = true;
	error_return = BNOCOM_register_read_by_table(RegisterTableOffset, RegisterTablePage, expectedLength);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_waiting_for_response);	//wait for transmission response
	//restore original callbacks
	BNOCOM_register_error_callback(SavedErrorCallback);
	BNOCOM_register_success_callback(SavedSuccessCallback);
	if(bno_response_status != SUCCESS)
		return bno_response_status;
	
	if(expectedLength != bno_response_length)
	{
		*responseLength = bno_response_length;
		return BNO055_ERROR_LENGTH_MISSMATCH;
	}
	//copy response to destination:
	for(uint8_t counter = 0; counter < bno_response_length; counter++)
	{
		responseData[counter] = bno_response_value[counter];
	}
	return SUCCESS;
}


/***************************************************************************************************
* BNO easy to use functions:
****************************************************************************************************/

StatusCode BNOCOM_register_write_1byte_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Data)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY)
			return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_write(BNO055_reg_table0[BNO_REG][RegisterTableOffset], 1, &Data);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY)
			return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_write(BNO055_reg_table1[BNO_REG][RegisterTableOffset], 1, &Data);
	}
}

StatusCode BNOCOM_register_write_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* Data, uint8_t Length)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY)
			return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_write(BNO055_reg_table0[BNO_REG][RegisterTableOffset], Length, Data);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY)
			return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_write(BNO055_reg_table1[BNO_REG][RegisterTableOffset], Length, Data);
	}
}

StatusCode BNOCOM_register_write_1byte(uint8_t Register, uint8_t Data)
{
	return BNOCOM_register_write(Register,1,&Data);
}

StatusCode BNOCOM_register_read_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Length)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_WRITE_ONLY)
			return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_read(BNO055_reg_table0[BNO_REG][RegisterTableOffset], Length);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_WRITE_ONLY)
			return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_read(BNO055_reg_table1[BNO_REG][RegisterTableOffset], Length);
	}
}

StatusCode BNOCOM_register_read_1byte_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_WRITE_ONLY)
			return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_read(BNO055_reg_table0[BNO_REG][RegisterTableOffset], 1);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_WRITE_ONLY)
			return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_read(BNO055_reg_table1[BNO_REG][RegisterTableOffset], 1);
	}
}

StatusCode BNOCOM_register_read_1byte(uint8_t Register)
{
	return BNOCOM_register_read(Register,1);
}






/***************************************************************************************************
* BNO Communication Level:
****************************************************************************************************/

bool bnocom_register_is_valid(uint8_t Register, uint8_t Length, bool read_write)
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

StatusCode BNOCOM_Init(BNOCOM_SUCCESS_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return BNO055_ERROR_GOT_NULL_POINTER;
	}
	DEFUALT_ERROR_HANDLER(USART0_init(115200, 2),error_return1);
	DEFUALT_ERROR_HANDLER(USART0_register_received_callback(bnocom_response_received),error_return2);
	bnocom_read_success_callback = callBack;
	return SUCCESS;
}

StatusCode BNOCOM_register_error_callback(BNOCOM_ERROR_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return BNO055_ERROR_GOT_NULL_POINTER;
	}
	bnocom_error_callback = callBack;
	return SUCCESS;
}

StatusCode BNOCOM_register_success_callback(BNOCOM_SUCCESS_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return BNO055_ERROR_GOT_NULL_POINTER;
	}
	bnocom_read_success_callback = callBack;
	return SUCCESS;
}

bool BNOCOM_is_idle()
{
	return bnocom_IsIdle;
}

StatusCode BNOCOM_register_read(uint8_t Register, uint8_t Length)
{
	if(!BNOCOM_is_idle() || !USART0_has_space())
	{
		return BNO055_ERROR_NOT_READY_FOR_OPERATION;
	}
	#ifdef BNO_USE_REGISTER_VALIDATION
	if(!bnocom_register_is_valid(Register, Length, BNO_REG_READ_ONLY))
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
	bnocom_IsIdle = false;

	return SUCCESS;
}



StatusCode BNOCOM_register_write(uint8_t Register, uint8_t Length, uint8_t* Data)
{
	if(!BNOCOM_is_idle() || !USART0_has_space())
	{
		return BNO055_ERROR_NOT_READY_FOR_OPERATION;
	}
	#ifdef BNO_USE_REGISTER_VALIDATION
	if(!bnocom_register_is_valid(Register, Length, BNO_REG_WRITE_ONLY))
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
		bnocom_IsIdle = false;
		free(Message);
		return SUCCESS;
	}else
	{
		free(Message);
		return USART_return;
	}
}


void bnocom_response_received(uint8_t* Data, uint16_t Length)
{
	StatusCode USART_return;
	switch(bnocom_rec_states)
	{
		case BNOCOM_REC_STATE_IDLE:
			if(Length == 2 && Data[0] == ACK_OR_ERROR_BYTE)
			{
				if(Data[2] == BNO_STATUS_WRITE_SUCCESS && bnocom_read_success_callback != NULL)
				{
					bnocom_read_success_callback(NULL,0);
				}else if(Data[2] != BNO_STATUS_WRITE_SUCCESS && bnocom_error_callback != NULL)
				{
					bnocom_error_callback(Data[1], SUCCESS);
				}
				bnocom_IsIdle = true;
			}
			else if(Length == 2 && Data[0] == READ_SUCCESS_BYTE)
			{
				bnocom_rec_length = Data[1];
				USART_return = USART0_set_receiver_length(bnocom_rec_length);
				if(USART_return != SUCCESS)
				{
					bnocom_error_callback(BNO_TRANSMIT_ERROR, USART_return);
					return;
				}
				bnocom_rec_states = BNOCOM_REC_STATE_READ_SUCCESS;
			}else
			{
				if(bnocom_error_callback != NULL)
				{
					bnocom_error_callback(BNO_TRANSMIT_ERROR, BNO055_ERROR_ARGUMENT_OUT_OF_RANGE);
				}
				bnocom_IsIdle = true;
			}
		break;
		
		case BNOCOM_REC_STATE_READ_SUCCESS:
			if(bnocom_read_success_callback != NULL)
			{
				bnocom_read_success_callback(Data, bnocom_rec_length);
			}
			USART_return = USART0_set_receiver_length(2);
			if(USART_return != SUCCESS)
			{
				bnocom_error_callback(BNO_TRANSMIT_ERROR, USART_return);
				bnocom_rec_states = BNOCOM_REC_STATE_IDLE;
				return;
			}
			bnocom_IsIdle = true;
			bnocom_rec_states = BNOCOM_REC_STATE_IDLE;
		break;
		
		default:
			if(bnocom_error_callback != NULL)
			{
				bnocom_error_callback(BNO_TRANSMIT_ERROR, ERROR_FATAL);
			}
			bnocom_IsIdle = true;
		break;
	}
}

