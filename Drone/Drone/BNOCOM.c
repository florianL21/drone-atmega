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

uint8_t* bno_response_value = NULL;
uint8_t bno_response_length = 0;
bool bno_waiting_for_response = false;
ErrorCode bno_response_status = SUCCESS;
BNO_STATUS_BYTES bno_response_error = BNO_STATUS_READ_SUCCESS;

void bno_success(uint8_t* Data, uint8_t Length)
{	
	bno_response_length = Length;
	if(bno_response_value != NULL)
		free(bno_response_value);
	bno_response_value = malloc(Length * sizeof(uint8_t));
	if(bno_response_value == NULL)
		bno_response_status = MODULE_BNOCOM | FUNCTION_success | ERROR_MALLOC_RETURNED_NULL;
	else{
		memcpy(bno_response_value, Data, Length);
		bno_response_status = SUCCESS;
	}
	bno_waiting_for_response = false;
}

void bno_error(BNO_STATUS_BYTES Error, ErrorCode Transmit_error_code)
{
	/*UART0_puts("error:\n\r");
	UART0_put_int(Error);
	UART0_puts("\n\r");*/
	bno_waiting_for_response = false;
	bno_response_error = Error;
	bno_response_status = MODULE_BNOCOM | FUNCTION_success | ERROR_TRANSMISSION_ERROR;
}

ErrorCode BNOCOM_write_and_wait_for_response_1byte(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Data)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return  MODULE_BNOCOM | FUNCTION_write_and_wait_for_response_1byte | ERROR_ARGUMENT_OUT_OF_RANGE;
	return ErrorHandling_set_top_level(BNOCOM_write_and_wait_for_response(RegisterTableOffset, RegisterTablePage, &Data, 1), MODULE_BNOCOM, FUNCTION_write_and_wait_for_response_1byte);
}

ErrorCode BNOCOM_read_and_wait_for_response_1byte(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* responseData)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return  MODULE_BNOCOM | FUNCTION_read_and_wait_for_response_1byte | ERROR_ARGUMENT_OUT_OF_RANGE;
	if(responseData == NULL)
		return  MODULE_BNOCOM | FUNCTION_read_and_wait_for_response_1byte | ERROR_GOT_NULL_POINTER;
	uint8_t responseLength = 1;
	DEFAULT_ERROR_HANDLER(BNOCOM_read_and_wait_for_response(RegisterTableOffset, RegisterTablePage, responseData, &responseLength), MODULE_BNOCOM, FUNCTION_read_and_wait_for_response_1byte);
	if(responseLength != 1)
		return  MODULE_BNOCOM | FUNCTION_read_and_wait_for_response_1byte | ERROR_ARGUMENT_OUT_OF_RANGE;
	return SUCCESS;
}

ErrorCode BNOCOM_write_and_wait_for_response(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* Data, uint8_t Length)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1 || Length == 0)
		return  MODULE_BNOCOM | FUNCTION_write_and_wait_for_response | ERROR_ARGUMENT_OUT_OF_RANGE;
	if(Data == NULL)
		return  MODULE_BNOCOM | FUNCTION_write_and_wait_for_response | ERROR_GOT_NULL_POINTER;
	ErrorCode error_return = SUCCESS;
	//Store original callbacks
	BNOCOM_ERROR_CALLBACK SavedErrorCallback = bnocom_error_callback;
	BNOCOM_SUCCESS_CALLBACK SavedSuccessCallback = bnocom_read_success_callback;
	//register new callbacks
	BNOCOM_register_error_callback(bno_error);
	BNOCOM_register_success_callback(bno_success);

	bno_waiting_for_response = true;
	error_return = BNOCOM_register_write_by_table(RegisterTableOffset, RegisterTablePage, Data, Length);
	if(error_return != SUCCESS)
		return ErrorHandling_set_top_level(error_return, MODULE_BNOCOM, FUNCTION_write_and_wait_for_response);
	while(bno_waiting_for_response);	//wait for transmission response
	//restore original callbacks
	BNOCOM_register_error_callback(SavedErrorCallback);
	BNOCOM_register_success_callback(SavedSuccessCallback);
	return ErrorHandling_set_top_level(bno_response_status, MODULE_BNOCOM, FUNCTION_write_and_wait_for_response);
}

ErrorCode BNOCOM_read_and_wait_for_response(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t responseData[], uint8_t* responseLength)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return  MODULE_BNOCOM | FUNCTION_read_and_wait_for_response | ERROR_ARGUMENT_OUT_OF_RANGE;
	if(responseData == NULL || responseLength == NULL)
		return  MODULE_BNOCOM | FUNCTION_read_and_wait_for_response | ERROR_GOT_NULL_POINTER;
	ErrorCode error_return = SUCCESS;
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
		return ErrorHandling_set_top_level(error_return, MODULE_BNOCOM, FUNCTION_read_and_wait_for_response);
	while(bno_waiting_for_response == true);	//wait for transmission response
	//restore original callbacks
	BNOCOM_register_error_callback(SavedErrorCallback);
	BNOCOM_register_success_callback(SavedSuccessCallback);
	if(bno_response_status != SUCCESS)
	{
		if(*responseLength != 0)
			responseData[0] = bno_response_error;
		return ErrorHandling_set_top_level(bno_response_status, MODULE_BNOCOM, FUNCTION_read_and_wait_for_response);
	}
	
	if(expectedLength != bno_response_length)
	{
		*responseLength = bno_response_length;
		return MODULE_BNOCOM | FUNCTION_read_and_wait_for_response | ERROR_LENGTH_MISSMATCH;
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

ErrorCode BNOCOM_register_write_1byte_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Data)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return MODULE_BNOCOM | FUNCTION_register_write_1byte_by_table | ERROR_ARGUMENT_OUT_OF_RANGE;
	
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY)
			return MODULE_BNOCOM | FUNCTION_register_write_1byte_by_table | ERROR_INVALID_ARGUMENT;
		return ErrorHandling_set_top_level(BNOCOM_register_write(BNO055_reg_table0[BNO_REG][RegisterTableOffset], 1, &Data), MODULE_BNOCOM, FUNCTION_register_write_1byte_by_table);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY)
			return MODULE_BNOCOM | FUNCTION_register_write_1byte_by_table | ERROR_INVALID_ARGUMENT;
		return ErrorHandling_set_top_level(BNOCOM_register_write(BNO055_reg_table1[BNO_REG][RegisterTableOffset], 1, &Data), MODULE_BNOCOM, FUNCTION_register_write_1byte_by_table);
	}
}

ErrorCode BNOCOM_register_write_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* Data, uint8_t Length)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return MODULE_BNOCOM | FUNCTION_register_write_by_table | ERROR_ARGUMENT_OUT_OF_RANGE;
	
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY)
			return MODULE_BNOCOM | FUNCTION_register_write_by_table | ERROR_INVALID_ARGUMENT;
		return ErrorHandling_set_top_level(BNOCOM_register_write(BNO055_reg_table0[BNO_REG][RegisterTableOffset], Length, Data), MODULE_BNOCOM, FUNCTION_register_write_by_table);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY)
			return MODULE_BNOCOM | FUNCTION_register_write_by_table | ERROR_INVALID_ARGUMENT;
		return ErrorHandling_set_top_level(BNOCOM_register_write(BNO055_reg_table1[BNO_REG][RegisterTableOffset], Length, Data), MODULE_BNOCOM, FUNCTION_register_write_by_table);
	}
}

ErrorCode BNOCOM_register_write_1byte(uint8_t Register, uint8_t Data)
{
	return ErrorHandling_set_top_level(BNOCOM_register_write(Register,1,&Data), MODULE_BNOCOM, FUNCTION_register_write_1byte);
}

ErrorCode BNOCOM_register_read_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Length)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return MODULE_BNOCOM | FUNCTION_register_read_by_table | ERROR_ARGUMENT_OUT_OF_RANGE;
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_WRITE_ONLY)
			return MODULE_BNOCOM | FUNCTION_register_read_by_table | ERROR_INVALID_ARGUMENT;
		return ErrorHandling_set_top_level(BNOCOM_register_read(BNO055_reg_table0[BNO_REG][RegisterTableOffset], Length), MODULE_BNOCOM, FUNCTION_register_read_by_table);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_WRITE_ONLY)
			return MODULE_BNOCOM | FUNCTION_register_read_by_table | ERROR_INVALID_ARGUMENT;
		return ErrorHandling_set_top_level(BNOCOM_register_read(BNO055_reg_table1[BNO_REG][RegisterTableOffset], Length), MODULE_BNOCOM, FUNCTION_register_read_by_table);
	}
}

ErrorCode BNOCOM_register_read_1byte_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return MODULE_BNOCOM | FUNCTION_register_read_1byte_by_table | ERROR_ARGUMENT_OUT_OF_RANGE;
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_WRITE_ONLY)
			return MODULE_BNOCOM | FUNCTION_register_read_1byte_by_table | ERROR_INVALID_ARGUMENT;
		return ErrorHandling_set_top_level(BNOCOM_register_read(BNO055_reg_table0[BNO_REG][RegisterTableOffset], 1), MODULE_BNOCOM, FUNCTION_register_read_1byte_by_table);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_WRITE_ONLY)
			return MODULE_BNOCOM | FUNCTION_register_read_1byte_by_table | ERROR_INVALID_ARGUMENT;
		return ErrorHandling_set_top_level(BNOCOM_register_read(BNO055_reg_table1[BNO_REG][RegisterTableOffset], 1), MODULE_BNOCOM, FUNCTION_register_read_1byte_by_table);
	}
}

ErrorCode BNOCOM_register_read_1byte(uint8_t Register)
{
	return ErrorHandling_set_top_level(BNOCOM_register_read(Register,1), MODULE_BNOCOM, FUNCTION_register_read_1byte);
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

ErrorCode BNOCOM_Init(BNOCOM_SUCCESS_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return MODULE_BNOCOM | FUNCTION_Init | ERROR_GOT_NULL_POINTER;
	}
	DEFAULT_ERROR_HANDLER(USART0_init(115200, 2), MODULE_BNOCOM, FUNCTION_Init);
	DEFAULT_ERROR_HANDLER(USART0_register_received_callback(bnocom_response_received), MODULE_BNOCOM, FUNCTION_Init);
	bnocom_read_success_callback = callBack;
	return SUCCESS;
}

ErrorCode BNOCOM_register_error_callback(BNOCOM_ERROR_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return MODULE_BNOCOM | FUNCTION_register_error_callback | ERROR_GOT_NULL_POINTER;
	}
	bnocom_error_callback = callBack;
	return SUCCESS;
}

ErrorCode BNOCOM_register_success_callback(BNOCOM_SUCCESS_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return MODULE_BNOCOM | FUNCTION_register_success_callback | ERROR_GOT_NULL_POINTER;
	}
	bnocom_read_success_callback = callBack;
	return SUCCESS;
}

bool BNOCOM_is_idle()
{
	return bnocom_IsIdle;
}

ErrorCode BNOCOM_register_read(uint8_t Register, uint8_t Length)
{
	if(!BNOCOM_is_idle() || !USART0_has_space())
	{
		return MODULE_BNOCOM | FUNCTION_register_read | ERROR_NOT_READY_FOR_OPERATION;
	}
	#ifdef BNO_USE_REGISTER_VALIDATION
	if(!bnocom_register_is_valid(Register, Length, BNO_REG_READ_ONLY))
	{
		return MODULE_BNOCOM | FUNCTION_register_read | ERROR_INVALID_ARGUMENT;
	}
	#endif
	uint8_t Message[4];
	Message[0] = START_BYTE;
	Message[1] = READ_BYTE;
	Message[2] = Register;
	Message[3] = Length;
	DEFAULT_ERROR_HANDLER(USART0_put_data(Message, 4), MODULE_BNOCOM, FUNCTION_register_read);
	DEFAULT_ERROR_HANDLER(USART0_set_receiver_length(2), MODULE_BNOCOM, FUNCTION_register_read);
	bnocom_IsIdle = false;

	return SUCCESS;
}

ErrorCode BNOCOM_register_write(uint8_t Register, uint8_t Length, uint8_t* Data)
{
	if(!BNOCOM_is_idle() || !USART0_has_space())
	{
		return MODULE_BNOCOM | FUNCTION_register_write | ERROR_NOT_READY_FOR_OPERATION;
	}
	#ifdef BNO_USE_REGISTER_VALIDATION
	if(!bnocom_register_is_valid(Register, Length, BNO_REG_WRITE_ONLY))
	{
		return MODULE_BNOCOM | FUNCTION_register_write | ERROR_INVALID_ARGUMENT;
	}
	#endif
	uint8_t* Message = malloc((Length+4)*sizeof(uint8_t));
	if(Message == NULL)
	{
		return MODULE_BNOCOM | FUNCTION_register_write | ERROR_MALLOC_RETURNED_NULL;
	}
	Message[0] = START_BYTE;
	Message[1] = WRITE_BYTE;
	Message[2] = Register;
	Message[3] = Length;
	for (uint8_t i = 0; i < Length; i++)
	{
		Message[i + 4] = Data[i];
	}
	ErrorCode USART_return = USART0_put_data(Message, Length + 4);
	if(USART_return == SUCCESS)
	{
		DEFAULT_ERROR_HANDLER(USART0_set_receiver_length(2), MODULE_BNOCOM, FUNCTION_register_write);
		bnocom_IsIdle = false;
		free(Message);
		return SUCCESS;
	}else
	{
		free(Message);
		return ErrorHandling_set_top_level(USART_return, MODULE_BNOCOM, FUNCTION_register_write);
	}
}


void bnocom_response_received(uint8_t* Data, uint16_t Length)
{
	ErrorCode USART_return;
	switch(bnocom_rec_states)
	{
		case BNOCOM_REC_STATE_IDLE:
			if(Length == 2 && Data[0] == ACK_OR_ERROR_BYTE)
			{
				bnocom_IsIdle = true;
				if(Data[1] == BNO_STATUS_WRITE_SUCCESS && bnocom_read_success_callback != NULL)
				{
					bnocom_read_success_callback(NULL,0);
				}else if(Data[1] != BNO_STATUS_WRITE_SUCCESS && bnocom_error_callback != NULL)
				{
					bnocom_error_callback(Data[1], MODULE_BNOCOM | FUNCTION_response_received | ERROR_GENERIC);
				}
			}
			else if(Length == 2 && Data[0] == READ_SUCCESS_BYTE)
			{
				bnocom_rec_length = Data[1];
				USART_return = ErrorHandling_set_top_level(USART0_set_receiver_length(bnocom_rec_length), MODULE_BNOCOM, FUNCTION_response_received);
				if(USART_return != SUCCESS)
				{
					bnocom_error_callback(BNO_TRANSMIT_ERROR, USART_return);
					return;
				}
				bnocom_rec_states = BNOCOM_REC_STATE_READ_SUCCESS;
			}else
			{
				bnocom_IsIdle = true;
				if(bnocom_error_callback != NULL)
				{
					bnocom_error_callback(BNO_TRANSMIT_ERROR, MODULE_BNOCOM | FUNCTION_response_received | ERROR_ARGUMENT_OUT_OF_RANGE);
				}
			}
		break;
		
		case BNOCOM_REC_STATE_READ_SUCCESS:
			bnocom_IsIdle = true;
			if(bnocom_read_success_callback != NULL)
			{
				bnocom_read_success_callback(Data, bnocom_rec_length);
			}
			USART_return = ErrorHandling_set_top_level(USART0_set_receiver_length(2), MODULE_BNOCOM, FUNCTION_response_received);
			if(USART_return != SUCCESS)
			{
				bnocom_error_callback(BNO_TRANSMIT_ERROR, USART_return);
				bnocom_rec_states = BNOCOM_REC_STATE_IDLE;
				return;
			}
			bnocom_rec_states = BNOCOM_REC_STATE_IDLE;
		break;
		
		default:
			bnocom_IsIdle = true;
			if(bnocom_error_callback != NULL)
			{
				bnocom_error_callback(BNO_TRANSMIT_ERROR, MODULE_BNOCOM | FUNCTION_response_received | ERROR_GENERIC);
			}
		break;
	}
}

