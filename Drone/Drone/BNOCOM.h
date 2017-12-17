/*
 * BNO055.h
 *
 * Created: 19.11.2017 12:44:12
 *  Author: flola
 */ 


#ifndef BNOCOM_H_
#define BNOCOM_H_

#include "sam.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "USART0.h"
#include "config.h"
#include "BNO055_reg_table.h"




typedef enum {
	BNO_STATUS_READ_SUCCESS				= 0x00,
	BNO_STATUS_WRITE_SUCCESS			= 0x01,
	BNO_STATUS_READ_FAIL				= 0x02,
	BNO_STATUS_WRITE_FAIL				= 0x03,
	BNO_STATUS_REGMAP_INVALID_ADDRESS	= 0x04,
	BNO_STATUS_REGMAP_WRITE_DISABLED	= 0x05,
	BNO_STATUS_WRONG_START_BYTE			= 0x06,
	BNO_STATUS_BUS_OVER_RUN_ERROR		= 0x07,
	BNO_STATUS_MAX_LENGTH_ERROR			= 0x08,
	BNO_STATUS_MIN_LENGTH_ERROR			= 0x09,
	BNO_STATUS_RECEIVE_CHARACTER_TIMEOUT= 0x0A,
	BNO_TRANSMIT_ERROR					= 0xFF
} BNO_STATUS_BYTES;

typedef void (*BNOCOM_ERROR_CALLBACK)(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code);
typedef void (*BNOCOM_SUCCESS_CALLBACK)(uint8_t* Data, uint8_t Length);



StatusCode BNOCOM_Init(BNOCOM_SUCCESS_CALLBACK callBack);
StatusCode BNOCOM_register_error_callback(BNOCOM_ERROR_CALLBACK callBack);
StatusCode BNOCOM_register_success_callback(BNOCOM_SUCCESS_CALLBACK callBack);
bool BNOCOM_is_idle();
StatusCode BNOCOM_register_write(uint8_t Register, uint8_t Length, uint8_t* Data);
StatusCode BNOCOM_register_read(uint8_t Register, uint8_t Length);

/***************************************************************************************************
* BNOCOM blocking mode function for easier initialisation:
****************************************************************************************************/
/*
* IMPORTANT: responseLength has to be set to the expected value before the function is called. 
* If the response from the BNO does not match the expected length an BNO055_ERROR_LENGTH_MISSMATCH is thrown to prevent memmory overrrides.
* In case of an error the recived length can be read from the responseLength pointer.
*/
StatusCode BNOCOM_read_and_wait_for_response(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t responseData[], uint8_t* responseLength);
StatusCode BNOCOM_write_and_wait_for_response(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* Data, uint8_t Length);
StatusCode BNOCOM_read_and_wait_for_response_1byte(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* responseData);
StatusCode BNOCOM_write_and_wait_for_response_1byte(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Data);

/***************************************************************************************************
* BNOCOM Functions for easier register access:
****************************************************************************************************/
StatusCode BNOCOM_register_write_1byte_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Data);
StatusCode BNOCOM_register_write_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t* Data, uint8_t Length);
StatusCode BNOCOM_register_write_1byte(uint8_t Register, uint8_t Data);
StatusCode BNOCOM_register_read_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Length);
StatusCode BNOCOM_register_read_1byte_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage);
StatusCode BNOCOM_register_read_1byte(uint8_t Register);




#endif /* BNO055_H_ */