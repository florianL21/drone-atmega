/*
 * BNO055.h
 *
 * Created: 19.11.2017 12:44:12
 *  Author: flola
 */ 


#ifndef BNO055_H_
#define BNO055_H_

#include "sam.h"
#include <stdlib.h>
#include <stdbool.h>
#include "USART0.h"
#include "config.h"


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

typedef void (*BNO_ERROR_CALLBACK)(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code);
typedef void (*BNO_READ_SUCCESS_CALLBACK)(uint8_t* Data, uint8_t Length);



StatusCode BNO055_Init(BNO_READ_SUCCESS_CALLBACK callBack);
StatusCode BNO055_register_error_callback(BNO_ERROR_CALLBACK callBack);
StatusCode BNO055_register_success_callback(BNO_READ_SUCCESS_CALLBACK callBack);
bool BNO055_is_idle();
StatusCode BNO055_register_write(uint8_t Register, uint8_t Length, uint8_t* Data);
StatusCode BNO055_register_read(uint8_t Register, uint8_t Length);




#endif /* BNO055_H_ */