/* 
DueFlashStorage saves non-volatile data for Arduino Due.
The library is made to be similar to EEPROM library
Uses flash block 1 per default.

Note: uploading new software will erase all flash so data written to flash
using this library will not survive a new software upload. 

Inspiration from Pansenti at https://github.com/Pansenti/DueFlash
Rewritten and modified by Sebastian Nilsson
*/


#ifndef FLASHSTORAGE_H
#define FLASHSTORAGE_H

#include "../../ASF_files/flash_efc.h"
#include "../../ASF_files/efc.h"
#include <stdbool.h>
#include "../../config.h"
#include <string.h>
#include "../ErrorHandling/ErrorHandling.h"

// 1Kb of data
#define DATA_LENGTH   ((IFLASH1_PAGE_SIZE/sizeof(uint8_t))*4)

// choose a start address that's offset to show that it doesn't have to be on a page boundary
#define  FLASH_START  ((uint8_t *)IFLASH1_ADDR)

/*
	This function initializes the flash storage
	Returns:
		Any error that might have occurred.
*/
ErrorCode FlashStorage_init();

/*
	This function reads from a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:	address to read from
	Returns:
		read value
*/
uint8_t FlashStorage_read(uint32_t address);

/*
	This function reads from a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:	address to read from
	Returns:
		pointer to the read value.
*/
uint8_t* FlashStorage_readAddress(uint32_t address);

/*
	This function writes to a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:		address to write from
		- data:			data to write
		- dataLength:	length to write
	Returns:
		Any error that might have occurred.
*/
ErrorCode FlashStorage_write(uint32_t address, uint8_t *data, uint32_t dataLength);

/*
	This function writes to a address without unlocking the flash first. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:		address to write from
		- data:			data to write
		- dataLength:	length to write
	Returns:
		Any error that might have occurred.
*/
ErrorCode FlashStorage_write_unlocked(uint32_t address, uint8_t *data, uint32_t dataLength);

/*
	This function writes a uint8_t value to a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:		Address to write from
		- Value:		Value to write
	Returns:
		Any error that might have occurred.
*/
ErrorCode FlashStorage_write_uint8_t(uint32_t address, uint8_t Value);

/*
	This function writes a float value to a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:		Address to write from
		- Value:		Value to write
	Returns:
		Any error that might have occurred.
*/
ErrorCode FlashStorage_write_float(uint32_t address, float Value);

/*
	This function reads a float value from a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:		Address to read from
	Returns:
		the read float value
*/
float FlashStorage_read_float(uint32_t address);

/*
	This function reads a uint16_t value to a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:		Address to read from
	Returns:
		The read uint16_t value
*/
uint16_t FlashStorage_read_uint16_t(uint32_t address);

/*
	This function unlocks the flash at a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:		Address to unlock
		- dataLength:	Length to unlock
	Returns:
		Any error that might have occurred.
*/
ErrorCode FlashStorage_unlock(uint32_t address, uint32_t dataLength);

/*
	This function locks the flash at a address. The address must be a multiple of 4.
	Requirements:
		- FlashStorage_init
	Parameters:
		- address:		Address to lock
		- dataLength:	Length to lock
	Returns:
		Any error that might have occurred.
*/
ErrorCode FlashStorage_lock(uint32_t address, uint32_t dataLength);

#endif