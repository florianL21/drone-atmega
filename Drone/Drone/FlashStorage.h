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

#include "ASF_files/flash_efc.h"
#include "ASF_files/efc.h"
#include <stdbool.h>
#include "config.h"
#include <string.h>

// 1Kb of data
#define DATA_LENGTH   ((IFLASH1_PAGE_SIZE/sizeof(uint8_t))*4)

// choose a start address that's offset to show that it doesn't have to be on a page boundary
#define  FLASH_START  ((uint8_t *)IFLASH1_ADDR)


StatusCode FlashStorage_Init();
uint8_t FlashStorage_read(uint32_t address);
uint8_t* FlashStorage_readAddress(uint32_t address);
StatusCode FlashStorage_write(uint32_t address, uint8_t *data, uint32_t dataLength);
StatusCode FlashStorage_write_unlocked(uint32_t address, uint8_t *data, uint32_t dataLength);
StatusCode FlashStorage_write_uint8_t(uint32_t address, uint8_t Value);
StatusCode FlashStorage_write_float(uint32_t address, float Value);
float FlashStorage_read_float(uint32_t address);
uint16_t FlashStorage_read_uint16_t(uint32_t address);

#endif
