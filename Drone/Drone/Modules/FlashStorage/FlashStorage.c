#include "FlashStorage.h"

ErrorCode FlashStorage_init()
{
	if(flash_init(FLASH_ACCESS_MODE_128, 6) != FLASH_RC_OK)
		return MODULE_FLASHSTORAGE | FUNCTION_init | ERROR_GENERIC;
	return SUCCESS;
}

uint8_t FlashStorage_read(uint32_t address) {
  return FLASH_START[address];
}

uint8_t* FlashStorage_readAddress(uint32_t address) {
  return FLASH_START+address;
}

ErrorCode FlashStorage_lock(uint32_t address, uint32_t dataLength)
{
	uint32_t retCode;
	if ((uint32_t)FLASH_START+address < IFLASH1_ADDR) {
		return MODULE_FLASHSTORAGE | FUNCTION_lock | ERROR_ADDRESS_TOO_LOW;
	}

	if ((uint32_t)FLASH_START+address >= (IFLASH1_ADDR + IFLASH1_SIZE)) {
		return MODULE_FLASHSTORAGE | FUNCTION_lock | ERROR_ADDRESS_TOO_HIGH;
	}

	if ((((uint32_t)FLASH_START+address) & 3) != 0) {
		return MODULE_FLASHSTORAGE | FUNCTION_lock | ERROR_ADDRESS_NOT_4_BYTE_BOUDARY;
	}
	
	 // Lock page
	 retCode = flash_lock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + dataLength - 1, 0, 0);
	 if (retCode != FLASH_RC_OK) {
		 return MODULE_FLASHSTORAGE | FUNCTION_lock | ERROR_FAILED_TO_LOCK_FLASH;
	 }
	return SUCCESS;
}

ErrorCode FlashStorage_unlock(uint32_t address, uint32_t dataLength)
{
	uint32_t retCode;
	if ((uint32_t)FLASH_START+address < IFLASH1_ADDR) {
		return MODULE_FLASHSTORAGE | FUNCTION_unlock | ERROR_ADDRESS_TOO_LOW;
	}

	if ((uint32_t)FLASH_START+address >= (IFLASH1_ADDR + IFLASH1_SIZE)) {
		return MODULE_FLASHSTORAGE | FUNCTION_unlock | ERROR_ADDRESS_TOO_HIGH;
	}

	if ((((uint32_t)FLASH_START+address) & 3) != 0) {
		return MODULE_FLASHSTORAGE | FUNCTION_unlock | ERROR_ADDRESS_NOT_4_BYTE_BOUDARY;
	}
	
	retCode = flash_unlock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + dataLength - 1, 0, 0);
	if (retCode != FLASH_RC_OK) {
		return MODULE_FLASHSTORAGE | FUNCTION_unlock | ERROR_FAILED_TO_UNLOCK_FLASH;
	}
	return SUCCESS;
}

ErrorCode FlashStorage_write(uint32_t address, uint8_t *data, uint32_t dataLength) {
  uint32_t retCode;

  if ((uint32_t)FLASH_START+address < IFLASH1_ADDR) {
    return MODULE_FLASHSTORAGE | FUNCTION_write | ERROR_ADDRESS_TOO_LOW;
  }

  if ((uint32_t)FLASH_START+address >= (IFLASH1_ADDR + IFLASH1_SIZE)) {
    return MODULE_FLASHSTORAGE | FUNCTION_write | ERROR_ADDRESS_TOO_HIGH;
  }

  if ((((uint32_t)FLASH_START+address) & 3) != 0) {
    return MODULE_FLASHSTORAGE | FUNCTION_write | ERROR_ADDRESS_NOT_4_BYTE_BOUDARY;
  }

  // Unlock page
  retCode = flash_unlock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + dataLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    return MODULE_FLASHSTORAGE | FUNCTION_write | ERROR_FAILED_TO_UNLOCK_FLASH;
  }

  // write data
  retCode = flash_write((uint32_t)FLASH_START+address, data, dataLength, 1);

  if (retCode != FLASH_RC_OK) {
    return MODULE_FLASHSTORAGE | FUNCTION_write | ERROR_WRITE_FAILED;
  }

  // Lock page
    retCode = flash_lock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + dataLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    return MODULE_FLASHSTORAGE | FUNCTION_write | ERROR_FAILED_TO_LOCK_FLASH;
  }
  return SUCCESS;
}

ErrorCode FlashStorage_write_unlocked(uint32_t address, uint8_t *data, uint32_t dataLength) {
  uint32_t retCode;

  if ((uint32_t)FLASH_START+address < IFLASH1_ADDR) {
    return MODULE_FLASHSTORAGE | FUNCTION_write_unlocked | ERROR_ADDRESS_TOO_LOW;
  }

  if ((uint32_t)FLASH_START+address >= (IFLASH1_ADDR + IFLASH1_SIZE)) {
    return MODULE_FLASHSTORAGE | FUNCTION_write_unlocked | ERROR_ADDRESS_TOO_HIGH;
  }

  if ((((uint32_t)FLASH_START+address) & 3) != 0) {
    return MODULE_FLASHSTORAGE | FUNCTION_write_unlocked | ERROR_ADDRESS_NOT_4_BYTE_BOUDARY;
  }

  // write data
  retCode = flash_write((uint32_t)FLASH_START+address, data, dataLength, 1);

  if (retCode != FLASH_RC_OK) {
    return MODULE_FLASHSTORAGE | FUNCTION_write_unlocked | ERROR_WRITE_FAILED;
  }

  return SUCCESS;
}

ErrorCode FlashStorage_write_uint8_t(uint32_t address, uint8_t Value)
{
	uint8_t b2[sizeof(Value)];
	memcpy(b2, &Value, sizeof(Value));
	return ErrorHandling_set_top_level(FlashStorage_write(address, b2, sizeof(Value)), MODULE_FLASHSTORAGE, FUNCTION_write_uint8_t);
}

ErrorCode FlashStorage_write_float(uint32_t address, float Value)
{
	uint8_t b2[sizeof(Value)];
	memcpy(b2, &Value, sizeof(Value));
	return ErrorHandling_set_top_level(FlashStorage_write(address, b2, sizeof(Value)), MODULE_FLASHSTORAGE, FUNCTION_write_float);
}

float FlashStorage_read_float(uint32_t address)
{
	uint8_t* b = FlashStorage_readAddress(address);
	float tempValue = 0;
	memcpy(&tempValue, b, sizeof(tempValue));
	return tempValue;
}

uint16_t FlashStorage_read_uint16_t(uint32_t address)
{
	uint8_t* b = FlashStorage_readAddress(address);
	uint16_t tempValue = 0;
	memcpy(&tempValue, b, 2);
	return tempValue;
}