/*
 * I2C.h
 *
 * Created: 29.09.2017 23:52:00
 *  Author: Markus Lorenz
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <stddef.h>
#include <string.h>
#include "config.h"

/*
 *	uint8_t  - address
 *	uint8_t* - pointer to data buffer that was written
 */
typedef void (*i2c_callback)(uint8_t, uint8_t*);

/*
 *	 Configures the TWI hardware
 */
void i2c_init();

/*
 *	Writes data to a specified i2c address
 */
void i2c_write(uint8_t address, uint8_t* data, uint8_t length, i2c_callback callback);

/*
 *	Reads data from a specified i2c address
 */
void i2c_read(uint8_t address, uint8_t length, i2c_callback callback);

/*
 *	Blocking wait for operation completion
 */
uint8_t* i2c_wait();

/*
 *	Returns if the I2C interface is busy
 *	If busy: do not attempt to write/read data -> will block
 */
uint8_t i2c_is_busy();

#endif /* I2C_H_ */