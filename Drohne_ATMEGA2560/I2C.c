/*
 * I2C.c
 *
 * Created: 30.09.2017 00:04:56
 *  Author: Markus Lorenz
 */ 

#include "I2C.h"

static volatile uint8_t busy;
static struct  
{
	uint8_t buffer[I2C_BUFFER_LENGTH];
	uint8_t length;
	uint8_t index;
	i2c_callback callback;
} transmission;


// internal functions
void i2c_start()
{
	//	clear Int. Flag | enable I2C  | enable Int. | create START condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTA);
}

void i2c_stop()
{
	//	clear Int. Flag | enable I2C  | create STOP condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void i2c_ack()
{
	//	clear Int. Flag | enable I2C  | enable Int. | create ACK pulse
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

void i2c_nack()
{
	//	clear Int. Flag | enable I2C  | enable Int.
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
}

void i2c_send(uint8_t data)
{
	TWDR = data;
}

void i2c_receive()
{
	transmission.buffer[transmission.index++] = TWDR;
}

void i2c_reply()
{
	if(transmission.index < (transmission.length - 1)) // free space
		i2c_ack();
	else // no free space
		i2c_nack();
}

void i2c_done()
{
	uint8_t address = transmission.buffer[0] >> 1;
	uint8_t *data = &transmission.buffer[1];
	
	busy = 0;
	
	if(transmission.callback != NULL)
		transmission.callback(address, data);
}



// user functions

void i2c_init()
{
	TWBR = ((F_CPU / I2C_FREQ) - 16) / 2; // calculate TWBR value from F_CPU and I2C_FREQ
	TWSR = 0; // prescaler = 1
	TWCR |= (1 << TWEN); // enable I2C (TWI) hardware
	busy = 0;
}

void i2c_write(uint8_t address, uint8_t *data, uint8_t length, i2c_callback callback)
{
	i2c_wait(); // Well you better not call this function if the module is busy, dumbass!
	
	busy = 1;
	
	transmission.buffer[0] = (address << 1) | TW_WRITE;
	transmission.length = length + 1;
	transmission.index = 0;
	transmission.callback = callback;
	memcpy(&transmission.buffer[1], data, length);
	
	i2c_start();
}

void i2c_read(uint8_t address, uint8_t length, i2c_callback callback)
{
	i2c_wait(); // Well you better not call this function if the module is busy, dumbass!
	
	busy = 1;
	
	transmission.buffer[0] = (address << 1) | TW_READ;
	transmission.length = length + 1;
	transmission.index = 0;
	transmission.callback = callback;
	
	i2c_start();
}

uint8_t* i2c_wait()
{
	while(busy);
	return &transmission.buffer[1];
}

uint8_t i2c_is_busy()
{
	return busy;
}

// ISR
ISR(TWI_vect)
{
	switch(TW_STATUS)
	{
		case TW_START: // start condition transmitted
		case TW_REP_START: // repeated start condition transmitted
		case TW_MT_SLA_ACK: // SLA+W transmitted, ACK received
		case TW_MT_DATA_ACK: // data transmitted, ACK received
			if(transmission.index < transmission.length)
			{
				i2c_send(transmission.buffer[transmission.index++]);
				i2c_nack();
			}
			else
			{
				i2c_stop();
				i2c_done();
			}
			break;
		
		case TW_MR_DATA_ACK: // data received, ACK returned
			i2c_receive();
			i2c_reply();
			break;
			
		case TW_MR_SLA_ACK: // SLA+R transmitted, ACK received
			i2c_reply();
			break;
		
		case TW_MR_DATA_NACK: // data received, NACK returned
			i2c_receive();
			i2c_stop();
			i2c_done();
			break;
		
		case TW_MT_SLA_NACK: // SLA+W transmitted, NACK received
		case TW_MR_SLA_NACK: // SLA+R transmitted, NACK received
		case TW_MT_DATA_NACK: // data transmitted, NACK received
		default:
			i2c_stop();
			i2c_done();
			break;			
	}
}