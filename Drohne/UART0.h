/*
 * UART0.h
 *
 * Created: 29.09.2017 20:07:13
 *  Author: flola
 */ 


#ifndef UART0_H_
#define UART0_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

typedef void (*UART_RECV_CALLBACK)(uint8_t recivedChar);

/*
 *	Sets all required registers for UART communication
 *
 */
void uart0_init(uint32_t BaudRate);

/*
 *	returns true if there is space in the send buffer
 */
bool uart0_buffer_has_space();

/*
 *	returns the number of free space in the send buffer
 */
uint8_t uart0_get_buffer_space();

/*
 *	Add a char to the UART send buffer
 * IMPORTANT: Check if the buffer has space before calling
 *			  this function. Otherwise the char will be discarded
 *			  if the buffer is full.
 */
void uart0_putc(uint8_t character);

/*
 *	Register a callback function which gets called when a char is received
 */
void uart0_register_recived_callback(UART_RECV_CALLBACK callBack);


/*
 *	Sends a String over UART, strictly meant for debug messages
 */
void uart0_puts(char* Data);


#endif /* UART0_H_ */