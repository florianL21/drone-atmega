/*
 * UART0.h
 *
 * Created: 29.09.2017 20:07:13
 *  Author: flola
 */ 


#ifndef UART0_H_
#define UART0_H_

#include "sam.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "HelperFunctions.h"
#ifdef DEBUG_UART0
	#include "UARTCOM.h"
#endif

bool uart0_is_idle();

typedef void (*UART_RECV_CALLBACK)(uint8_t recivedChar);

/*
 *	Sets all required registers for UART communication
 *
 */
void uart0_init(uint32_t BaudRate);

/*
 *	Add a char to the UART send buffer
 * IMPORTANT: Check if the buffer has space before calling
 *			  this function. Otherwise the char will be discarded
 *			  if the buffer is full.
 */
//void uart0_putc(uint8_t character);

/*
 *	Register a callback function which gets called when a char is received
 */
void uart0_register_received_callback(UART_RECV_CALLBACK callBack);


/*
 *	Sends a data array over uart. Strings are also allowed.
 * IMPORTANT: Check if the buffer has space before calling
 *			  this function. Otherwise the data will be discarded
 *			  if the buffer is full.
 */
void uart0_puts(uint8_t Data[]);


void uart0_put_data(uint8_t* sendData, uint16_t Length, bool requiresMemmoryCleanup);

/*
 *	returns true if the uart queue has space
 */

bool uart0_has_space();

/*
* Strictly meant for debugging purposes, may breaks the uart data transfer
*/
void uart0_force_debug_output(char Text[]);
void uart0_force_debug_output_n(uint8_t Number);



#endif /* UART0_H_ */
