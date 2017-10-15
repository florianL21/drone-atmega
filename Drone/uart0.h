/*
 * uart0.h
 *
 * Created: 15.10.2017 15:50:23
 *  Author: flola
 */ 


#ifndef UART0_H_
#define UART0_H_

#include "sam.h"
#include <stdbool.h>


void uart0_init(uint32_t baudRate);
void uart0_putc(char Character);


#endif /* UART0_H_ */