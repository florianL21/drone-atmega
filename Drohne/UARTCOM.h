/*
 * UARTCOM.h
 *
 * Created: 29.09.2017 23:25:38
 *  Author: flola
 */ 


#ifndef UARTCOM_H_
#define UARTCOM_H_

#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include "config.h"
#include "UART0.h"

#define ACK_CHAR	6	//Acknowledge
#define NACK_CHAR	21	//Negative acknowledge
#define START_CHAR	2	//Start of text
#define STOP_CHAR	4	//End of transmission

struct transmissionData  
{
	uint8_t Type;
	uint8_t Length;
	uint8_t* Data;
	uint8_t CRC[4];
}; 
typedef struct transmissionData transmissionData;


#endif /* UARTCOM_H_ */