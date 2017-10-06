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
#include <string.h>

#define ACK_CHAR	6	//Acknowledge
#define NACK_CHAR	21	//Negative acknowledge
#define START_CHAR	2	//Start of text
#define STOP_CHAR	4	//End of transmission

typedef void (*LISTENER_CALLBACK)(uint8_t Type, uint8_t recivedData[]);

struct transmissionData  
{
	uint8_t Type;
	uint8_t Length;
	uint8_t* Data;
	uint8_t CRC[4];
}; 
typedef struct transmissionData transmissionData;

struct listenHandlerData  
{
	uint8_t Type;
	LISTENER_CALLBACK CallBack;
}; 
typedef struct listenHandlerData listenHandlerData;

typedef enum {
	BEGIN = 0, 
	TYPE = 1, 
	LENGTH = 2, 
	DATA = 3, 
	CRC = 4, 
	END = 5, 
	ERROR = 6, 
	TIMEOUT = 7
} recvStates;

/*
* returns the Length of an Array
*/
#define LENGTH(x)  (sizeof(x) / sizeof((x)[0]))
#define DATADEF(x)		x,(sizeof(x) / sizeof((x)[0]))

/*
* Initializes everything needed for the UARTCOMM Module
* the interrupts have to be enabled afterwards manually
*/
void UARTCOM_init(uint32_t BaudRate);


/*
* Checks if there is enough space in the send buffer to send the required data
* Suggested implementation (LENGTH only works if the dataArray is NOT passed by a pointer!): 
* ...
* uint8_t dataArray[] = { ... };
* if( UARTCOMM_ready_to_send(LENGTH(dataArray)) )
* {
* 	UARTCOM_transmit_block( ... );
* }
*/
bool UARTCOM_ready_to_send(uint8_t dataLength);

/*
* Sends data over the UARTCOMM Module after a safety check.
* Returns true if successful, returns false if unsuccessful
* Suggested implementation (LENGTH only works if the dataArray is NOT passed by a pointer!): 
* uint8_t dataArray[] = { ... };
* UARTCOM_transmit_block(0-256, dataArray, LENGTH(dataArray));
*/
bool UARTCOM_transmit_block(uint8_t Type, const uint8_t Data[], uint8_t Length);

/*
* Registers a listener for a specific type and links a callback to that type
* returns true if the registration was sucessful
*/
bool UARTCOM_register_listener(uint8_t Type, LISTENER_CALLBACK callBack);

/*
* Sends a low level message which doesen't require a ack from the reciver.
* gets delivered even if another transmission is waiting for the ack.
*/
void UARTCOM_sendDebug(char Text[]);

/*
* Same as sendDebug, but sends a single 8 bit number instead of a string
*/
void UARTCOM_sendDebug_n(uint8_t Number);


//temporarily public for testing





#endif /* UARTCOM_H_ */
