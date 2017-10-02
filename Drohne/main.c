/*
 * Drohne.c
 *
 * Created: 29.09.2017 19:08:07
 * Author : Markus Lorenz
 */ 

#include <avr/io.h>
#include "ESCControl.h"
#include "UARTCOM.h"

int main(void)
{
	uint8_t dataArray[] = {0,1,2,8};
	UARTCOM_init(57600);
	sei();
	sendDebug("START");
    while (1) 
    {
		if(UARTCOM_ready_to_send(LENGTH(dataArray)))
		{
			dataArray[3]++;
			UARTCOM_transmit_block(23,DATADEF(dataArray));
		}
		
    }
}

