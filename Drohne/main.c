/*
 * Drohne.c
 *
 * Created: 29.09.2017 19:08:07
 * Author : Markus Lorenz
 */ 

#include <avr/io.h>
#include "ESCControl.h"
#include "UARTCOM.h"
#include "HelperFunctions.h"

int main(void)
{
	uint8_t dataArray[] = {0,0};
	UARTCOM_init(57600);
	sei();
	UARTCOM_sendDebug("START");
    while (1) 
    {
		if(UARTCOM_ready_to_send(LENGTH(dataArray)))
		{
			dataArray[0]++;
			if(!UARTCOM_transmit_block('0'+dataArray[1], DATADEF(dataArray)))
			{
				UARTCOM_sendDebug("STOP");
			}
			if(dataArray[0] == 255)
				dataArray[1]++;
		}
    }
}