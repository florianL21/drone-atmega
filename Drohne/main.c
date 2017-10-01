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
	uint8_t DataArray[] = {0,1,2,3};
	transmissionData sendData;
	sendData = UARTCOM_get_data_block(23,DataArray,LENGTH(DataArray));
	UARTCOM_init(57600);
	sei();
    while (1) 
    {
		if(UARTCOM_ready_to_send(sendData))
		{
			DataArray[3]++;
			sendData = UARTCOM_get_data_block(23,DataArray,LENGTH(DataArray));
			UARTCOM_transmit_block(sendData);
		}
		
    }
}

