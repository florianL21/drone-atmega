/*
 * UARTCOM.c
 *
 * Created: 29.09.2017 23:25:27
 *  Author: flola
 */ 
#include "UARTCOM.h"

bool readyToSend = true;
uint8_t* transmitData;

void uart_recived_char(uint8_t recvChar)
{
	static uint8_t CharCounter = 0;
	static bool recvInProgress = false;
	static transmissionData receivedData;
	if(recvInProgress == false && CharCounter == 0 && recvChar == START_CHAR)
	{
		recvInProgress = true;
		CharCounter++;
	}else if(CharCounter == 1)
	{
		receivedData.Type = recvChar;
		CharCounter++;
	}else if(CharCounter == 2)
	{
		receivedData.Length = recvChar;
		CharCounter++;
	}else if(CharCounter == 3)
	{
		receivedData.Data = malloc(receivedData.Length*sizeof(uint8_t));
		receivedData.Data[0] = recvChar;
		CharCounter++;
	}else if(CharCounter >= 4 && CharCounter <= receivedData.Length + 3)
	{
		receivedData.Data[CharCounter - 3] = recvChar;
		CharCounter++;
	}else if(CharCounter > receivedData.Length + 3 && CharCounter <= receivedData.Length + 7)
	{
		receivedData.CRC[CharCounter - receivedData.Length - 3] = recvChar;
		CharCounter++;
	}else if((CharCounter = (receivedData.Length + 8)) && (recvChar == STOP_CHAR))
	{
		#ifdef DEBUG
			uart0_puts("Recive Sucsessful\n\r");
		#endif
	}else
	{
		#ifdef DEBUG
			uart0_puts("Recive ERROR\n\r");
		#endif
	}
}

void UARTCOM_init(uint32_t BaudRate)
{
	uart0_init(BaudRate);
	uart0_register_recived_callback(uart_recived_char);
}

bool UARTCOM_ready_to_send()
{
	return readyToSend;
}

void UARTCOM_transmit_ack()
{
	uart0_putc(START_CHAR); //Start of Text
	uart0_putc(ACK_CHAR);	//Type
	uart0_putc(0);			//length = 0
	for (uint8_t i = 0; i < 4; i++)
	{
		uart0_putc(0);	//CRC
	}
	uart0_putc(STOP_CHAR); //End of transmission
}

void UARTCOM_transmit_nack()
{
	uart0_putc(START_CHAR);	//Start
	uart0_putc(NACK_CHAR);	//Type
	uart0_putc(0);			//length = 0
	for (uint8_t i = 0; i < 4; i++)
	{
		uart0_putc(0);	//CRC
	}
	uart0_putc(STOP_CHAR);	//End
}

void UARTCOM_transmit_block(transmissionData Data)
{
	if(UARTCOM_ready_to_send())
	{
		uart0_putc(START_CHAR);			//Start
		uart0_putc(Data.Type);			//Type
		uart0_putc(sizeof(Data.Data)/sizeof(Data.Data[0]));		//Length
		for(uint8_t i = 0; i < Data.Length; i++)
		{
			uart0_putc(Data.Data[i]);	//Transmit Data
		}
		for (uint8_t i = 0; i < 4; i++)
		{
			uart0_putc(Data.CRC[i]);	//CRC
		}
		uart0_putc(STOP_CHAR);			//End
		readyToSend = false;
	}
}

