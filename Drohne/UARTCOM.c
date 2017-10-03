/*
 * UARTCOM.c
 *
 * Created: 29.09.2017 23:25:27
 *  Author: flola
 */ 
#include "UARTCOM.h"

bool readyToSend = true;
transmissionData tempData; 

void parseRecvData(transmissionData recvData);
void uart_recived_char(uint8_t recvChar);
void transmit_ack();
void transmit_nack();
void transmit_block(transmissionData Data);
transmissionData get_data_block(uint8_t Type, const uint8_t Data[], uint8_t Length);
bool enough_buffer_space(uint8_t dataLength);
void sendDebug_n(uint8_t Number);


void parseRecvData(transmissionData recvData)
{
	if(recvData.Type == ACK_CHAR && recvData.Length == 0)
	{
		readyToSend = true;
		tempData.Type = 0;
	}else if(recvData.Type == NACK_CHAR && recvData.Length == 0)
	{
		if(tempData.Type != 0)
		{
			transmit_block(tempData);
		}
	}
}

void uart_recived_char(uint8_t recvChar)
{
	volatile static uint8_t CharCounter = 0;
	volatile static bool recvInProgress = false;
	volatile static transmissionData receivedData;
	if(recvInProgress == false && CharCounter == 0 && recvChar == START_CHAR)
	{
		recvInProgress = true;
		//CharCounter++;
	}else if(CharCounter == 1)
	{
		receivedData.Type = recvChar;
		//CharCounter++;
	}else if(CharCounter == 2)
	{
		receivedData.Length = recvChar;
		//CharCounter++;
	}else if(CharCounter == 3)
	{
		receivedData.Data = malloc(receivedData.Length*sizeof(uint8_t));
		receivedData.Data[0] = recvChar;
		//CharCounter++;
	}else if(CharCounter >= 4 && CharCounter <= receivedData.Length + 3)
	{
		receivedData.Data[CharCounter - 3] = recvChar;
		//CharCounter++;
	}else if(CharCounter > receivedData.Length + 3 && CharCounter <= receivedData.Length + 7)
	{
		receivedData.CRC[CharCounter - receivedData.Length - 3] = recvChar;
		//CharCounter++;
	}else if((CharCounter = (receivedData.Length + 8)) && (recvChar == STOP_CHAR))
	{
		//Recv. Sucessful
		sendDebug("Recv. SUC");
		parseRecvData(receivedData);
		CharCounter = 0;
		recvInProgress = false;
	}else
	{
		CharCounter = 0;
		recvInProgress = false;
		sendDebug("Recv. ERROR");
	}
	sendDebug_n(CharCounter);
	CharCounter++;
}


void sendDebug(char Text[])
{
	uint8_t Length = strlen(Text);
	//uart0_puts(Text);
	transmit_block(get_data_block(1,Text,Length));
}

void sendDebug_n(uint8_t Number)
{
	transmit_block(get_data_block(2,&Number,1));
}

void UARTCOM_init(uint32_t BaudRate)
{
	uart0_init(BaudRate);
	uart0_register_recived_callback(uart_recived_char);
}

bool enough_buffer_space(uint8_t dataLength)
{
	return (dataLength + 8) < uart0_get_buffer_space();
}

bool UARTCOM_ready_to_send(uint8_t dataLength)
{
	return readyToSend && enough_buffer_space(dataLength);
}

void transmit_ack()
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

void transmit_nack()
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

void transmit_block(transmissionData Data)
{
	if(enough_buffer_space(Data.Length))
	{
		uart0_putc(START_CHAR);			//Start
		uart0_putc(Data.Type);			//Type
		uart0_putc(Data.Length);		//Length
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

transmissionData get_data_block(uint8_t Type, const uint8_t Data[], uint8_t Length)
{
	transmissionData tranData;
	tranData.Length = Length;
	tranData.Type = Type;
	tranData.Data = malloc(tranData.Length*sizeof(uint8_t));
	for (uint8_t i = 0; i < tranData.Length; i++)
	{
		tranData.Data[i] = Data[i];
	}
	tranData.CRC[0] = 0;
	tranData.CRC[1] = 0;
	tranData.CRC[2] = 0;
	tranData.CRC[3] = 0;
	return tranData;
}

bool UARTCOM_transmit_block(uint8_t Type, const uint8_t Data[], uint8_t Length)
{
	transmissionData tranData;
	tranData = get_data_block(Type,Data,Length);
	if(UARTCOM_ready_to_send(tranData.Length) && Type > 1 && Type <= 154)
	{
		tempData = tranData;			//Store data for retransmission
		transmit_block(tranData);
		return true;
	}else
	{
		tranData.Type = 0;
		return false;
	}
}
