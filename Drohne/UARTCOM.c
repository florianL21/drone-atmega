/*
 * UARTCOM.c
 *
 * Created: 29.09.2017 23:25:27
 *  Author: flola
 */ 
#include "UARTCOM.h"

volatile bool readyToSend = true;
volatile transmissionData tempData; 

volatile recvStates recvState = BEGIN;

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
		//sendDebug("A");
		readyToSend = true;
		tempData.Type = 0;
	}else if(recvData.Type == NACK_CHAR && recvData.Length == 0)
	{
		//sendDebug("N");
		if(tempData.Type != 0)
		{
			transmit_block(tempData);
		}
	} else
	{
		//sendDebug("E");
	}
}

void uart_recived_char(uint8_t recvChar)
{
	static transmissionData receivedData;
	static bool recvInProgress = false;
	static uint8_t CharCounter = 0;
	switch(recvState)
	{
		case BEGIN:
			if(recvChar == START_CHAR)
			{
				recvState = TYPE;
				recvInProgress = true;
			} else
			{
				//wrong start char
				recvState = ERROR;
			}
			break;
		case TYPE:
			receivedData.Type = recvChar;
			if(receivedData.Type != 0)
			{
				recvState = LENGTH;
			} else
			{
				//Type 0 is not allowed
				recvState = ERROR;
			}
			break;
		case LENGTH:
			receivedData.Length = recvChar;
			receivedData.Data = malloc(receivedData.Length*sizeof(uint8_t));
			if(receivedData.Length != 0)
			{
				recvState = DATA;
			}
			else
			{
				recvState = CRC;
			}
			break;
		case DATA:
			receivedData.Data[CharCounter] = recvChar;
			CharCounter++;
			if(CharCounter >= receivedData.Length)
			{
				CharCounter = 0;
				recvState = CRC;
			}
			break;
		case CRC:
			receivedData.CRC[CharCounter] = recvChar;
			CharCounter++;
			if(CharCounter >= 4)
			{
				CharCounter = 0;
				recvState = END;
			}
			break;
		case END:
			if(recvChar == STOP_CHAR)
			{
				//RECV. Sucsessful
				parseRecvData(receivedData);
				recvInProgress = false;
			} else 
			{
				//ERROR with end char
			}
			recvState = BEGIN;
			//sendDebug("OK");
			break;
		case ERROR:
				//TODO: Handle errors
				sendDebug("ERROR");
			break;
		case TIMEOUT:
				//TODO: Implement timeout timer
				sendDebug("TIMEOUT");
			break;
		default:
				//TODO: Something went horribly wrong
				sendDebug("FATAL ERROR");
			break;
	}
}


void sendDebug(char Text[])
{
	uint8_t Length = strlen(Text);
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
		//readyToSend = false;
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
	if(UARTCOM_ready_to_send(tranData.Length) && Type > 2 && Type <= 154)
	{
		tempData = tranData;			//Store data for retransmission
		transmit_block(tranData);
		readyToSend = false;
		return true;
	}else
	{
		tranData.Type = 0;
		return false;
	}
}
