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

volatile listenHandlerData reciveListeners[MAX_RECIVE_TYPES];
volatile uint8_t numReciveListeners = 0;

void parseRecvData(transmissionData recvData);
void uart_recived_char(uint8_t recvChar);
void transmit_ack();
void transmit_nack();
void transmit_block(transmissionData Data);
transmissionData get_data_block(uint8_t Type, const uint8_t Data[], uint8_t Length);
bool enough_buffer_space(uint8_t dataLength);

bool checkCRC(transmissionData tranData)
{
	return true;
}

uint8_t* generateCRC(transmissionData tranData)
{
	static uint8_t CRC[4] = {0};
	return CRC;
}

void parseRecvData(transmissionData recvData)
{
	if(checkCRC(recvData))
	{
		if(recvData.Type == ACK_CHAR && recvData.Length == 0)
		{
			#ifdef DEBUG_UARTCOM
				UARTCOM_sendDebug("GOT ACK");
			#endif
			readyToSend = true;
			tempData.Type = 0;
		}else if(recvData.Type == NACK_CHAR && recvData.Length == 0)
		{
			#ifdef DEBUG_UARTCOM
				UARTCOM_sendDebug("GOT NACK");
			#endif
			if(tempData.Type != 0)
			{
				transmit_block(tempData);
			}
		} else if (recvData.Type != 0)
		{
			bool gotMatch = false;
			for(uint8_t i = 0; i < numReciveListeners; i++)
			{
				if(reciveListeners[i].Type == recvData.Type)
				{
					gotMatch = true;
					reciveListeners[i].CallBack(recvData.Type, recvData.Data);
					break;
				}
			}
			if(gotMatch == false)
			{
				#ifdef DEBUG_UARTCOM
					UARTCOM_sendDebug("Type Missmatch");
				#endif
			}

		}else
		{
			#ifdef DEBUG_UARTCOM
				UARTCOM_sendDebug("Type was NULL");
			#endif
		}
	} else
	{
		#ifdef DEBUG_UARTCOM
			UARTCOM_sendDebug("CRC Check Error");
		#endif
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
			#ifdef DEBUG_UARTCOM
				UARTCOM_sendDebug("RECV. OK");
			#endif
			break;
		case ERROR:
				//TODO: Handle errors
				#ifdef DEBUG_UARTCOM
					UARTCOM_sendDebug("ERROR");
				#endif
			break;
		case TIMEOUT:
				//TODO: Implement timeout timer
				#ifdef DEBUG_UARTCOM
					UARTCOM_sendDebug("TIMEOUT");
				#endif
			break;
		default:
				//TODO: Something went horribly wrong
				#ifdef DEBUG_UARTCOM
					UARTCOM_sendDebug("FATAL ERROR");
				#endif
			break;
	}
}

bool UARTCOM_register_listener(uin8_t Type, LISTENER_CALLBACK callBack)
{
	listenHandlerData ListenHandler;
	ListenHandler.Type = Type;
	ListenHandler.CallBack = callBack;
	for(uint8_t i = 0; i < numReciveListeners; i++)
	{
		if(Type == reciveListeners[i])
		{
			return false;
		}
	}
	reciveListeners[numReciveListeners++] = ListenHandler;
	return true;
}

void UARTCOM_sendDebug(char Text[])
{
	uint8_t Length = strlen(Text);
	transmit_block(get_data_block(1,Text,Length));
}

void UARTCOM_sendDebug_n(uint8_t Number)
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
	tranData.CRC = generateCRC(tranData);
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
