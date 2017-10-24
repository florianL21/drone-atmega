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

volatile listenHandlerData reciveListeners[UARTCOM_MAX_RECIVE_TYPES];
volatile uint8_t numReciveListeners = 0;

void parseRecvData(transmissionData recvData);
void uart_recived_char(uint8_t recvChar);
void transmit_ack();
void transmit_nack();
void transmit_block(transmissionData Data);
transmissionData get_data_block(uint8_t Type, const uint8_t Data[], uint8_t Length);

bool checkCRC(transmissionData tranData)
{
	return true;
}

void generateCRC(transmissionData tranData, uint8_t* CRC)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		CRC[i] = 0;
	}
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
	//static bool recvInProgress = false;
	static uint8_t CharCounter = 0;
	switch(recvState)
	{
		case BEGIN:
			if(recvChar == START_CHAR)
			{
				recvState = TYPE;
				//recvInProgress = true;
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
				//recvInProgress = false;
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

bool UARTCOM_register_listener(uint8_t Type, LISTENER_CALLBACK callBack)
{
	listenHandlerData ListenHandler;
	ListenHandler.Type = Type;
	ListenHandler.CallBack = callBack;
	for(uint8_t i = 0; i < numReciveListeners; i++)
	{
		if(Type == reciveListeners[i].Type)
		{
			return false;
		}
	}
	reciveListeners[numReciveListeners++] = ListenHandler;
	return true;
}

void UARTCOM_sendDebug(char Text[])
{
	uint16_t Length = strlen(Text);
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

bool UARTCOM_ready_to_send()
{
	return readyToSend && uart0_has_space();
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
	generateCRC(tranData, tranData.CRC);
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

void transmit_ack()
{
	if(readyToSend == true)
	{
		transmissionData ackData;
		ackData.Type = ACK_CHAR;
		ackData.Length = 0;
		generateCRC(ackData, ackData.CRC);
		transmit_block(ackData);
	}
}

void transmit_nack()
{
	if(readyToSend == true)
	{
		transmissionData nackData;
		nackData.Type = NACK_CHAR;
		nackData.Length = 0;
		generateCRC(nackData, nackData.CRC);
		transmit_block(nackData);
	}
}

//Functions which interface with the UART module directly are below:
void transmit_block(transmissionData Data)
{
	if(!uart0_has_space())
		return;
	uint8_t* rawData = malloc(sizeof(uint8_t)*(Data.Length + 8));
	rawData[0] = START_CHAR;			//Start
	rawData[1] = Data.Type;				//Type
	rawData[2] = Data.Length;			//Length
	for(uint8_t i = 0; i < Data.Length; i++)
	{
		rawData[i + 3] = Data.Data[i];	//Transmit Data
	}
	for (uint8_t i = 0; i < 4; i++)
	{
		rawData[i + Data.Length + 3] = Data.CRC[i];	//CRC
	}
	rawData[Data.Length + 7] = STOP_CHAR;		//End
	uart0_putData(rawData,Data.Length + 8);
	//readyToSend = false;
}