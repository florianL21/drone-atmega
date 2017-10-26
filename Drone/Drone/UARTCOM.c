/*
 * UARTCOM.c
 *
 * Created: 29.09.2017 23:25:27
 *  Author: flola
 */ 
#include "UARTCOM.h"

volatile bool readyToSend = true;
transmissionData* lastTranData; 
bool lastTranDataAdvancedCleanup = false;

recvStates recvState = BEGIN;
transmissionData* receivedData = NULL;

listenHandlerData reciveListeners[UARTCOM_MAX_RECIVE_TYPES];
uint8_t numReciveListeners = 0;

transmissionData *_generate_data_block(uint8_t Type, const uint8_t Data[], uint8_t Length);
bool _check_CRC(transmissionData* tranData);
void _generateCRC(transmissionData* tranData, uint8_t* CRC);
void _transmit_ack();
void _transmit_nack();
void _transmit_block(transmissionData* Data, bool DeepMemmoryCleanRequired);
void _parse_recv_data(transmissionData* recvData);
void _uart_recived_char(uint8_t recvChar);
void _clean_up(transmissionData* recvData, bool deepMemmoryCleanRequired);

bool _check_CRC(transmissionData* tranData)
{
	return true;
}

void _generateCRC(transmissionData* tranData, uint8_t* CRC)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		CRC[i] = 0;
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

void UARTCOM_send_debug(char Text[])
{
	uint16_t Length = strlen(Text);
	_transmit_block(_generate_data_block(1,(uint8_t*)Text,Length),true);
}

void UARTCOM_send_debug_n(uint8_t Number)
{
	_transmit_block(_generate_data_block(2,&Number,1),true);
}

void UARTCOM_init(uint32_t BaudRate)
{
	uart0_init(BaudRate);
	uart0_register_recived_callback(_uart_recived_char);
}

bool UARTCOM_ready_to_send()
{
	return readyToSend && uart0_has_space();
}

transmissionData* _generate_data_block(uint8_t Type, const uint8_t Data[], uint8_t Length)
{
	transmissionData* tranData = malloc(sizeof(*tranData));
	tranData->Length = Length;
	tranData->Type = Type;
	if(tranData->Length == 0)
	{
		tranData->Data = NULL;
	}else
	{
		tranData->Data = malloc(tranData->Length*sizeof(uint8_t));
		if(tranData->Data != NULL)
		{
			for (uint8_t i = 0; i < tranData->Length; i++)
			{
				tranData->Data[i] = Data[i];
			}
		} else
		{
			#ifdef DEBUG_UARTCOM
			UARTCOM_send_debug("generate_data_block: mallock returned NULL");
			#endif
		}
	}
	_generateCRC(tranData, tranData->CRC);
	return tranData;
}


bool UARTCOM_transmit_block(uint8_t Type, const uint8_t Data[], uint8_t Length)
{
	transmissionData* tranData;
	tranData = _generate_data_block(Type,Data,Length);
	if(UARTCOM_ready_to_send())// && Type > 2 && Type <= 154
	{
		_transmit_block(tranData,false);
		readyToSend = false;
		return true;
	}else
	{
		_clean_up(tranData, false);
		tranData->Type = 0;
		return false;
	}
}

void _transmit_ack()
{
	if(readyToSend == true)
	{
		transmissionData* ackData = malloc(sizeof(*ackData));
		ackData->Type = ACK_CHAR;
		ackData->Length = 0;
		_generateCRC(ackData, ackData->CRC);
		_transmit_block(ackData,false);
	}
}

void _transmit_nack()
{
	if(readyToSend == true)
	{
		transmissionData* nackData = malloc(sizeof(*nackData));
		nackData->Type = NACK_CHAR;
		nackData->Length = 0;
		_generateCRC(nackData, nackData->CRC);
		_transmit_block(nackData,false);
	}
}

//interfaces with the UART module directly:
void _transmit_block(transmissionData* Data, bool DeepMemmoryCleanRequired)
{
	if(!uart0_has_space())
	{
		_clean_up(Data,DeepMemmoryCleanRequired);
		return;
	}
	
	uint8_t* rawData = malloc(sizeof(uint8_t)*(Data->Length + 8));
	if(rawData != NULL)
	{
		rawData[0] = START_CHAR;			//Start
		rawData[1] = Data->Type;			//Type
		rawData[2] = Data->Length;			//Length
		if(Data->Data != NULL && Data != NULL)
		{
			for(uint8_t i = 0; i < Data->Length; i++)
			{
				rawData[i + 3] = Data->Data[i];	//Transmit Data
			}
			for (uint8_t i = 0; i < 4; i++)
			{
				rawData[i + Data->Length + 3] = Data->CRC[i];	//CRC
			}
			rawData[Data->Length + 7] = STOP_CHAR;		//End
			uart0_put_data(rawData,Data->Length + 8, true);
			//TODO: create a copy tranData function to manage lastTran data
			//lastTranData = Data;					//Store data for retransmission
			//lastTranDataAdvancedCleanup = DeepMemmoryCleanRequired;
			_clean_up(Data,DeepMemmoryCleanRequired);
		} else {
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("transmit_block: data pointet to NULL");
			#endif
			_clean_up(Data,DeepMemmoryCleanRequired);
			free(rawData);
		}
		
		//readyToSend = false;
	}else
	{
		#ifdef DEBUG_UARTCOM
			UARTCOM_send_debug("transmit_block: malloc returned NULL");
		#endif
		_clean_up(Data,DeepMemmoryCleanRequired);
	}
}


void _clean_up(transmissionData* recvData, bool deepMemmoryCleanRequired)
{
	if(recvData != NULL)
	{
		if(recvData->Length != 0 && deepMemmoryCleanRequired == true && recvData->Data != NULL)
			free(recvData->Data);
		free(recvData);
	}
}



void _parse_recv_data(transmissionData* recvData)
{
	if(_check_CRC(recvData) && recvData != NULL)
	{
		if(recvData->Type == ACK_CHAR && recvData->Length == 0)
		{
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("parse_recv_data: GOT ACK");
			#endif
			
			readyToSend = true;
			_clean_up(recvData, false);
			//lastTranData->Type = 0;
		}else if(recvData->Type == NACK_CHAR && recvData->Length == 0)
		{
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("parse_recv_data: GOT NACK");
			#endif
			/*
			if(lastTranData->Type != 0)
			{
				_transmit_block(lastTranData,lastTranDataAdvancedCleanup);
			}*/
			_clean_up(recvData, false);
		} else if (recvData->Type != 0)
		{
			bool gotMatch = false;
			for(uint8_t i = 0; i < numReciveListeners; i++)
			{
				if(reciveListeners[i].Type == recvData->Type)
				{
					gotMatch = true;
					reciveListeners[i].CallBack(recvData->Type, recvData->Data);
					#ifdef DEBUG_UARTCOM
					UARTCOM_send_debug("parse_recv_data: Type match: ");
					UARTCOM_send_debug_n(recvData->Type);
					#endif
					_clean_up(recvData,false);
					break;
				}
			}
			if(gotMatch == false)
			{
				_clean_up(recvData,true);
				#ifdef DEBUG_UARTCOM
					UARTCOM_send_debug("parse_recv_data: Type Missmatch: ");
					UARTCOM_send_debug_n(recvData->Type);
				#endif
			}
		}else
		{
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("parse_recv_data: Type was 0");
			#endif
			_clean_up(recvData,true);
		}
	} else
	{
		if(recvData == NULL)
		{
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("parse_recv_data: Data pointet to NULL");
			#endif
		}else{
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("parse_recv_data: CRC Check Error");
			#endif
			_clean_up(recvData,true);
		}
	}
}

void _uart_recived_char(uint8_t recvChar)
{
	//static bool recvInProgress = false;
	static uint8_t CharCounter = 0;
	switch(recvState)
	{
		case BEGIN:
			if(recvChar == START_CHAR)
			{
				receivedData = malloc(sizeof(*receivedData));
				recvState = TYPE;
				//recvInProgress = true;
			} else
			{
				//wrong start char
				recvState = ERROR;
			}
			break;
		case TYPE:
			receivedData->Type = recvChar;
			if(receivedData->Type != 0)
			{
				recvState = LENGTH;
			} else
			{
				//Type 0 is not allowed
				recvState = ERROR;
			}
			break;
		case LENGTH:
			receivedData->Length = recvChar;
			if(receivedData->Length != 0)
			{
				receivedData->Data = malloc(receivedData->Length*sizeof(uint8_t));
				if(receivedData->Data != 0)
				{
					recvState = DATA;
				}else
				{
					recvState = ERROR;
				}
			} else {
				recvState = CRC;
			}
			break;
		case DATA:
			receivedData->Data[CharCounter] = recvChar;
			CharCounter++;
			if(CharCounter >= receivedData->Length)
			{
				CharCounter = 0;
				recvState = CRC;
			}
			break;
		case CRC:
			receivedData->CRC[CharCounter] = recvChar;
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
				#ifdef DEBUG_UARTCOM
					UARTCOM_send_debug("uart_recived_char: RECV. OK");
				#endif
				//RECV. Sucsessful
				_parse_recv_data(receivedData);
				//recvInProgress = false;
				recvState = BEGIN;
				
			} else
			{
				//ERROR with end char
				recvState = ERROR;
			}
			break;
		case ERROR:
			_clean_up(receivedData,true);
			//TODO: Handle errors
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("uart_recived_char: ERROR");
			#endif
			break;
		case TIMEOUT:
			_clean_up(receivedData,true);
			//TODO: Implement timeout timer
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("uart_recived_char: TIMEOUT");
			#endif
			break;
		default:
			_clean_up(receivedData,true);
			//TODO: Something went horribly wrong
			#ifdef DEBUG_UARTCOM
				UARTCOM_send_debug("uart_recived_char: FATAL ERROR");
			#endif
			break;
	}
}