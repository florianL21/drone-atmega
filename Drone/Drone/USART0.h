/*
 * USART0.h
 *
 * Created: 18.11.2017 19:15:08
 *  Author: flola
 */ 


#ifndef USART0_H_
#define USART0_H_

#include "sam.h"
#include <stdlib.h>
#include <stdbool.h>
#include "HelperFunctions.h"
#include "config.h"

typedef void (*USART_RECV_CALLBACK)(uint8_t* startPtr, uint16_t Length);

void USART0_init(uint32_t BaudRate, uint32_t RecvLength);
bool USART0_put_data(uint8_t* sendData, uint16_t Length);
void USART0_set_receiver_length(uint32_t Length);
void USART0_register_received_callback(USART_RECV_CALLBACK callBack);
bool USART0_has_space();



#endif /* USART0_H_ */