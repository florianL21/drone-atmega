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
#include "../HelperFunctions/HelperFunctions.h"
#include "../../config.h"
#include "../ErrorHandling/ErrorHandling.h"

typedef void (*USART_RECV_CALLBACK)(uint8_t* startPtr, uint16_t Length);

ErrorCode USART0_init(uint32_t BaudRate, uint32_t RecvLength);
ErrorCode USART0_put_data(uint8_t* sendData, uint16_t Length);
ErrorCode USART0_set_receiver_length(uint32_t Length);
ErrorCode USART0_register_received_callback(USART_RECV_CALLBACK callBack);
bool USART0_has_space();
bool USART0_is_idle();


#endif /* USART0_H_ */