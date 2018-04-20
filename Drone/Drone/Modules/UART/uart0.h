/*
 * UART0.h
 *
 * Created: 18.11.2017 19:15:08
 *  Author: flola
 */ 


#ifndef UART0_H_
#define UART0_H_

#include "sam.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "../HelperFunctions/HelperFunctions.h"
#include "../../config.h"
#include <stdio.h>
#include "../ErrorHandling/ErrorHandling.h"
#include "../GPT/GPT.h"

typedef void (*UART_RECV_CALLBACK)(uint8_t* startPtr, uint16_t Length);

ErrorCode UART0_init(uint32_t BaudRate, uint32_t RecvLength);
ErrorCode UART0_put_data(uint8_t* sendData, uint16_t Length);
ErrorCode UART0_puts(char* Text);
ErrorCode UART0_set_receiver_length(uint32_t Length);
ErrorCode UART0_register_received_callback(UART_RECV_CALLBACK callBack);
bool UART0_has_space();
bool UART0_is_idle();
uint8_t UART0_get_space();
ErrorCode UART0_put_int(int num);
ErrorCode UART0_put_float(float num);


ErrorCode UART0_put_int_blocking(int num);
ErrorCode UART0_puts_blocking(char sendData[]);

void uart0_put_raw_data(uint8_t* sendData, uint16_t Length);



#endif /* UART0_H_ */