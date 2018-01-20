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
#include "HelperFunctions.h"
#include "config.h"
#include <stdio.h>

typedef void (*UART_RECV_CALLBACK)(uint8_t* startPtr, uint16_t Length);

StatusCode UART0_init(uint32_t BaudRate, uint32_t RecvLength);
StatusCode UART0_put_data(uint8_t* sendData, uint16_t Length);
StatusCode UART0_puts(char* Text);
StatusCode UART0_set_receiver_length(uint32_t Length);
StatusCode UART0_register_received_callback(UART_RECV_CALLBACK callBack);
bool UART0_has_space();
bool UART0_is_idle();
StatusCode UART0_put_int(int num);
StatusCode UART0_put_float(float num);

StatusCode UART0_put_int_blocking(int num);
StatusCode UART0_puts_blocking(char sendData[]);

void uart0_put_raw_data(uint8_t* sendData, uint16_t Length);



#endif /* UART0_H_ */