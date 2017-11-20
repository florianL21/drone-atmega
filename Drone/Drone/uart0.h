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

typedef void (*UART_RECV_CALLBACK)(uint8_t* startPtr, uint16_t Length);

ERROR_CODE UART0_init(uint32_t BaudRate, uint32_t RecvLength);
ERROR_CODE UART0_put_data(uint8_t* sendData, uint16_t Length);
ERROR_CODE UART0_puts(char* Text);
ERROR_CODE UART0_set_receiver_length(uint32_t Length);
ERROR_CODE UART0_register_received_callback(UART_RECV_CALLBACK callBack);
bool UART0_has_space();
bool UART0_is_idle();



#endif /* UART0_H_ */