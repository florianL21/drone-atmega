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

void USART0_init(uint32_t BaudRate);
void USART0_put_raw_data(uint8_t* sendData, uint16_t Length);



#endif /* USART0_H_ */