/*
 * GPT.h
 *
 * Created: 03.04.2018 17:33:25
 *  Author: flola
 */ 


#ifndef GPT_H_
#define GPT_H_

#include "sam.h"
#include "config.h"
#include <stdbool.h>
#include <stdio.h>

typedef void (*GPT_TIMER_CALLBACK)(void);
typedef uint8_t Timer;

/*
	This function initializes the GPT module.
*/
void GPT_Init();

/*
	This function returns the current ms count since controller start.
	ATTENTION: This function is not overflow safe!!! Runtime before overflow: 7 weeks
	returns:
		Time since start in ms
*/
uint32_t GPT_GetTime();

/*
	This function returns the current ms count since controller start.
	ATTENTION: This function is not overflow safe!!! Runtime before overflow: 7 weeks
	returns:
		Time since start in ms
*/
float GPT_GetPreciseTime();

/*
	This function stops the program for a defined time
	Parameters:
		- Delay_in_ms: Time to wait in ms
*/
void GPT_Delay(uint16_t Delay_in_ms);

#endif /* GPT_H_ */