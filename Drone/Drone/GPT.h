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
	Returns:
		Time since start in ms
*/
float GPT_GetPreciseTime();

/*
	This function stops the program for a defined time
	Parameters:
		- Delay_in_ms: Time to wait in ms
*/
void GPT_Delay(uint16_t Delay_in_ms);


/*
	This function initializes a new General purpose Timer to call an function periodically
	Parameters:
		- timeInMs: Period to call the function
		- callback: What function should be called?
		- enabled: specifies if the function should be enabled after the setup process right away
	Returns:
		0 if all timers are in use or the number of the timer if there was space.
*/
Timer GPT_TimerSetup(uint16_t timeInMs, GPT_TIMER_CALLBACK callback, bool enabled);

/*
	This function changes the intervall of a given timer
	Parameters:
		- timerNum: number of the timer which should be changed
		- timeInMs: new period time
*/
void GPT_TimerSetTime(Timer TimerNum, uint16_t timeInMs);

/*
	This changes if the timer should run
	Parameters:
		- timerNum: number of the timer which should be changed
		- timeInMs: true to enable
*/
void GPT_TimerSetEnabled(Timer TimerNum, bool enabled);

/*
	This deletes a timer if it is no longer used
	Parameters:
		- timerNum: number of the timer which should be changed
*/
void GPT_TimerDelete(Timer TimerNum);



#endif /* GPT_H_ */