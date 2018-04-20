/*
 * WDT_config.h
 *
 * Created: 24.03.2018 08:17:43
 *  Author: flola
 */ 


#ifndef WDT_CONFIG_H_
#define WDT_CONFIG_H_

#include "sam.h"

/*
	Initializes the wdt timer.
	IMPORTANT:	This function can only be called once. Changes can only be made after a Processor reset! 
				If it gets called more than once only the first call is executed and all others are ignored without any error indication.
				If the WDT_disable() function was called before, this has no effect!!!
	Parameters:
		- wdt_count_register:	Defines the wdt count register value
*/
void WDT_init(uint16_t wdt_count_register);

/*
	Initializes the wdt timer. and disables it
	IMPORTANT:	This function can only be called once. Changes can only be made after a Processor reset! 
				If it gets called more than once only the first call is executed and all others are ignored without any error indication.
				If the WDT_init() function was called before, this has no effect!!!
*/
void WDT_disable();

/*
	This function restarts the wdt timer. It should be called when the wdt timer is not disabled to prevent a processor reset.
	If it is not called within the time period defined at the init function the processor will reset!
*/
void WDT_restart();



#endif /* WDT_CONFIG_H_ */