/*
 * RCReader.h
 *
 * Created: 29.09.2017 21:28:45
 *  Author: Markus Lorenz
 */ 


#ifndef RCREADER_H_
#define RCREADER_H_

#include "sam.h"
#include "config.h"
#include <stdbool.h>
//#include "HelperFunctions.h"

struct RemoteControlValues
{
	uint16_t Throttle;
	uint16_t Role;
	uint16_t Pitch;
	uint16_t Yaw;
	bool Gear;
	bool error;
};
typedef struct RemoteControlValues RemoteControlValues;

/*
 *	Configures the Timer 5 with a 4us tick time for
 *	RC PWM input measuring and the appropriate pins
 *	for the PWM input
 */
void rc_init();

/*
 *	Returns the last measured PWM input of throttle
 */
RemoteControlValues rc_read_values();

#endif /* RCREADER_H_ */