/*
 * RCReader.h
 *
 * Created: 29.09.2017 21:28:45
 *  Author: Markus Lorenz
 */ 


#ifndef RCREADER_H_
#define RCREADER_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "HelperFunctions.h"

/*
 *	Configures the Timer 5 with a 4us tick time for
 *	RC PWM input measuring and the appropriate pins
 *	for the PWM input
 */
void rc_init();

/*
 *	Returns the last measured PWM input of throttle
 */
uint16_t rc_read_throttle();

#endif /* RCREADER_H_ */