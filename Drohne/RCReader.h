/*
 * RCReader.h
 *
 * Created: 29.09.2017 21:28:45
 *  Author: Markus Lorenz
 */ 


#ifndef RCREADER_H_
#define RCREADER_H_

#include <avr/io.h>

/*
 *	Configures the Timer 5 with a 4us tick time for
 *	RC PWM input measuring
 */
void rc_init();

/*
 *	Returns the last measured PWM input of throttle
 */
uint16_t rc_read_throttle();

#endif /* RCREADER_H_ */