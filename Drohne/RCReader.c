/*
 * RCReader.c
 *
 * Created: 29.09.2017 21:59:22
 *  Author: Markus Lorenz
 */ 

#include "RCReader.h"

volatile uint16_t	throttle_mStartCount = 0;
volatile uint16_t	throttle_lastMeasuredValue = 0;
volatile uint8_t	throttle_lastState = 0;

void rc_init()
{
	// Configure PIN A8 (PK0/PCINT16) as input and activate the internal pull-up
	DDRK &= ~(1 << PK0);
	PORTK |= (1 << PK0);
	
	// Configure Timer 5 in normal mode with a prescaler of 8 for 0.5us tick interval
	TCCR5A = 0x00;
	TCCR5B = 0x00;
	TCCR5B |= (1 << CS51);
	
	// Configure Pin Change Interrupt and Mask PIN A8 (PK0/PCINT16)
	PCMSK2 |= (1 << PCINT16);
	PCICR |= (1 << PCIE2);
}

uint16_t calculateTickDifference(uint16_t start, uint16_t stop)
{
	if(stop > start) // no overflow
		return (stop - start);
	else if(stop < start) // overflow
		return (stop + (65535 - start));
	else if(stop == start) // perfect overflow
		return 65535;
	return 0;
}

void calculateThrottleValue(uint16_t diff)
{
	if(diff < RC_ControlMin || diff > RC_ControlMax) // diff is out of range...
		throttle_lastMeasuredValue = 0; // ...so return 0 (min throttle)
	else
		throttle_lastMeasuredValue = diff - RC_ControlMin;
}

uint16_t rc_read_throttle()
{
	calculateThrottleValue(throttle_lastMeasuredValue);
	return throttle_lastMeasuredValue;
}

// Pin Change Interrupt
ISR(PCINT2_vect)
{
	if((PINK & (1 << PK0)) != throttle_lastState)
	{	// PIN A8 (PK0) has changed (either high->low or low->high)
		throttle_lastState = (PINK & (1 << PK0));
		if(throttle_lastState)
		{	// changed from low->high
			throttle_mStartCount = TCNT5;
		}
		else
		{	// changed from high->low
			uint16_t dT = calculateTickDifference(throttle_mStartCount, TCNT5);
			//calculateThrottleValue(dT);
			throttle_lastMeasuredValue = dT;
		}
	}
	
}