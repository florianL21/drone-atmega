/*
 * RCReader.c
 *
 * Created: 29.09.2017 21:59:22
 *  Author: Markus Lorenz
 */ 

#include "RCReader.h"

volatile uint32_t	throttle_mStartCount = 0;
volatile uint32_t	throttle_lastMeasuredValue = 0;
volatile uint8_t	throttle_lastState = 0;


void rc_init()
{
	// Configure PIN A8 (PK0/PCINT16) as input and activate the internal pull-up
	//DDRK &= ~(1 << PK0);
	//PORTK |= (1 << PK0);
	
	// Configure Timer 5 in normal mode with a prescaler of 8 for 0.5us tick interval
	// Enable TC0 (27 is TC0)
	PMC->PMC_PCER0 = 1 << ID_TC0;
	// Disable TC clock
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
	// Disable interrupts
	TC0->TC_CHANNEL[0].TC_IDR = 0xFFFFFFFF;
	// Clear status register
	TC0->TC_CHANNEL[0].TC_SR;
	// Set TC0 Mode: Capture mode Clock3 (84Mhz/32)
	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3;
	// Set Compare Value in RC register
	TC0->TC_CHANNEL[0].TC_RC = 64000; // note: RC oscillator is around 32kHz
	// Reset counter (SWTRG) and start counter clock (CLKEN)
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
	
	// Configure Pin Change Interrupt and Mask PIN A8 (PK0/PCINT16)
	//PCMSK2 |= (1 << PCINT16);
	//PCICR |= (1 << PCIE2);
}

uint32_t calculateTickDifference(uint32_t start, uint32_t stop)
{
	if(stop > start) // no overflow
		return (stop - start);
	else if(stop < start) // overflow
		return (stop + (4294967295 - start));
	else if(stop == start) // perfect overflow
		return 4294967295;
	return 0;
}

void calculateThrottleValue(uint32_t diff)
{
	if(diff < RC_ControlMin || diff > RC_ControlMax) // diff is out of range...
		throttle_lastMeasuredValue = 0; // ...so return 0 (min throttle)
	else
		throttle_lastMeasuredValue = diff - RC_ControlMin;
}

uint32_t rc_read_throttle()
{
	calculateThrottleValue(throttle_lastMeasuredValue);
	return throttle_lastMeasuredValue;
}

void TC0_Handler(void)
{
	// read status from TC0 status register
	TC0->TC_CHANNEL[0].TC_SR;
}

// Pin Change Interrupt
/*
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
	
}*/