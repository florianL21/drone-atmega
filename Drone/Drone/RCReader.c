/*
 * RCReader.c
 *
 * Created: 29.09.2017 21:59:22
 *  Author: Markus Lorenz
 */ 

#include "RCReader.h"

struct MeasurementHelpers
{
	uint32_t LastState;
	uint32_t mStartCount;
	uint16_t LastMeasuredValue;
};
typedef struct MeasurementHelpers MeasurementHelpers;

struct
{
	MeasurementHelpers Throttle;
	MeasurementHelpers Role;
	MeasurementHelpers Pitch;
	MeasurementHelpers Yaw;
	MeasurementHelpers Gear;
	bool error;
}ReaderValues;


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
	
	// Configure Pin Change Interrupts for all needed pins
	// Enable Clock for PIOB - needed for sampling falling edge
	PMC->PMC_PCER0 = 1 << RCREADER_PIO_ID;
	// Enable IO pin control
	RCREADER_PIO_PORT->PIO_PER = RCREADER_ENABLED_PINS;
	// Disable output (set to High Z)
	RCREADER_PIO_PORT->PIO_ODR = RCREADER_ENABLED_PINS;
	// Enable pull-up
	RCREADER_PIO_PORT->PIO_PUER = RCREADER_ENABLED_PINS;
	// The interrupt source is an Edge detection event.
	RCREADER_PIO_PORT->PIO_ESR = RCREADER_ENABLED_PINS;
	// The interrupt source is set to a falling and rising Edge detection
	RCREADER_PIO_PORT->PIO_FELLSR = RCREADER_ENABLED_PINS;
	RCREADER_PIO_PORT->PIO_REHLSR = RCREADER_ENABLED_PINS;
	// Enables the Input Change Interrupt on the I/O line.
	RCREADER_PIO_PORT->PIO_IER = RCREADER_ENABLED_PINS;
	// Enable Interrupt Handling in NVIC
	NVIC_EnableIRQ(RCREADER_PIO_IRQN);
	
	//Initialize the measurement structure to zero
	ReaderValues.error						= false;
	ReaderValues.Gear.LastMeasuredValue		= 0;
	ReaderValues.Gear.LastState				= 0;
	ReaderValues.Gear.mStartCount			= 0;
	ReaderValues.Pitch.LastMeasuredValue	= 0;
	ReaderValues.Pitch.LastState			= 0;
	ReaderValues.Pitch.mStartCount			= 0;
	ReaderValues.Role.LastMeasuredValue		= 0;
	ReaderValues.Role.LastState				= 0;
	ReaderValues.Role.mStartCount			= 0;
	ReaderValues.Throttle.LastMeasuredValue	= 0;
	ReaderValues.Throttle.LastState			= 0;
	ReaderValues.Throttle.mStartCount		= 0;
	ReaderValues.Yaw.LastMeasuredValue		= 0;
	ReaderValues.Yaw.LastState				= 0;
	ReaderValues.Yaw.mStartCount			= 0;
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

RemoteControlValues rc_read_values()
{
	RemoteControlValues returnValues;
	//Throttle
	if(ReaderValues.Throttle.LastMeasuredValue < RC_ControlMin || ReaderValues.Throttle.LastMeasuredValue > RC_ControlMax) // diff is out of range...
	{
		returnValues.Throttle = 0;	// ...so return 0 (min throttle)
		returnValues.error = true;	//and indicate that there was an error
	}
	else
	{
		returnValues.Throttle = ReaderValues.Throttle.LastMeasuredValue - RC_ControlMin;
	}
	
	//Role
	if(ReaderValues.Role.LastMeasuredValue < RC_ControlMin || ReaderValues.Role.LastMeasuredValue > RC_ControlMax) // diff is out of range...
	{
		returnValues.Role = 0;	// ...so return 0 (min throttle)
		returnValues.error = true;	//and indicate that there was an error
	}
	else
	{
		returnValues.Role = ReaderValues.Role.LastMeasuredValue - RC_ControlMin;
	}
	//TODO:Manage all the other control values
	
	return returnValues;
}

void TC0_Handler(void)
{
	// read status from TC0 status register
	TC0->TC_CHANNEL[0].TC_SR;
}

// Pin Change Interrupt
void RCREADER_INTERRUPT(void)
{
	// Save all triggered interrupts
	uint32_t status = RCREADER_PIO_PORT->PIO_ISR;
	if (status & THROTTLE_PIN)
	{
		uint32_t PinState = (RCREADER_PIO_PORT->PIO_PDSR & THROTTLE_PIN);
		if(PinState != ReaderValues.Throttle.LastState)
		{	// PIN A8 (PK0) has changed (either high->low or low->high)
			ReaderValues.Throttle.LastState = PinState;
			if(ReaderValues.Throttle.LastState)
			{	// changed from low->high
				ReaderValues.Throttle.mStartCount = TC0->TC_CHANNEL[0].TC_CV;
			}
			else
			{	// changed from high->low
				ReaderValues.Throttle.LastMeasuredValue = calculateTickDifference(ReaderValues.Throttle.mStartCount, TC0->TC_CHANNEL[0].TC_CV);
			}
		}
	}
	if (status & ROLE_PIN)
	{
		uint32_t PinState = (RCREADER_PIO_PORT->PIO_PDSR & ROLE_PIN);
		if(PinState != ReaderValues.Role.LastState)
		{	// PIN A8 (PK0) has changed (either high->low or low->high)
			ReaderValues.Role.LastState = PinState;
			if(ReaderValues.Role.LastState)
			{	// changed from low->high
				ReaderValues.Role.mStartCount = TC0->TC_CHANNEL[0].TC_CV;
			}
			else
			{	// changed from high->low
				ReaderValues.Role.LastMeasuredValue = calculateTickDifference(ReaderValues.Role.mStartCount, TC0->TC_CHANNEL[0].TC_CV);
			}
		}
	}
}