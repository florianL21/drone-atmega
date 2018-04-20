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

uint16_t rcreader_throttle,rcreader_role,rcreader_pitch,rcreader_yaw,rcreader_gear;
/*
MedianFilter	rcreader_filter_throttle,
				rcreader_filter_role,
				rcreader_filter_pitch,
				rcreader_filter_yaw,
				rcreader_filter_gear;
*/

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
	//TC0->TC_CHANNEL[0].TC_RC = 64000; // note: RC oscillator is around 32kHz
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
	NVIC_SetPriority(RCREADER_PIO_IRQN, ISR_PRIORITY_RCREADER);
	NVIC_EnableIRQ(RCREADER_PIO_IRQN);
	
	
	//Initialize the measurement structure to zero
	ReaderValues.error						= false;
	ReaderValues.Gear.LastMeasuredValue		= 0;
	ReaderValues.Gear.LastState				= 0;
	ReaderValues.Gear.mStartCount			= 0;
	ReaderValues.Pitch.LastMeasuredValue		= 0;
	ReaderValues.Pitch.LastState			= 0;
	ReaderValues.Pitch.mStartCount			= 0;
	ReaderValues.Role.LastMeasuredValue		= 0;
	ReaderValues.Role.LastState				= 0;
	ReaderValues.Role.mStartCount			= 0;
	ReaderValues.Throttle.LastMeasuredValue	= 0;
	ReaderValues.Throttle.LastState			= 0;
	ReaderValues.Throttle.mStartCount		= 0;
	ReaderValues.Yaw.LastMeasuredValue			= 0;
	ReaderValues.Yaw.LastState				= 0;
	ReaderValues.Yaw.mStartCount			= 0;
	/*
	median_filter_new(&rcreader_filter_throttle, FILTER_SIZE, 0);
	median_filter_new(&rcreader_filter_role, FILTER_SIZE, 0);
	median_filter_new(&rcreader_filter_pitch, FILTER_SIZE, 0);
	median_filter_new(&rcreader_filter_yaw, FILTER_SIZE, 0);
	median_filter_new(&rcreader_filter_gear, FILTER_SIZE, 0);
	*/
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

uint16_t calculate_control_value(uint16_t LastMeasuredValue, bool* error, uint16_t RC_ControlMin, uint16_t RC_ControlMax, uint16_t RC_ControlCenter, uint16_t RC_ControlDeadSpot)
{
	if(LastMeasuredValue < RC_ControlMin || LastMeasuredValue > RC_ControlMax) // diff is out of range...
	{
		*error = true;	//and indicate that there was an error
		return RC_ControlCenter;	// ...so return 0 (Center point)
	}
	else
	{
		if(LastMeasuredValue - RC_ControlMin >= RC_ControlCenter - RC_ControlDeadSpot && LastMeasuredValue - RC_ControlMin <= RC_ControlCenter + RC_ControlDeadSpot)
			return RC_ControlCenter;
		else
			return LastMeasuredValue - RC_ControlMin;
	}
}

RemoteControlValues rc_read_values()
{
	RemoteControlValues returnValues;
	returnValues.error = false;
	//Throttle
	uint16_t MeasuredValue = rcreader_throttle;
	if(MeasuredValue < RC_CCONTROL_MIN__THROTTLE || MeasuredValue > RC_CONTROL_MAX__THROTTLE) // is out of range...
	{
		returnValues.Throttle = 0;	// ...so return 0 (min throttle)
		returnValues.error = true;	//and indicate that there was an error
	}
	else
	{
		if(MeasuredValue - RC_CCONTROL_MIN__THROTTLE <= RC_CONTROL_DEAD_SPOT__THROTTLE)
			returnValues.Throttle = 0;
		else
			returnValues.Throttle = MeasuredValue - RC_CCONTROL_MIN__THROTTLE;
	}
	MeasuredValue = rcreader_role;
	returnValues.Roll = calculate_control_value(MeasuredValue, &returnValues.error, RC_CONTROL_MIN__ROLE, RC_CONTROL_MAX__ROLE, RC_CONTROL_CENTER__ROLE, RC_CONTROL_DEAD_SPOT__ROLE);
	MeasuredValue = rcreader_pitch;
	returnValues.Pitch = calculate_control_value(MeasuredValue, &returnValues.error, RC_CONTROL_MIN__PITCH, RC_CONTROL_MAX__PITCH, RC_CONTROL_CENTER__PITCH, RC_CONTROL_DEAD_SPOT__PITCH);
	MeasuredValue = rcreader_yaw;
	returnValues.Yaw = calculate_control_value(ReaderValues.Yaw.LastMeasuredValue, &returnValues.error, RC_CONTROL_MIN__YAW, RC_CONTROL_MAX__YAW, RC_CONTROL_CENTER__YAW, RC_CONTROL_DEAD_SPOT__YAW);
	//Gear:
	MeasuredValue = rcreader_gear;
	if(MeasuredValue < RC_CONTROL_MIN__GEAR || MeasuredValue > RC_CONTROL_MAX__GEAR) // is out of range...
	{
		returnValues.Gear = false;	// ...so return 0 (min throttle)
		returnValues.error = true;	//and indicate that there was an error
	}
	else
	{
		if(MeasuredValue - RC_CONTROL_MIN__GEAR >= RC_CONTROL_THRESCHHOLD__GEAR)
			returnValues.Gear = true;
		else
			returnValues.Gear = false;
	}
	return returnValues;
}

void TC0_Handler(void)
{
	// read status from TC0 status register
	TC0->TC_CHANNEL[0].TC_SR;
}

uint16_t Calculate_measurement(uint32_t status, uint32_t MeasurementPin, MeasurementHelpers* ReadValues)
{
	if (status & MeasurementPin)
	{
		uint32_t PinState = (RCREADER_PIO_PORT->PIO_PDSR & MeasurementPin);
		if(PinState != ReadValues->LastState)
		{	// PIN A8 (PK0) has changed (either high->low or low->high)
			ReadValues->LastState = PinState;
			if(ReadValues->LastState)
			{	// changed from low->high
				ReadValues->mStartCount = TC0->TC_CHANNEL[0].TC_CV;
			}
			else
			{	// changed from high->low
				ReadValues->LastMeasuredValue = calculateTickDifference(ReadValues->mStartCount, TC0->TC_CHANNEL[0].TC_CV);
				
			}
		}
	}
	return ReadValues->LastMeasuredValue;
}

// Pin Change Interrupt
void RCREADER_INTERRUPT(void)
{
	// Save all triggered interrupts
	uint32_t status = RCREADER_PIO_PORT->PIO_ISR;
	rcreader_throttle = Calculate_measurement(status, THROTTLE_PIN, &ReaderValues.Throttle);
	rcreader_role = Calculate_measurement(status, ROLE_PIN, &ReaderValues.Role);
	rcreader_pitch = Calculate_measurement(status, PITCH_PIN, &ReaderValues.Pitch);
	rcreader_yaw = Calculate_measurement(status, YAW_PIN, &ReaderValues.Yaw);
	rcreader_gear = Calculate_measurement(status, GEAR_PIN, &ReaderValues.Gear);
	//TODO: Implement Timeout error
}
