/*
 * PID.c
 *
 * Created: 23.08.2016 12:44:19
 *  Author: Michael
 */ 


#include "PID.h"

uint32_t calculateTicks(uint32_t start, uint32_t stop)
{
	if(stop > start) // no overflow
		return (stop - start);
	else if(stop < start) // overflow
		return (stop + (4294967295 - start));
	else if(stop == start) // perfect overflow
		return 4294967295;
	return 0;
}

uint32_t getTicks()
{
	return TC0->TC_CHANNEL[0].TC_CV;
}

void PID_Init()
{
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
}

//struct pidData myPidData[NUM_PID_CONTROLLERS];

PID_Initialize(pidData* pidController, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, float Min, float Max, float SampleTime)
{
	pidController->Input = Input;
	pidController->Output = Output;
	pidController->Setpoint = Setpoint;
	
	pidController->lastInput = *(pidController->Input);
	pidController->ITerm = *(pidController->Output);
	
	PID_SetOutputLimits(pidController, Min, Max);
	
	if(pidController->ITerm > pidController->outMax)
		pidController->ITerm = pidController->outMax;
	else if(pidController->ITerm < pidController->outMin)
		pidController->ITerm = pidController->outMin;
	
	pidController->SampleTime = SampleTime*(TICKS_PER_SECOND / 1000);
	pidController->LastTime = getTicks() - pidController->SampleTime;
	
	PID_SetTunings(pidController, Kp, Ki, Kd);
	
	PID_SetControllerDirection(pidController, DIRECT);
}

bool PID_need_compute(pidData* pidController)
{
	if(pidController==NULL)
		return false;
	return calculateTicks(pidController->LastTime,getTicks()) >= pidController->SampleTime;
}

void PID_Compute(pidData* pidController)
{
	if(pidController==NULL)
		return;
	if(calculateTicks(pidController->LastTime,getTicks()) >= pidController->SampleTime)
	{
		/*Compute all the working error variables*/
		float input = *(pidController->Input);
		float error = (*(pidController->Setpoint) - input);
		
		pidController->ITerm += (pidController->ki * error);
		
		if(pidController->ITerm > pidController->outMax)
			pidController->ITerm = pidController->outMax;
		else if(pidController->ITerm < pidController->outMin)
			pidController->ITerm = pidController->outMin;
		
		float dInput = (input - pidController->lastInput);
		
		/*Compute PID Output*/
		float output = pidController->kp * error + pidController->ITerm - pidController->kd * dInput;
		
		if(output > pidController->outMax)
			output = pidController->outMax;
		else if(output < pidController->outMin)
			output = pidController->outMin;
		
		*(pidController->Output) = output;
		
		/*Remember some variables for next time*/
		pidController->lastInput = input;
		pidController->LastTime=getTicks();
	}
}

void PID_SetTunings(pidData* pidController, float Kp, float Ki, float Kd)
{
	if(pidController==NULL)
		return;
	if (Kp < 0 || Ki < 0 || Kd < 0)
		return;
	
	float SampleTimeInSec = (float)(pidController->SampleTime) / ((float) TICKS_PER_SECOND);
	pidController->kp = Kp;
	pidController->ki = Ki * SampleTimeInSec;
	pidController->kd = Kd / SampleTimeInSec;
	
	if(pidController->controllerDirection == REVERSE)
	{
		pidController->kp = (0 - Kp);
		pidController->ki = (0 - Ki);
		pidController->kd = (0 - Kd);
	}
}

void PID_SetSampleTime(pidData* pidController, float NewSampleTime)
{
	if(pidController==NULL)
		return;
	if (NewSampleTime > 0)
	{
		float ratio  = (float)(NewSampleTime* (TICKS_PER_SECOND / 1000)) / (float)(pidController->SampleTime);
		
		pidController->ki *= ratio;
		pidController->kd /= ratio;
		pidController->SampleTime = (uint32_t)NewSampleTime * (TICKS_PER_SECOND / 1000);
	}
}

void PID_SetOutputLimits(pidData* pidController, float Min, float Max)
{
	if(pidController==NULL)
		return;
	if(Min > Max) return;
	
	pidController->outMin = Min;
	pidController->outMax = Max;
	
	if(*(pidController->Output) > pidController->outMax)
		*(pidController->Output) = pidController->outMax;
	else if(*(pidController->Output) < pidController->outMin)
		*(pidController->Output) = pidController->outMin;
	
	if(pidController->ITerm > pidController->outMax)
		pidController->ITerm = pidController->outMax;
	else if(pidController->ITerm < pidController->outMin)
		pidController->ITerm = pidController->outMin;
}

void PID_SetControllerDirection(pidData* pidController, int Direction)
{
	if(pidController==NULL)
		return;
	if(Direction != pidController->controllerDirection)
	{
		pidController->kp = (0 - pidController->kp);
		pidController->ki = (0 - pidController->ki);
		pidController->kd = (0 - pidController->kd);
	}
	
	pidController->controllerDirection = Direction;
}