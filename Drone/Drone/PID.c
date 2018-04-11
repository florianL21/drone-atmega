/*
 * PID.c
 *
 * Created: 23.08.2016 12:44:19
 *  Author: Michael
 */ 


#include "PID.h"

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

ErrorCode PID_Initialize(pidData* pidController, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, float Min, float Max, float SampleTime)
{
	if(pidController == NULL || Input == NULL || Output == NULL || Setpoint == NULL)
		return MODULE_PID | FUNCTION_Initialize | ERROR_GOT_NULL_POINTER;
	if(Kp < 0 || Ki < 0 || Kd < 0 || SampleTime < 0)
		return MODULE_PID | FUNCTION_Initialize | ERROR_ARGUMENT_OUT_OF_RANGE;
	if(Min > Max) 
		return MODULE_PID | FUNCTION_Initialize | ERROR_INVALID_ARGUMENT;

	pidController->Input = Input;
	pidController->Output = Output;
	pidController->Setpoint = Setpoint;
	
	pidController->lastInput = *(pidController->Input);
	pidController->ITerm = *(pidController->Output);
	
	DEFAULT_ERROR_HANDLER(PID_SetOutputLimits(pidController, Min, Max), MODULE_PID, FUNCTION_Initialize);
	
	if(pidController->ITerm > pidController->outMax)
		pidController->ITerm = pidController->outMax;
	else if(pidController->ITerm < pidController->outMin)
		pidController->ITerm = pidController->outMin;
	
	pidController->SampleTime = SampleTime;
	pidController->LastTime = GPT_GetPreciseTime() - pidController->SampleTime;
	
	DEFAULT_ERROR_HANDLER(PID_SetTunings(pidController, Kp, Ki, Kd), MODULE_PID, FUNCTION_Initialize);
	
	DEFAULT_ERROR_HANDLER(PID_SetControllerDirection(pidController, DIRECT), MODULE_PID, FUNCTION_Initialize);
	return SUCCESS;
}

bool PID_need_compute(pidData* pidController)
{
	if(pidController == NULL)
		return false;
	return (GPT_GetPreciseTime() - pidController->LastTime) >= pidController->SampleTime;
}

ErrorCode PID_Compute(pidData* pidController)
{
	if(pidController == NULL)
		return MODULE_PID | FUNCTION_Compute | ERROR_GOT_NULL_POINTER;
	if(PID_need_compute(pidController) == true)
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
		pidController->LastTime = GPT_GetPreciseTime();
	}
	return SUCCESS;
}

void PID_Reset(pidData* pidController)
{
	pidController->lastInput = 0;
	pidController->ITerm = 0;
}

ErrorCode PID_SetTunings(pidData* pidController, float Kp, float Ki, float Kd)
{
	if(pidController == NULL)
		return MODULE_PID | FUNCTION_SetTunings | ERROR_GOT_NULL_POINTER;
	if (Kp < 0 || Ki < 0 || Kd < 0)
		return MODULE_PID | FUNCTION_SetTunings | ERROR_ARGUMENT_OUT_OF_RANGE;
	
	float SampleTimeInSec = pidController->SampleTime * 100.0;
	pidController->kp = Kp;
	pidController->ki = Ki * SampleTimeInSec;
	pidController->kd = Kd / SampleTimeInSec;
	
	PID_Reset(pidController);
	
	if(pidController->controllerDirection == REVERSE)
	{
		pidController->kp = (0 - Kp);
		pidController->ki = (0 - Ki);
		pidController->kd = (0 - Kd);
	}
	return SUCCESS;
}

ErrorCode PID_SetSampleTime(pidData* pidController, float NewSampleTime)
{
	if(pidController == NULL)
		return MODULE_PID | FUNCTION_SetSampleTime | ERROR_GOT_NULL_POINTER;
	if (NewSampleTime > 0)
	{
		float ratio  = NewSampleTime / pidController->SampleTime;
		
		pidController->ki *= ratio;
		pidController->kd /= ratio;
		pidController->SampleTime = NewSampleTime;
		return SUCCESS;
	}
	return MODULE_PID | FUNCTION_SetSampleTime | ERROR_ARGUMENT_OUT_OF_RANGE;
}

ErrorCode PID_SetOutputLimits(pidData* pidController, float Min, float Max)
{
	if(pidController==NULL)
		return MODULE_PID | FUNCTION_SetOutputLimits | ERROR_GOT_NULL_POINTER;
	if(Min > Max) 
		return MODULE_PID | FUNCTION_SetOutputLimits | ERROR_INVALID_ARGUMENT;
	
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
	return SUCCESS;
}

ErrorCode PID_SetControllerDirection(pidData* pidController, uint8_t Direction)
{
	if(pidController == NULL)
		return MODULE_PID | FUNCTION_SetControllerDirection | ERROR_GOT_NULL_POINTER;
	if(Direction != 0 && Direction != 1)
		return MODULE_PID | FUNCTION_SetControllerDirection | ERROR_ARGUMENT_OUT_OF_RANGE;

	if(Direction != pidController->controllerDirection)
	{
		pidController->kp = (0 - pidController->kp);
		pidController->ki = (0 - pidController->ki);
		pidController->kd = (0 - pidController->kd);
	}
	
	pidController->controllerDirection = Direction;
	return SUCCESS;
}