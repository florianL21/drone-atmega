#include "PID.h"


void PID_Initialize(PID_Controller* aPID);


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

/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

/*********************************************************************
 * 
 * Has to be called once before the PID Controllers can be used.
 * 
 *********************************************************************/
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

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/

void PID_delete(PID_Controller* aPID)
{
	if(aPID!=NULL)
	{
		free(aPID);
	}
}


PID_Controller* PID_newPID(int32_t* Input, int32_t* Output, int32_t* Setpoint, int32_t Kp, int32_t Ki, int32_t Kd, int8_t POn, int8_t ControllerDirection)
{
    PID_Controller *aPID = malloc(sizeof(*aPID));
	if(aPID==NULL)
		return NULL;
    aPID->myOutput = Output;
    aPID->myInput = Input;
    aPID->mySetpoint = Setpoint;
    aPID->inAuto = false;

    PID_SetOutputLimits(aPID, 0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    aPID->SampleTime = 100;							//default Controller Sample Time is 0.01 seconds

    PID_SetControllerDirection(aPID, ControllerDirection);
    PID_SetTunings(aPID, Kp, Ki, Kd, POn);

    aPID->lastTime = TC0->TC_CHANNEL[0].TC_CV - (aPID->SampleTime/TIMER_CONVERSION_FACTOR);
    return aPID;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time the while loop executes.(can also be called in a periodic timer interrupt)
 *   the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID_Compute(PID_Controller* aPID)
{
	if(aPID==NULL)
		return false;
	if(!aPID->inAuto) 
		return false;
	unsigned long now = TC0->TC_CHANNEL[0].TC_CV;
	if(calculateTicks(now, aPID->lastTime)*TIMER_CONVERSION_FACTOR >= aPID->SampleTime)
	{
		/*Compute all the working error variables*/
		int32_t input = *aPID->myInput;
		int32_t error = *aPID->mySetpoint - input;
		int32_t dInput = (input - aPID->lastInput);
		aPID->outputSum += (aPID->ki * error);

		/*Add Proportional on Measurement, if P_ON_M is specified*/
		if(!aPID->pOnE) 
			aPID->outputSum -= aPID->kp * dInput;

		if(aPID->outputSum > aPID->outMax) 
			aPID->outputSum = aPID->outMax;
		else if(aPID->outputSum < aPID->outMin) 
			aPID->outputSum = aPID->outMin;

		/*Add Proportional on Error, if P_ON_E is specified*/
		int32_t output;
		if(aPID->pOnE) 
			output = aPID->kp * error;
		else 
			output = 0;

		/*Compute Rest of PID Output*/
		output += aPID->outputSum - aPID->kd * dInput;

		if(output > aPID->outMax) 
			output = aPID->outMax;
		else if(output < aPID->outMin) 
			output = aPID->outMin;
		*aPID->myOutput = output;

		/*Remember some variables for next time*/
		aPID->lastInput = input;
		aPID->lastTime = now;
		return true;
	}
	else 
		return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID_SetTunings(PID_Controller* aPID, int32_t Kp, int32_t Ki, int32_t Kd, int16_t POn)
{
	if(aPID==NULL)
		return;
	if (Kp < 0 || Ki < 0 || Kd < 0) 
		return;

	aPID->pOn = POn;
	aPID->pOnE = POn == P_ON_E;

	aPID->dispKp = Kp; 
	aPID->dispKi = Ki; 
	aPID->dispKd = Kd;

	int32_t SampleTimeInSec = ((int32_t)aPID->SampleTime)/1000;
	aPID->kp = Kp;
	aPID->ki = Ki * SampleTimeInSec;
	aPID->kd = Kd / SampleTimeInSec;

	if(aPID->controllerDirection == REVERSE)
	{
		aPID->kp = (0 - aPID->kp);
		aPID->ki = (0 - aPID->ki);
		aPID->kd = (0 - aPID->kd);
	}
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SetSampleTime(PID_Controller* aPID, int16_t NewSampleTime)
{
	if(aPID==NULL)
		return;
	if (NewSampleTime > 0)
	{
		int32_t ratio  = (int32_t)NewSampleTime / (int32_t)aPID->SampleTime;
		aPID->ki *= ratio;
		aPID->kd /= ratio;
		aPID->SampleTime = (unsigned long)NewSampleTime;
	}
}


/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(PID_Controller* aPID, int32_t Min, int32_t Max)
{
	if(aPID==NULL)
		return;
	if(Min >= Max) 
		return;
	aPID->outMin = Min;
	aPID->outMax = Max;

	if(aPID->inAuto)
	{
		if(*aPID->myOutput > aPID->outMax)
			*aPID->myOutput = aPID->outMax;
		else if(*aPID->myOutput < aPID->outMin)
			*aPID->myOutput = aPID->outMin;

		if(aPID->outputSum > aPID->outMax)
			aPID->outputSum = aPID->outMax;
		else if(aPID->outputSum < aPID->outMin)
			aPID->outputSum = aPID->outMin;
	}
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_SetMode(PID_Controller* aPID, int16_t Mode)
{
	if(aPID==NULL)
		return;
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !aPID->inAuto)
    {  /*we just went from manual to auto*/
        PID_Initialize(aPID);
    }
    aPID->inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID_Initialize(PID_Controller* aPID)
{
	if(aPID==NULL)
		return;
	aPID->outputSum = *aPID->myOutput;
	aPID->lastInput = *aPID->myInput;
	if(aPID->outputSum > aPID->outMax)
		aPID->outputSum = aPID->outMax;
	else if(aPID->outputSum < aPID->outMin)
		aPID->outputSum = aPID->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(PID_Controller* aPID, int16_t Direction)
{
	if(aPID==NULL)
		return;
	if(aPID->inAuto && Direction != aPID->controllerDirection)
	{
		aPID->kp = (0 - aPID->kp);
		aPID->ki = (0 - aPID->ki);
		aPID->kd = (0 - aPID->kd);
	}
	aPID->controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/

int32_t PID_GetKp(PID_Controller* aPID)
{ 
	if(aPID==NULL)
		return 0;
	return  aPID->dispKp; 
}

int32_t PID_GetKi(PID_Controller* aPID)
{ 
	if(aPID==NULL)
		return 0;
	return  aPID->dispKi;
}

int32_t PID_GetKd(PID_Controller* aPID)
{ 
	if(aPID==NULL)
		return 0;
	return  aPID->dispKd;
}

int16_t PID_GetMode(PID_Controller* aPID)
{ 
	if(aPID==NULL)
		return 0;
	return  aPID->inAuto ? AUTOMATIC : MANUAL;
}

int16_t PID_GetDirection(PID_Controller* aPID)
{ 
	if(aPID==NULL)
		return 0;
	return aPID->controllerDirection;
}
