#include "PID.h"


void PID_Initialize(PID_Controller* aPID);


unsigned long millis()
{
  return 0; //Dummy Function for compilation
}


/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID_Controller* PID_newPID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    PID_Controller *aPID = malloc(sizeof(*aPID));
    aPID->myOutput = Output;
    aPID->myInput = Input;
    aPID->mySetpoint = Setpoint;
    aPID->inAuto = false;

    PID_SetOutputLimits(aPID, 0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    aPID->SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID_SetControllerDirection(aPID, ControllerDirection);
    PID_SetTunings(aPID, Kp, Ki, Kd, POn);

    aPID->lastTime = millis() - aPID->SampleTime; //TODO: change to use a timer value
    return aPID;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID_Compute(PID_Controller* aPID)
{
   if(!aPID->inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - aPID->lastTime);
   if(timeChange >= aPID->SampleTime)
   {
      /*Compute all the working error variables*/
      double input = *aPID->myInput;
      double error = *aPID->mySetpoint - input;
      double dInput = (input - aPID->lastInput);
      aPID->outputSum += (aPID->ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!aPID->pOnE) 
        aPID->outputSum -= aPID->kp * dInput;

      if(aPID->outputSum > aPID->outMax) 
        aPID->outputSum = aPID->outMax;
      else if(aPID->outputSum < aPID->outMin) 
        aPID->outputSum = aPID->outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	   double output;
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
void PID_SetTunings(PID_Controller* aPID, double Kp, double Ki, double Kd, int POn)
{
   if (Kp < 0 || Ki < 0 || Kd < 0) 
     return;

   aPID->pOn = POn;
   aPID->pOnE = POn == P_ON_E;

   aPID->dispKp = Kp; 
   aPID->dispKi = Ki; 
   aPID->dispKd = Kd;

   double SampleTimeInSec = ((double)aPID->SampleTime)/1000;
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
void PID_SetSampleTime(PID_Controller* aPID, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime / (double)aPID->SampleTime;
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
void PID_SetOutputLimits(PID_Controller* aPID, double Min, double Max)
{
   if(Min >= Max) return;
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
void PID_SetMode(PID_Controller* aPID, int Mode)
{
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
void PID_SetControllerDirection(PID_Controller* aPID, int Direction)
{
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

double PID_GetKp(PID_Controller* aPID)
{ 
  return  aPID->dispKp; 
}

double PID_GetKi(PID_Controller* aPID)
{ 
  return  aPID->dispKi;
}

double PID_GetKd(PID_Controller* aPID)
{ 
  return  aPID->dispKd;
}

int PID_GetMode(PID_Controller* aPID)
{ 
  return  aPID->inAuto ? AUTOMATIC : MANUAL;
}

int PID_GetDirection(PID_Controller* aPID)
{ 
  return aPID->controllerDirection;
}
