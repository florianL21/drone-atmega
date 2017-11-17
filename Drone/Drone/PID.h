/*
This is a direct port of the Arduino-PID library by Brett Beauregard: https://github.com/br3ttb/Arduino-PID-Library
It was modified to work in plain C
*/



#ifndef PID_H_
#define PID_H_
#include <stdbool.h>
#include <stdlib.h>
#include "sam.h"

//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	  0
#define DIRECT    0
#define REVERSE   1
#define P_ON_M    0
#define P_ON_E    1

#define TIMER_CONVERSION_FACTOR		0.380952381





struct PID_Controller {
	int32_t dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	int32_t dispKi;				//   format for display purposes
	int32_t dispKd;				//

	int32_t kp;                  // * (P)roportional Tuning Parameter
	int32_t ki;                  // * (I)ntegral Tuning Parameter
	int32_t kd;                  // * (D)erivative Tuning Parameter

	int8_t controllerDirection;
	int8_t pOn;

	int32_t *myInput;              // * Pointers to the Input, Output, and Setpoint16_t variables
	int32_t *myOutput;             //   This creates a hard link between the variables and the 
	int32_t *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                            //   what these values are.  with pointers we'll just know.

	uint32_t lastTime;
	int32_t outputSum, lastInput;

	uint32_t SampleTime;
	int32_t outMin, outMax;
	bool inAuto, pOnE;
};
typedef struct PID_Controller PID_Controller;




int16_t PID_GetDirection(PID_Controller* aPID);
int16_t PID_GetMode(PID_Controller* aPID);
int32_t PID_GetKd(PID_Controller* aPID);
int32_t PID_GetKi(PID_Controller* aPID);
int32_t PID_GetKp(PID_Controller* aPID);
void PID_SetControllerDirection(PID_Controller* aPID, int16_t Direction);
void PID_SetOutputLimits(PID_Controller* aPID, int32_t Min, int32_t Max);
void PID_SetSampleTime(PID_Controller* aPID, int16_t NewSampleTime);
void PID_SetTunings(PID_Controller* aPID, int32_t Kp, int32_t Ki, int32_t Kd, int16_t POn);
bool PID_Compute(PID_Controller* aPID);
PID_Controller* PID_newPID(int32_t* Input, int32_t* Output, int32_t* Setpoint, int32_t Kp, int32_t Ki, int32_t Kd, int8_t POn, int8_t ControllerDirection);
void PID_Init();
void PID_delete(PID_Controller* aPID);


/*
//commonly used functions **************************************************************************

PID_Controller* newPID(int32_t* Input, int32_t* Output, int32_t* Setpoint, int32_t Kp, int32_t Ki, int32_t Kd, int16_t POn, int16_t ControllerDirection);               // * constructor.  links the PID to the Input, Output, and Setpoint.  Initial tuning parameters are also set here

void SetMode(int16_t Mode);               // * sets PID to either Manual (0) or Auto (non-0)

bool Compute();                       // * performs the PID calculation.  it should be
                                      //   called every time loop() cycles. ON/OFF and
                                      //   calculation frequency can be set using SetMode
                                      //   SetSampleTime respectively

void SetOutputLimits(int32_t, int32_t); // * clamps the output to a specific range. 0-255 by default, but
                                      //   it's likely the user will want to change this depending on
                                      //   the application
	


//available but not commonly used functions ********************************************************
void SetTunings(int32_t, int32_t, int32_t);        // * While most users will set the tunings once in the 
                         	                      //   constructor, this function gives the user the option
                                                //   of changing tunings during runtime for Adaptive control

void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
                  //   means the output will increase when error is positive. REVERSE
                  //   means the opposite.  it's very unlikely that this will be needed
                  //   once it is set in the constructor.
void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                      //   the PID calculation is performed.  default is 100

//Display functions ****************************************************************
int32_t GetKp();						  // These functions query the pid for interal values.
int32_t GetKi();						  //  they were created mainly for the pid front-end,
int32_t GetKd();						  // where it's important to know what is actually 
int16_t GetMode();						  //  inside the PID.
int16_t GetDirection();
*/
#endif
