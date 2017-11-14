/*
This is a direct port of the Arduino-PID library by Brett Beauregard: https://github.com/br3ttb/Arduino-PID-Library
It was modified to work in plain C
*/



#ifndef PID_H_
#define PID_H_
#include <stdbool.h>

//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	  0
#define DIRECT    0
#define REVERSE   1
#define P_ON_M    0
#define P_ON_E    1





struct PID_Controller {
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
  double dispKi;				//   format for display purposes
  double dispKd;				//

  double kp;                  // * (P)roportional Tuning Parameter
  double ki;                  // * (I)ntegral Tuning Parameter
  double kd;                  // * (D)erivative Tuning Parameter

  int controllerDirection;
  int pOn;

  double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput;             //   This creates a hard link between the variables and the 
  double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                            //   what these values are.  with pointers we'll just know.

  unsigned long lastTime;
  double outputSum, lastInput;

  unsigned long SampleTime;
  double outMin, outMax;
  bool inAuto, pOnE;
};
typedef struct PID_Controller PID_Controller;




int PID_GetDirection(PID_Controller* aPID);
int PID_GetMode(PID_Controller* aPID);
double PID_GetKd(PID_Controller* aPID);
double PID_GetKi(PID_Controller* aPID);
double PID_GetKp(PID_Controller* aPID);
void PID_SetControllerDirection(PID_Controller* aPID, int Direction);
void PID_Initialize(PID_Controller* aPID);
void PID_SetOutputLimits(PID_Controller* aPID, double Min, double Max);
void PID_SetSampleTime(PID_Controller* aPID, int NewSampleTime);
void PID_SetTunings(PID_Controller* aPID, double Kp, double Ki, double Kd, int POn);
bool PID_Compute(PID_Controller* aPID);
PID_Controller* PID_newPID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection);


/*
//commonly used functions **************************************************************************

PID_Controller* newPID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection);               // * constructor.  links the PID to the Input, Output, and Setpoint.  Initial tuning parameters are also set here

void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

bool Compute();                       // * performs the PID calculation.  it should be
                                      //   called every time loop() cycles. ON/OFF and
                                      //   calculation frequency can be set using SetMode
                                      //   SetSampleTime respectively

void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
                                      //   it's likely the user will want to change this depending on
                                      //   the application
	


//available but not commonly used functions ********************************************************
void SetTunings(double, double, double);        // * While most users will set the tunings once in the 
                         	                      //   constructor, this function gives the user the option
                                                //   of changing tunings during runtime for Adaptive control

void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
                  //   means the output will increase when error is positive. REVERSE
                  //   means the opposite.  it's very unlikely that this will be needed
                  //   once it is set in the constructor.
void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                      //   the PID calculation is performed.  default is 100

//Display functions ****************************************************************
double GetKp();						  // These functions query the pid for interal values.
double GetKi();						  //  they were created mainly for the pid front-end,
double GetKd();						  // where it's important to know what is actually 
int GetMode();						  //  inside the PID.
int GetDirection();
*/
#endif
