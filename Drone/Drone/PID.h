/*
 * PID.h
 *
 * Created: 23.08.2016 12:44:32
 *  Author: Michael
 */ 


#ifndef PID_H_
#define PID_H_

#include "sam.h"
#include <stdbool.h>
#include <stdlib.h>
#include "config.h"
#include "ErrorHandling.h"
#include "GPT.h"

#define DIRECT 0
#define REVERSE 1


struct pidData
{
	float *Input, *Output, *Setpoint;
	float ITerm, lastInput;
	float kp, ki, kd;
	float SampleTime;
	float outMin, outMax;
	uint8_t controllerDirection;
	float LastTime;
};
typedef struct pidData pidData;


ErrorCode PID_Initialize(pidData* pidController, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, float Min, float Max, float SampleTime);
ErrorCode PID_Compute(pidData* pidController);
ErrorCode PID_SetTunings(pidData* pidController, float Kp, float Ki, float Kd);
ErrorCode PID_SetSampleTime(pidData* pidController, float NewSampleTime);
ErrorCode PID_SetOutputLimits(pidData* pidController, float Min, float Max);
ErrorCode PID_SetControllerDirection(pidData* pidController, uint8_t Direction);
void PID_Init();
bool PID_need_compute(pidData* pidController);
void PID_Reset(pidData* pidController);

#endif /* PID_H_ */