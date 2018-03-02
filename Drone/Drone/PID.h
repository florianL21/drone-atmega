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

//SampleTime in ms


#define NUM_PID_CONTROLLERS 5
#define TICKS_PER_SECOND	2625000

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
	uint32_t LastTime;
};
typedef struct pidData pidData;


StatusCode PID_Initialize(pidData* pidController, float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, float Min, float Max, float SampleTime);
StatusCode PID_Compute(pidData* pidController);
StatusCode PID_SetTunings(pidData* pidController, float Kp, float Ki, float Kd);
StatusCode PID_SetSampleTime(pidData* pidController, float NewSampleTime);
StatusCode PID_SetOutputLimits(pidData* pidController, float Min, float Max);
StatusCode PID_SetControllerDirection(pidData* pidController, uint8_t Direction);
void PID_Init();
bool PID_need_compute(pidData* pidController);
void PID_Reset(pidData* pidController);

#endif /* PID_H_ */