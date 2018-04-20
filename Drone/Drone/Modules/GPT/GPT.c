/*
 * GPT.c
 *
 * Created: 03.04.2018 17:33:16
 *  Author: flola
 */ 
#include "GPT.h"

typedef struct {
	uint16_t timeInMs;
	uint16_t timeCount;
	GPT_TIMER_CALLBACK callback;
	bool IsEnabled;
	bool IsEmpty;
	bool IsDelay;
}GPT_Timer;

GPT_Timer allTimers[MAX_NUM_GPT];

uint8_t numTimers = 0;

uint32_t runtime_in_ms = 0;

void GPT_Init()
{
	// Enable TC0 --> channel 1 ==> TC1 (28 is TC1)
	PMC->PMC_PCER0 = 1 << ID_TC1;
	
	// Disable TC clock
	TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKDIS;
	
	// Disable interrupts
	TC0->TC_CHANNEL[1].TC_IDR = 0xFFFFFFFF;

	// Clear status register
	TC0->TC_CHANNEL[1].TC_SR;
	
	// Set TC0 Mode: Compare C and Clock3 (MCLK/32) = 2.625.000
	TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK3;
	
	// Set Compare Value in RC register
	TC0->TC_CHANNEL[1].TC_RC = 2625; // exactly 1ms
	
	// Enable interrupt on RC compare
	TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;

	// Enable interrupt in NVIC
	NVIC_SetPriority(TC1_IRQn, ISR_PRIORITY_GPT);
	NVIC_EnableIRQ(TC1_IRQn);
	
	// Reset counter (SWTRG) and start counter clock (CLKEN)
	TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
	
	GPT_Timer NullTimer;
	NullTimer.callback = NULL;
	NullTimer.timeInMs = 0;
	NullTimer.IsEnabled = false;
	NullTimer.IsEmpty = true;
	NullTimer.timeCount = 0;
	for (Timer i = 0; i < MAX_NUM_GPT; i++)
	{
		allTimers[i] = NullTimer;
	}
}

Timer GPT_TimerSetup(uint16_t timeInMs, GPT_TIMER_CALLBACK callback, bool enabled)
{
	bool hasSpace = false;
	uint8_t emptyTimerIndex = 0;
	for (Timer i = 0; i < MAX_NUM_GPT; i++)
	{
		if(allTimers[i].IsEmpty == true)
		{
			hasSpace = true;
			emptyTimerIndex = i;
			break;
		}
	}
	if(hasSpace == true)
	{
		allTimers[emptyTimerIndex].IsEmpty = false;
		allTimers[emptyTimerIndex].callback = callback;
		allTimers[emptyTimerIndex].timeInMs = timeInMs;
		allTimers[emptyTimerIndex].IsEnabled = enabled;
		numTimers++;
		return emptyTimerIndex + 1;
	}
	return 0;
}

void GPT_TimerSetTime(Timer TimerNum, uint16_t timeInMs)
{
	if(TimerNum != 0)
	{
		allTimers[TimerNum - 1].timeInMs = timeInMs;
	}
}

void GPT_TimerSetEnabled(Timer TimerNum, bool enabled)
{
	if(TimerNum != 0)
	{
		allTimers[TimerNum - 1].IsEnabled = enabled;
	}
}

void GPT_TimerDelete(Timer TimerNum)
{
	if(TimerNum != 0)
	{
		allTimers[TimerNum - 1].IsEmpty = true;
		numTimers--;
	}
}

uint32_t GPT_GetTime()
{
	return runtime_in_ms;
}

float GPT_GetPreciseTime()
{
	return runtime_in_ms + (((float)TC0->TC_CHANNEL[1].TC_CV) * (1.0/2625.0));
}

void GPT_Delay(float Delay_in_ms)
{
	float startTime = GPT_GetPreciseTime();
	while(GPT_GetPreciseTime() - startTime < Delay_in_ms);
}

Timer GPT_DelayedCall(GPT_TIMER_CALLBACK callback, uint16_t Delay_in_ms)
{
	Timer timerNum = 0;
	timerNum = GPT_TimerSetup(Delay_in_ms, callback, true);
	if(timerNum != 0)
		allTimers[timerNum - 1].IsDelay = true;
	return timerNum;
}

void GPT_CancelDelayedCall(Timer TimerNum)
{
	GPT_TimerDelete(TimerNum);
}

void TC1_Handler()
{
	// read status from TC0 status register
	TC0->TC_CHANNEL[1].TC_SR;
	runtime_in_ms++;
	
	uint8_t numCheckedTimers = 0;
	for (Timer i = 0; i < MAX_NUM_GPT; i++)
	{
		if(allTimers[i].IsEmpty == false)
		{
			if(allTimers[i].callback != NULL && allTimers[i].IsEnabled == true)
			{
				if(++allTimers[i].timeCount >= allTimers[i].timeInMs)
				{
					allTimers[i].timeCount = 0;
					allTimers[i].callback();
					if(allTimers[i].IsDelay == true)
						GPT_TimerDelete(i);
				}
			}
			
			if(++numCheckedTimers >= numTimers)
				return;
		}
	}
}