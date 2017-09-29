/*
 * ESCControl.c
 *
 * Created: 29.09.2017 20:07:33
 *  Author: Markus Lorenz
 */ 


#include "ESCControl.h"

void esc_init()
{
	// Configure PIN 2 (PE4), 3 (PE5), 5 (PE3), and 6 (PH3) as output
	DDRE |= (1 << PE3) | (1 << PE4) | (1 << PE5);
	DDRH |= (1 << PH3);
	
	// Configure Timer 4 in PWM mode for PIN 6 (PH3) - OCR4A
	TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << WGM41);
	TCCR4B = (1 << WGM42) | (1 << WGM43);
	TCCR4B |= (1 << CS41);
	OCR4A = 2000; // PIN 6 (PH3)
	ICR4 = 39960;
	
	// Configure Timer 3 in PWM mode for PIN 2, 3, and 5
	TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM31);
	TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31);
	OCR3A = 2000; // PIN 5 (PE3)
	OCR3B = 2000; // PIN 2 (PE4)
	OCR3C = 2000; // PIN 3 (PE5)
	ICR3 = 39960;
}

void esc_set_m1(int16_t speed) // PIN 2
{
	if(speed > ESC_MaxLimit)
	{
		speed = ESC_MaxLimit;
	}
	else if(speed <= 0)
	{
		speed = 0;
		OCR3B = 2000;
	}
	else
	{
		OCR3B = round((2000 + speed + ESC_Offset_M1) + (((float)speed) * ESC_SlopeComp_M1));
	}
}

void esc_set_m2(int16_t speed) // PIN 3
{
	if(speed > ESC_MaxLimit)
	{
		speed = ESC_MaxLimit;
	}
	else if(speed <= 0)
	{
		speed = 0;
		OCR3C = 2000;
	}
	else
	{
		OCR3C = round((2000 + speed + ESC_Offset_M2) + (((float)speed) * ESC_SlopeComp_M2));
	}
}

void esc_set_m3(int16_t speed) // PIN 5
{
	if(speed > ESC_MaxLimit)
	{
		speed = ESC_MaxLimit;
	}
	else if(speed <= 0)
	{
		speed = 0;
		OCR3A = 2000;
	}
	else
	{
		OCR3A = round((2000 + speed + ESC_Offset_M3) + (((float)speed) * ESC_SlopeComp_M3));
	}
}

void esc_set_m4(int16_t speed) // PIN 6
{
	if(speed > ESC_MaxLimit)
	{
		speed = ESC_MaxLimit;
	}
	else if(speed <= 0)
	{
		speed = 0;
		OCR4A = 2000;
	}
	else
	{
		OCR4A = round((2000 + speed + ESC_Offset_M4) + (((float)speed) * ESC_SlopeComp_M4));
	}
}
