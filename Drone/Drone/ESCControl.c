/*
 * ESCControl.c
 *
 * Created: 29.09.2017 20:07:33
 *  Author: Markus Lorenz
 */ 


#include "ESCControl.h"


uint16_t ESC_Offset[4] = {ESC_Offset_M1,ESC_Offset_M2,ESC_Offset_M3,ESC_Offset_M4};
uint16_t ESC_SlopeComp[4] = {ESC_SlopeComp_M1,ESC_SlopeComp_M2,ESC_SlopeComp_M3,ESC_SlopeComp_M4};

void esc_init()
{
	// PWM set-up on pins DAC1, A8, A9, A10, D9, D8, D7 and D6 for channels 0 through to 7 respectively
	REG_PMC_PCER1 |= PMC_PCER1_PID36;                                               // Enable PWM
	REG_PIOC_ABSR |= PIO_ABSR_P24 | PIO_ABSR_P23 | PIO_ABSR_P22 | PIO_ABSR_P21;     // Set the port C PWM pins to peripheral type B
	REG_PIOC_PDR |= PIO_PDR_P24 | PIO_PDR_P23 | PIO_PDR_P22 | PIO_PDR_P21;          // Set the port C PWM pins to outputs
	REG_PWM_CLK = PWM_CLK_PREA(5) | PWM_CLK_DIVA(1);                                // Set the PWM clock A rate to 84MHz (84MHz/1)
	for (uint8_t i = 0; i < PWMCH_NUM_NUMBER; i++)                      // Loop for each PWM channel (8 in total)
	{
		PWM->PWM_CH_NUM[i].PWM_CMR =  PWM_CMR_CPRE_CLKA;							// Enable single slope PWM and set the clock source as CLKA
		PWM->PWM_CH_NUM[i].PWM_CPRD = ESC_PWM_PERIOD;                               // Set the PWM period register 84MHz/(50Hz)=2100;
	}
	REG_PWM_ENA = PWM_ENA_CHID7 | PWM_ENA_CHID6 | PWM_ENA_CHID5 | PWM_ENA_CHID4;
	for (uint8_t i = 0; i < PWMCH_NUM_NUMBER; i++)                      // Loop for each PWM channel (8 in total) PWMCH_NUM_NUMBER
	{
		PWM->PWM_CH_NUM[i].PWM_CDTYUPD = ESC_PWM_MIN_DUTY_CYCLE;                            // Set the PWM duty cycle to 50% (2100/2=1050) on all channels
	}
}

//Motor1: Timer0 PIN PB16
//Motor2: Timer1 PIN PB17
//Motor3: Timer2 PIN PB18
//Motor4: Timer3 PIN PB19
//value must be in between 0 and ESC_MaxLimit-ESC_PWM_MIN_DUTY_CYCLE
StatusCode esc_set(uint8_t MotorNum, int16_t speed) 
{
	MotorNum += 3;
	if(MotorNum < 4 || MotorNum > 7)
		return ESCControl_ERROR_ARGUMENT_OUT_OF_RANGE;
	
	if(speed > (ESC_MaxLimit-ESC_PWM_MIN_DUTY_CYCLE))
	{
		speed = (ESC_MaxLimit-ESC_PWM_MIN_DUTY_CYCLE);
	}
	if(speed <= 0)
	{
		speed = 0;
		PWM->PWM_CH_NUM[MotorNum].PWM_CDTYUPD = ESC_PWM_MIN_DUTY_CYCLE;
	}
	else
	{
		PWM->PWM_CH_NUM[MotorNum].PWM_CDTYUPD = ((ESC_PWM_MIN_DUTY_CYCLE + speed + ESC_Offset[MotorNum-4]) + (((float)speed) * ESC_SlopeComp[MotorNum-4]));
	}
	return SUCCESS;
}