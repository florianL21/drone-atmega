/*
 * Drone.c
 *
 * Created: 15.10.2017 14:14:19
 * Author : flola
 */ 

#include "sam.h"
#include "uart0.h"
#include "UARTCOM.h"





/*
void PIOB_Handler(void)
{
	// Save all triggered interrupts
	uint32_t status = PIOB->PIO_ISR;
	
	if (status & PIO_PB26)
	{
		uart0_putc('I');
		
		LEDTOGGLE;
	}
}





void configure_int(void)
{
	// Enable Clock for PIOB - needed for sampling falling edge
	PMC->PMC_PCER0 = 1 << ID_PIOB;
	
	// Enable IO pin control
	PIOB->PIO_PER = PIO_PB26;
	
	// Disable output (set to High Z)
	PIOB->PIO_ODR = PIO_PB26;
	
	// Enable pull-up
	PIOB->PIO_PUER = PIO_PB26;
	
	
	// Enable Glitch/Debouncing filter
	PIOB->PIO_IFER = PIO_PB26;
	
	// Select Debouncing filter
	PIOB->PIO_DIFSR = PIO_PB26;
	
	// Set Debouncing clock divider
	PIOB->PIO_SCDR = 0x4FF;
	
	
	// Select additional detection mode (for single edge detection)
	PIOB->PIO_AIMER = PIO_PB26;
	
	// The interrupt source is an Edge detection event.
	PIOB->PIO_ESR = PIO_PB26;
	
	// The interrupt source is set to a Falling Edge detection
	PIOB->PIO_FELLSR = PIO_PB26;
	
	
	// Enables the Input Change Interrupt on the I/O line.
	PIOB->PIO_IER = PIO_PB26;
	
	// Enable Interrupt Handling in NVIC
	NVIC_EnableIRQ(PIOB_IRQn);
}

*/
void configure_wdt(void)
{
	WDT->WDT_MR = 0x00000000; // disable WDT
}



/*
void configure_tc(void)
{

	// Enable TC0 (27 is TC0)
	PMC->PMC_PCER0 = 1 << ID_TC0;
	
	// Disable TC clock
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
	
	// Disable interrupts
	TC0->TC_CHANNEL[0].TC_IDR = 0xFFFFFFFF;
	
	// Clear status register
	TC0->TC_CHANNEL[0].TC_SR;
	
	// Set TC0 Mode: Compare C and Clock5 (slow clock)
	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK5;
	
	// Set Compare Value in RC register
	TC0->TC_CHANNEL[0].TC_RC = 64000; // note: RC oscillator is around 32kHz
	
	// Enable interrupt on RC compare
	TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;

	// Enable interrupt in NVIC
	NVIC_EnableIRQ(TC0_IRQn);
	
	// Reset counter (SWTRG) and start counter clock (CLKEN)
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
	
}
void TC0_Handler(void)
{
	// read status from TC0 status register
	TC0->TC_CHANNEL[0].TC_SR;
	
	//LEDTOGGLE;
}*/

void config1()
{
	// PWM set-up on pins DAC1, A8, A9, A10, D9, D8, D7 and D6 for channels 0 through to 7 respectively
	REG_PMC_PCER1 |= PMC_PCER1_PID36;                                               // Enable PWM
	REG_PIOB_ABSR |= PIO_ABSR_P19 | PIO_ABSR_P18 | PIO_ABSR_P17 | PIO_ABSR_P16;     // Set the port B PWM pins to peripheral type B
	REG_PIOC_ABSR |= PIO_ABSR_P24 | PIO_ABSR_P23 | PIO_ABSR_P22 | PIO_ABSR_P21;     // Set the port C PWM pins to peripheral type B
	REG_PIOB_PDR |= PIO_PDR_P19 | PIO_PDR_P18 | PIO_PDR_P17 | PIO_PDR_P16;          // Set the port B PWM pins to outputs
	REG_PIOC_PDR |= PIO_PDR_P24 | PIO_PDR_P23 | PIO_PDR_P22 | PIO_PDR_P21;          // Set the port C PWM pins to outputs
	REG_PWM_CLK = PWM_CLK_PREA(5) | PWM_CLK_DIVA(1);                                // Set the PWM clock A rate to 84MHz (84MHz/1)
	//REG_PWM_SCM |= PWM_SCM_SYNC7 | PWM_SCM_SYNC6 | PWM_SCM_SYNC5 | PWM_SCM_SYNC4 |  // Set the PWM channels as synchronous
	//               PWM_SCM_SYNC3 | PWM_SCM_SYNC2 | PWM_SCM_SYNC1 | PWM_SCM_SYNC0;
	for (uint8_t i = 0; i < PWMCH_NUM_NUMBER; i++)                      // Loop for each PWM channel (8 in total)
	{
		PWM->PWM_CH_NUM[i].PWM_CMR =  PWM_CMR_CPRE_CLKA;                  // Enable single slope PWM and set the clock source as CLKA
		PWM->PWM_CH_NUM[i].PWM_CPRD = 52500;                               // Set the PWM period register 84MHz/(40kHz)=2100;
	}
	//REG_PWM_ENA = PWM_ENA_CHID0;           // Enable the PWM channels, (only need to set channel 0 for synchronous mode)
	REG_PWM_ENA = PWM_ENA_CHID7 | PWM_ENA_CHID6 | PWM_ENA_CHID5 | PWM_ENA_CHID4 |    // Enable all PWM channels
	PWM_ENA_CHID3 | PWM_ENA_CHID2 | PWM_ENA_CHID1 | PWM_ENA_CHID0;
	for (uint8_t i = 0; i < PWMCH_NUM_NUMBER; i++)                      // Loop for each PWM channel (8 in total)
	{
		PWM->PWM_CH_NUM[i].PWM_CDTYUPD = 2625;                            // Set the PWM duty cycle to 50% (2100/2=1050) on all channels
	}
	//REG_PWM_SCUC = PWM_SCUC_UPDULOCK;      // Set the update unlock bit to trigger an update at the end of the next PWM period

}

void configure_PWM()
{
	// PWM set-up on pins DAC1, A8, A9, A10, D9, D8, D7 and D6 for channels 0 through to 7 respectively
	REG_PMC_PCER1 |= PMC_PCER1_PID36;                                               // Enable PWM
	REG_PIOB_ABSR |= PIO_ABSR_P19 | PIO_ABSR_P18 | PIO_ABSR_P17 | PIO_ABSR_P16;     // Set the port B PWM pins to peripheral type B
	REG_PIOB_PDR |= PIO_PDR_P19 | PIO_PDR_P18 | PIO_PDR_P17 | PIO_PDR_P16;          // Set the port B PWM pins to outputs
	REG_PWM_CLK = PWM_CLK_PREA(5) | PWM_CLK_DIVA(1);                                // Set the PWM clock A rate to 84MHz/32 ((84MHz/32)/1)

	for (uint8_t i = 0; i < 4; i++)                      // Loop for each PWM channel (8 in total)
	{
		PWM->PWM_CH_NUM[i].PWM_CMR =  PWM_CMR_CPRE_CLKA;                  // Enable single slope PWM and set the clock source as CLKA
		PWM->PWM_CH_NUM[i].PWM_CPRD = 52500;                               // Set the PWM period register 84MHz/(40kHz)=2100;
	}

	REG_PWM_ENA = PWM_ENA_CHID3 | PWM_ENA_CHID2 | PWM_ENA_CHID1 | PWM_ENA_CHID0;    // Enable all PWM channels
	
	for (uint8_t i = 0; i < 4; i++)                      // Loop for each PWM channel (8 in total)
	{
		PWM->PWM_CH_NUM[i].PWM_CDTYUPD = 2625;                            // Set the PWM duty cycle to 50% (2100/2=1050) on all channels
	}

	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = 2625;				// Set the PWM duty cycle to 1ms  -> (1/20)*PWM Frequency
	PWM->PWM_CH_NUM[1].PWM_CDTYUPD = 3281;
	PWM->PWM_CH_NUM[2].PWM_CDTYUPD = 3937;
	PWM->PWM_CH_NUM[3].PWM_CDTYUPD = 4593;
}

int main(void)
{
	/*configure_led_io();
	*/
	/* Initialize the SAM system */
	SystemInit();
	configure_wdt();
	configure_PWM();
	//config1();
	
	// Enable IO
	PIOC->PIO_PER = PIO_PC2;
	PIOC->PIO_PER = PIO_PC1;
	PIOC->PIO_PER = PIO_PC3 | PIO_PC4;
	// Set to output
	PIOC->PIO_OER = PIO_PC2;
	PIOC->PIO_OER = PIO_PC1;
	PIOC->PIO_OER = PIO_PC3 | PIO_PC4;
	// Disable pull-up
	PIOC->PIO_PUDR = PIO_PC2;
	PIOC->PIO_PUDR = PIO_PC1;
	PIOC->PIO_PUDR = PIO_PC3 | PIO_PC4;
	
	PIOB->PIO_CODR = PIO_PB27;
	while (1)
	{
		
		
	}
}