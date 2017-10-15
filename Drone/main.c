/*
 * Drone.c
 *
 * Created: 15.10.2017 14:14:19
 * Author : flola
 */ 

#include "sam.h"

#include "uart0.h"

#define LEDON (PIOB->PIO_SODR = PIO_PB27)
#define LEDOFF (PIOB->PIO_CODR = PIO_PB27)
#define LEDTOGGLE ((PIOB->PIO_ODSR & PIO_PB27) ? LEDOFF : LEDON)



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
	
	LEDTOGGLE;
}

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


void configure_wdt(void)
{
	WDT->WDT_MR = 0x00000000; // disable WDT
}

void _Delay(uint32_t num_time)
{
	for (int i = 0; i < num_time; i++)
		asm("nop");
}

int main(void)
{
	/* Initialize the SAM system */
	SystemInit();
	//SystemCoreClockUpdate();
	/*configure_led_io();
	configure_tc();
	*/
	configure_wdt();
	uart0_init(115200);
	
	// Enable IO
	PIOB->PIO_PER = PIO_PB27;
	// Set to output
	PIOB->PIO_OER = PIO_PB27;
	// Disable pull-up
	PIOB->PIO_PUDR = PIO_PB27;
	//configure_int();
	
	while (1)
	{
		LEDON;
		//uart0_putc('S');
		//_Delay(1000);
		//LEDOFF;
	}
}