/*
 * Drone.c
 *
 * Created: 15.10.2017 14:14:19
 * Author : flola
 */ 

#include "sam.h"
#include "uart0.h"
#include "ESCControl.h"
#include "RCReader.h"






/*
void PIOB_Handler(void)
{
	// Save all triggered interrupts
	uint32_t status = PIOB->PIO_ISR;
	
	if (status & PIO_PB26)
	{
		uart0_puts("t");
	}
}*/
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
	//PIOB->PIO_IFER = PIO_PB26;
	// Select Debouncing filter
	//PIOB->PIO_DIFSR = PIO_PB26;
	// Set Debouncing clock divider
	//PIOB->PIO_SCDR = 0x4FF;
	// Select additional detection mode (for single edge detection)
	PIOB->PIO_AIMER = PIO_PB26;
	// The interrupt source is an Edge detection event.
	PIOB->PIO_ESR = PIO_PB26;
	// The interrupt source is set to a falling and rising Edge detection
	PIOB->PIO_FELLSR = PIO_PB26;
	PIOB->PIO_REHLSR = PIO_PB26;
	// Enables the Input Change Interrupt on the I/O line.
	PIOB->PIO_IER = PIO_PB26;
	// Enable Interrupt Handling in NVIC
	NVIC_EnableIRQ(PIOB_IRQn);
}



void configure_wdt(void)
{
	WDT->WDT_MR = 0x00000000; // disable WDT
}

int main(void)
{
	/*configure_led_io();
	*/
	/* Initialize the SAM system */
	SystemInit();
	configure_wdt();
	uart0_init(115200);
	//configure_int();
	rc_init();
	/*esc_init();
	esc_set(1,20);
	esc_set(2,500);
	esc_set(3,800);
	esc_set(4,1100);*/
	
	/*// Enable IO
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
	
	PIOB->PIO_CODR = PIO_PB27;*/
	
	uint8_t buffer[20];
	RemoteControlValues Values;
	while (1)
	{
		if(uart0_has_space())
		{
			Values = rc_read_values();
			itoa(Values.Role,buffer,10);
			strcat(buffer,"\n\r");
			uart0_puts(buffer);
		}
	}
}