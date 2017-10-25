/*
 * Drone.c
 *
 * Created: 15.10.2017 14:14:19
 * Author : flola
 */ 

#include "sam.h"
//#include "uart0.h"
#include "UARTCOM.h"


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

*/
void configure_wdt(void)
{
	WDT->WDT_MR = 0x00000000; // disable WDT
}



int main(void)
{
	//SystemCoreClockUpdate();
	/*configure_led_io();
	configure_tc();
	*/
	/* Initialize the SAM system */
	SystemInit();
	// Enable IO
	PIOB->PIO_PER = PIO_PB27;
	PIOC->PIO_PER = PIO_PC2;
	PIOC->PIO_PER = PIO_PC1;
	PIOC->PIO_PER = PIO_PC3 | PIO_PC4;
	// Set to output
	PIOB->PIO_OER = PIO_PB27;
	PIOC->PIO_OER = PIO_PC2;
	PIOC->PIO_OER = PIO_PC1;
	PIOC->PIO_OER = PIO_PC3 | PIO_PC4;
	// Disable pull-up
	PIOB->PIO_PUDR = PIO_PB27;
	PIOC->PIO_PUDR = PIO_PC2;
	PIOC->PIO_PUDR = PIO_PC1;
	PIOC->PIO_PUDR = PIO_PC3 | PIO_PC4;
	
	PIOB->PIO_CODR = PIO_PB27;
	
	configure_wdt();
	//uart0_init(115200);
	//configure_int();
	UARTCOM_init(115200);
	
	UARTCOM_send_debug("START!");
	
	PIOB->PIO_SODR = PIO_PB27;
	//UARTCOM_send_debug("START1!");
	PIOB->PIO_CODR = PIO_PB27;
	uint8_t dataArray[] = {0,0};
	/*uint8_t testData=5;
	Queue* testQueue = queue_new();
	queue_node dataOut;
	queue_write(testQueue, &testData, 1,false);
	testData = 10;
	queue_write(testQueue, &testData, 1,false);
	dataOut=queue_read(testQueue);
	UARTCOM_send_debug_n(*dataOut.data);
	dataOut=queue_read(testQueue);
	UARTCOM_send_debug_n(*dataOut.data);
	UARTCOM_send_debug("END!");
	
	
	dataOut=queue_read(testQueue);
	UARTCOM_send_debug_n(*dataOut.data);
	dataOut=queue_read(testQueue);
	UARTCOM_send_debug_n(*dataOut.data);*/
	while (1)
	{
		if(UARTCOM_ready_to_send())
		{
			dataArray[0]++;
			if(!UARTCOM_transmit_block('5',ARRAYDEF(dataArray)))
			{
				UARTCOM_send_debug("STOP");
			}
			if(dataArray[0] == 255)
				dataArray[1]++;
			PIOC->PIO_CODR = PIO_PC1;
			PIOC->PIO_SODR = PIO_PC1;
		}
		
	}
}