/*
 * uart0.c
 *
 * Created: 15.10.2017 15:50:14
 *  Author: flola
 */ 
#include "uart0.h"

void uart0_init(uint32_t baudRate)
{
	// Enable Clock for UART
	PMC->PMC_PCER0 = 1 << ID_UART;
	
	// Set pin in peripheral mode
	PIOA->PIO_PDR = PIO_PA8;
	PIOA->PIO_PDR = PIO_PA9;
	
	// Select peripheral A
	PIOA->PIO_ABSR &= !PIO_PA8;
	PIOA->PIO_ABSR &= !PIO_PA9;
	
	// Enable pull-ups
	PIOA->PIO_PUER = PIO_PA8;
	PIOA->PIO_PUER = PIO_PA9;
	
	// UART clock divider
	UART->UART_BRGR = 84000000 / baudRate / 16;
	
	// UART mode 8n1
	UART->UART_MR = UART_MR_PAR_NO;
	
	// Enable Receive and Transmit
	UART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	
	//Normal Mode
	UART->UART_MR |= UART_MR_CHMODE_NORMAL;
	
	// Enable UART interrupt
	UART->UART_IER = UART_IER_RXRDY | UART_IER_ENDTX;
	//UART->UART_IDR = ~(UART_IDR_ENDRX | UART_IER_ENDTX);
	
	// Enable UART Interrupt Handling in NVIC
	NVIC_EnableIRQ(UART_IRQn);
	
	// Enable Peripheral DMA Controller Transmission
	//UART->UART_PTCR = UART_PTCR_TXTEN;
}

void UART_Handler(void) //recive interrupt triggerd
{
	if (UART->UART_SR & UART_SR_ENDRX)
	{
		//uart0_putc('R');
	}
	else if(UART->UART_SR & UART_SR_ENDTX)
	{
		//uart0_putc('T');
	}
	else
	{
		//uart0_putc('x');
	}
	/*if (UART->UART_SR & UART_SR_RXRDY)
	{
		uint8_t in = UART->UART_RHR; // Read in received byte
		UART->UART_CR = UART_CR_RSTSTA; // Clear receiver errors (if any)
		
		if (in == 'a')
		{
			uart0_putc('a');
			// Disable TC clock
			TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
			
			// Set compare value to about 1/3rd of a sec
			TC0->TC_CHANNEL[0].TC_RC = 32000 / 3;

			// Reset counter (SWTRG) and enable counter clock (CLKEN)
			TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
		}
		else if (in == 'b')
		{
			uart0_putc('b');
			// Disable TC clock
			TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
			
			// Set compare value to about a sec
			TC0->TC_CHANNEL[0].TC_RC = 32000;

			// Reset counter (SWTRG) and enable counter clock (CLKEN)
			TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
		}
	}*/
}

void uart0_putc(char Character)
{
	//while (UART->UART_TCR); // wait until previous transmission is finished
	//UART->UART_TPR = (uint32_t)&Character; // set Trasmission pointer in PDC register
	//UART->UART_TCR = 1; // set count of characters to be sent; starts transmission (since UART_PTCR_TXTEN is already set)
	UART->UART_THR |= Character;
}