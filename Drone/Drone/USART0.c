/*
 * USART0.c
 *
 * Created: 18.11.2017 19:14:58
 *  Author: flola
 */ 
#include "USART0.h"

uint8_t TXBUFFER[270]; // UART PDC Transmit buffer

void USART0_init(uint32_t BaudRate)
{
	// Enable Clock for UART
	PMC->PMC_PCER0 = 1 << ID_USART0;
	// Set pin in peripheral mode
	PIOA->PIO_PDR = PIO_PA10;
	PIOA->PIO_PDR = PIO_PA11;
	// Select peripheral A
	PIOA->PIO_ABSR &= !PIO_PA10;
	PIOA->PIO_ABSR &= !PIO_PA11;
	// Enable pull-ups
	PIOA->PIO_PUER = PIO_PA10;
	PIOA->PIO_PUER = PIO_PA11;
	PIOA->PIO_PDR = PIO_PA11|PIO_PA10;
	// UART clock divider (around 115200)b
	USART0->US_BRGR = 84000000 / BaudRate / 16;
	// UART mode 8n1
	USART0->US_MR = US_MR_USART_MODE_NORMAL|US_MR_USCLKS_MCK|US_MR_PAR_NO|US_MR_CHRL_8_BIT|US_MR_PAR_NO|US_MR_NBSTOP_1_BIT|US_MR_CHMODE_NORMAL;
	// Enable Receive and Transmit
	USART0->US_CR = US_CR_TXEN; //| US_CR_RXEN;
	// Enable UART interrupt
	//USART0->US_IER = US_IER_RXRDY;//| UART_IER_TXRDY;
	//UART->UART_IDR = UART_IDR_ENDRX|UART_IDR_ENDTX|UART_IDR_TXBUFE|UART_IDR_TXEMPTY|UART_IDR_TXRDY;
	// Enable UART Interrupt Handling in NVIC
	//NVIC_EnableIRQ(USART0_IRQn);
	// Enable Peripheral DMA Controller Transmission
	//USART0->US_PTCR=US_PTCR_TXTEN;
}

void USART0_put_raw_data(uint8_t* sendData, uint16_t Length)
{
	if(sendData!=NULL)
	{
		// copy to buffer - MUST be in SRAM as PDC is not connected to Flash
		for (uint16_t count = 0; count < Length; count++)
		{
			TXBUFFER[count] = sendData[count];
		}
		USART0->US_THR ='t';
		//USART0->US_TPR = (uint32_t)TXBUFFER; 	// set Trasmission pointer in PDC register
		//USART0->US_TCR = Length; 					// set count of characters to be sent; starts transmission (since UART_PTCR_TXTEN is already set)
		//USART0->US_IER = US_IER_TXRDY;		//activate Transmit done interrupt
	} else
	{
		
	}
}