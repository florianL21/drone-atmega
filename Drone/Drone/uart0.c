/*
 * UART0.c
 *
 * Created: 29.09.2017 20:07:25
 *  Author: flola
 */ 

#include "UART0.h"

volatile bool sendingActive = false;
Queue* uartSendQueue;

UART_RECV_CALLBACK reciveCallBack = NULL;
uint8_t TXBUFFER[270]; // UART PDC Transmit buffer
uint8_t TXCHAR;

#ifdef UART_USE_QUEUE_LIMIT
	uint16_t queueItemCount = 0;
#endif

void uart0_putData(uint8_t* sendData, uint16_t Length);

void uart0_init(uint32_t BaudRate)
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
	// UART clock divider (around 115200)
	UART->UART_BRGR = 84000000 / BaudRate / 16;
	// UART mode 8n1
	UART->UART_MR = UART_MR_PAR_NO;
	// Enable Receive and Transmit
	UART->UART_CR = UART_CR_TXEN | UART_CR_RXEN;
	// Enable UART interrupt
	UART->UART_IER = UART_IER_RXRDY ;//| UART_IER_TXRDY;
	// Enable UART Interrupt Handling in NVIC
	NVIC_EnableIRQ(UART_IRQn);
	// Enable Peripheral DMA Controller Transmission
	UART->UART_PTCR = UART_PTCR_TXTEN;

	//initialize the Send queue
	uartSendQueue = queue_new();
}

void uart0_puts(uint8_t Data[])
{
	#ifdef UART_USE_QUEUE_LIMIT
		queueItemCount++;
		if(!uart0_has_space())
			return;
	#endif
		
	if(sendingActive == false)
	{
		uart0_putData(Data,strlen((char*)Data));
		sendingActive = true;
	} else
	{
		queue_write(uartSendQueue, Data, strlen((char*)Data));
	}
}

void uart0_register_recived_callback(UART_RECV_CALLBACK callBack)
{
	reciveCallBack = callBack;
}

void UART_Handler(void)
{	
	if (UART->UART_SR & UART_SR_RXRDY)
	{
		uint8_t in = UART->UART_RHR; // Read in received byte		
		UART->UART_CR = UART_CR_RSTSTA; // Clear receiver errors (if any)
		if(reciveCallBack != NULL)
		{
			reciveCallBack(in);
		}
	}else if(UART->UART_SR & UART_SR_TXRDY)	
	{
		UART->UART_IDR = UART_IER_TXRDY;		//deactivate transmit done interrupt to keep it from firing constantly...
		if(queue_is_empty(uartSendQueue))
		{
			sendingActive = false;
		} else
		{
			queue_node writeData = queue_read(uartSendQueue);
			uart0_putData(writeData.data, writeData.Length);
		}
		#ifdef UART_USE_QUEUE_LIMIT
			queueItemCount--;
		#endif
	}else
	{
		//impossible to reach... but maybe an error should be thrown anyways, who knows...
	}
}
/*
void uart0_putc(uint8_t Character)
{
	uint8_t dataArray[2];
	dataArray[0]=Character;
	dataArray[1]='\0';
	uart0_puts(dataArray);
}
*/

void uart0_putData(uint8_t* sendData, uint16_t Length)
{
	// copy to buffer - MUST be in SRAM as PDC is not connected to Flash
	for (uint16_t count = 0; count < Length; count++)
	{
		TXBUFFER[count] = sendData[count];
	}
	UART->UART_TPR = (uint32_t)TXBUFFER; 	// set Trasmission pointer in PDC register
	UART->UART_TCR = Length; 					// set count of characters to be sent; starts transmission (since UART_PTCR_TXTEN is already set)
	UART->UART_IER = UART_IER_TXRDY;		//activate Transmit done interrupt
}

#ifdef UART_USE_QUEUE_LIMIT
	bool uart0_has_space()
	{
		return queueItemCount < UART_QUEUE_MAX_ITEMS;
	}
#endif
