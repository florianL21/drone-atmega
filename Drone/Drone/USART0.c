/*
 * USART0.c
 *
 * Created: 18.11.2017 19:14:58
 *  Author: flola
 */ 
#include "USART0.h"

uint8_t TXBUFFER[270]; // UART PDC Transmit buffer
USART_RECV_CALLBACK usart_reciveCallBack = NULL;
uint32_t ReceiveLength = 1;
uint8_t *ReceivePtr = NULL;
uint32_t LastReceiveLength = 0;
uint8_t *LastReceivePtr = NULL;
bool IsFirstReceive = true;
Queue* usart0SendQueue;
bool transmitInProgress = false;

void USART0_init(uint32_t BaudRate, uint32_t RecvLength)
{
	USART0_set_receiver_length(RecvLength);
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
	USART0->US_CR = US_CR_TXEN | US_CR_RXEN;
	// Disable all UART interrupts
	USART0->US_IDR = 0xFFFFFFFF;
	// Enable UART interrupt
	USART0->US_IER = US_IER_ENDRX;//| UART_IER_TXRDY;
	// Enable UART Interrupt Handling in NVIC
	NVIC_EnableIRQ(USART0_IRQn);
	// Enable Peripheral DMA Controller Transmission
	USART0->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN;
	
	usart0SendQueue = queue_new(USART0_QUEUE_MAX_ITEMS);

	
}

bool USART0_has_space()
{
	return queue_has_space(usart0SendQueue);
}

void _put_raw_data_(uint8_t* sendData, uint16_t Length)
{
	// copy to buffer - MUST be in SRAM as PDC is not connected to Flash
	for (uint16_t count = 0; count < Length; count++)
	{
		TXBUFFER[count] = sendData[count];
	}
	USART0->US_TPR = (uint32_t)TXBUFFER; 	// set Trasmission pointer in PDC register
	USART0->US_TCR = Length; 				// set count of characters to be sent; starts transmission (since UART_PTCR_TXTEN is already set)
	USART0->US_IER = US_IER_TXRDY;			//activate Transmit done interrupt
}

bool USART0_put_data(uint8_t* sendData, uint16_t Length)
{
	if(sendData != NULL)
	{
		if(transmitInProgress == false)
		{
			transmitInProgress = true;
			_put_raw_data_(sendData,Length);
			return true;
		} else if(queue_has_space(usart0SendQueue))
		{
			queue_write(usart0SendQueue,sendData,Length);
			return true;
		}else{
			return false;
		}
	} else
	{
		return false;
	}
}

void USART0_set_receiver_length(uint32_t Length)
{
	
	if(ReceivePtr != NULL)
	{
		free(ReceivePtr);
	}
	ReceiveLength = Length;
	USART0->US_RCR = ReceiveLength;
	ReceivePtr = malloc(Length*sizeof(uint8_t));
	USART0->US_RPR = (uint32_t)ReceivePtr;
	/*ReceiveLength = Length;
	if(IsFirstReceive == true)
	{
		USART0->US_RCR = ReceiveLength - 1;
		IsFirstReceive = false;
	}else
	{
		USART0->US_RCR = ReceiveLength;
		if(ReceivePtr != NULL)
		{
			free(ReceivePtr);
		}
	}
	ReceivePtr = malloc(ReceiveLength*sizeof(uint8_t));
	USART0->US_RPR = (uint32_t)ReceivePtr;*/
}

void USART0_register_received_callback(USART_RECV_CALLBACK callBack)
{
	usart_reciveCallBack = callBack;
}

void USART0_Handler(void)
{
	USART0->US_CR = US_CR_RSTSTA; // Clear receiver errors (if any)
	
	if (USART0->US_CSR & US_CSR_ENDRX)
	{
		
		if(usart_reciveCallBack != NULL)
		{
			usart_reciveCallBack(ReceivePtr,ReceiveLength);
		}
		
		USART0->US_RCR = ReceiveLength;
		USART0->US_RPR = (uint32_t)ReceivePtr;
	}
	if(USART0->US_CSR & US_CSR_TXRDY)
	{
		if(queue_is_empty(usart0SendQueue))
		{
			transmitInProgress = false;
			USART0->US_IDR = US_IDR_TXRDY;		//deactivate transmit done interrupt to keep it from firing constantly...
		}else{
			queue_node qData = queue_read(usart0SendQueue);
			_put_raw_data_(qData.data, qData.Length);
		}
	}
}