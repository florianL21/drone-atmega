/*
 * USART0.c
 *
 * Created: 18.11.2017 19:14:58
 *  Author: flola
 */ 
#include "USART0.h"

#define MIN_BAUD_RATE	9600
#define MAX_BAUD_RATE	1843200

uint8_t TXBUFFER[270]; // UART PDC Transmit buffer
USART_RECV_CALLBACK usart_reciveCallBack = NULL;
uint32_t usart0_ReceiveLength = 1;
uint8_t *usart0_ReceivePtr = NULL;
Queue* usart0SendQueue;
bool usart0_transmitInProgress = false;

void usart0_put_raw_data(uint8_t* sendData, uint16_t Length);

StatusCode USART0_init(uint32_t BaudRate, uint32_t RecvLength)
{
	if(BaudRate < MIN_BAUD_RATE || BaudRate > MAX_BAUD_RATE)
	{
		return USART0_ERROR_ARGUMENT_OUT_OF_RANGE;
	}
	DEFUALT_ERROR_HANDLER(USART0_set_receiver_length(RecvLength),error_return);
	
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
	USART0->US_IER = US_IER_ENDRX;
	// Enable UART Interrupt Handling in NVIC
	NVIC_EnableIRQ(USART0_IRQn);
	// Enable Peripheral DMA Controller Transmission
	USART0->US_PTCR = US_PTCR_TXTEN | US_PTCR_RXTEN;
	
	return queue_new(usart0SendQueue, USART0_QUEUE_MAX_ITEMS);
}

BoolStatusCode USART0_has_space()
{
	return queue_has_space(usart0SendQueue);
}

bool USART0_is_idle()
{
	return !usart0_transmitInProgress;
}

void usart0_put_raw_data(uint8_t* sendData, uint16_t Length)
{
	// copy to buffer - MUST be in SRAM as PDC is not connected to Flash
	for (uint16_t count = 0; count < Length; count++)
	{
		TXBUFFER[count] = sendData[count];
	}
	USART0->US_TPR = (uint32_t)TXBUFFER; 	// set Trasmission pointer in PDC register
	USART0->US_TCR = Length; 				// set count of characters to be sent; starts transmission (since UART_PTCR_TXTEN is already set)
	USART0->US_IER = US_IER_ENDTX;			//activate Transmit done interrupt
}

StatusCode USART0_put_data(uint8_t* sendData, uint16_t Length)
{
	if(sendData == NULL)
	{
		return USART0_ERROR_GOT_NULL_POINTER;
	}
	if(Length == 0)
	{
		return USART0_ERROR_ARGUMENT_OUT_OF_RANGE;
	}
	if(usart0_transmitInProgress == false)
	{
		usart0_transmitInProgress = true;
		usart0_put_raw_data(sendData,Length);
		return SUCCESS;
	} else 
	{
		BoolStatusCode queue_return = queue_has_space(usart0SendQueue);
		if(queue_return == true)
		{
			return queue_write(usart0SendQueue,sendData,Length);
		}else if(queue_return == false)
		{
			return USART0_ERROR_NOT_READY_FOR_OPERATION;
		}else
		{
			return queue_return;
		}
	}
}

StatusCode USART0_set_receiver_length(uint32_t Length)
{
	if(Length == 0)
	{
		return USART0_ERROR_ARGUMENT_OUT_OF_RANGE;
	}
	if(usart0_ReceivePtr != NULL)
	{
		free(usart0_ReceivePtr);
	}
	usart0_ReceiveLength = Length;
	USART0->US_RCR = usart0_ReceiveLength;
	usart0_ReceivePtr = malloc(Length*sizeof(uint8_t));
	if(usart0_ReceivePtr == NULL)
	{
		return USART0_ERROR_MALLOC_RETURNED_NULL;
	}
	USART0->US_RPR = (uint32_t)usart0_ReceivePtr;
	return SUCCESS;
}

StatusCode USART0_register_received_callback(USART_RECV_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return USART0_ERROR_GOT_NULL_POINTER;
	}
	usart_reciveCallBack = callBack;
	return SUCCESS;
}

void USART0_Handler(void)
{
	USART0->US_CR = US_CR_RSTSTA; // Clear receiver errors (if any)
	
	if (USART0->US_CSR & US_CSR_ENDRX)
	{
		
		if(usart_reciveCallBack != NULL)
		{
			usart_reciveCallBack(usart0_ReceivePtr,usart0_ReceiveLength);
		}
		
		USART0->US_RCR = usart0_ReceiveLength;
		USART0->US_RPR = (uint32_t)usart0_ReceivePtr;
	}
	if(USART0->US_CSR & US_CSR_ENDTX)
	{
		BoolStatusCode queue_return = queue_is_empty(usart0SendQueue);
		if(queue_return == true)
		{
			usart0_transmitInProgress = false;
			USART0->US_IDR = US_IDR_ENDTX;		//deactivate transmit done interrupt to keep it from firing constantly...
		}else if(queue_return == false){
			queue_node qData;
			//TODO: ERROR Handling
			queue_read(usart0SendQueue, &qData);
			usart0_put_raw_data(qData.data, qData.Length);
		} else {
			//TODO: ERROR Handling
			//queue_return;
		}
	}
}