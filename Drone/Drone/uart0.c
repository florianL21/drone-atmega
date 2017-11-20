/*
 * UART0.c
 *
 * Created: 18.11.2017 19:14:58
 *  Author: flola
 */ 
#include "UART0.h"

#define MIN_BAUD_RATE	9600
#define MAX_BAUD_RATE	1843200

uint8_t TXBUFFER[270]; // UART PDC Transmit buffer
UART_RECV_CALLBACK uart_reciveCallBack = NULL;
uint32_t uart0_ReceiveLength = 1;
uint8_t *uart0_ReceivePtr = NULL;
Queue* uart0SendQueue;
bool uart0_transmitInProgress = false;

void uart0_put_raw_data_(uint8_t* sendData, uint16_t Length);

ERROR_CODE UART0_init(uint32_t BaudRate, uint32_t RecvLength)
{
	if(BaudRate < MIN_BAUD_RATE || BaudRate > MAX_BAUD_RATE)
	{
		return UART0_ERROR_ARGUMENT_OUT_OF_RANGE;
	}
	DEFUALT_ERROR_HANDLER(UART0_set_receiver_length(RecvLength),error_return);
	
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
	UART->UART_IER = UART_IER_RXRDY;//| UART_IER_TXRDY;
	//UART->UART_IDR = UART_IDR_ENDRX|UART_IDR_ENDTX|UART_IDR_TXBUFE|UART_IDR_TXEMPTY|UART_IDR_TXRDY;
	// Enable UART Interrupt Handling in NVIC
	NVIC_EnableIRQ(UART_IRQn);
	// Enable Peripheral DMA Controller Transmission
	UART->UART_PTCR = UART_PTCR_TXTEN | UART_PTCR_RXTEN;
	
	uart0SendQueue = queue_new(UART0_QUEUE_MAX_ITEMS);
	if(uart0SendQueue == NULL)
	{
		return UART0_ERROR_MALLOC_RETURNED_NULL;
	}

	return SUCCESS;
}

bool UART0_is_idle()
{
	return uart0_transmitInProgress;
}

bool UART0_has_space()
{
	return queue_has_space(uart0SendQueue);
}

ERROR_CODE UART0_puts(char Data[])
{
	return UART0_put_data((uint8_t*)Data,strlen((char*)Data));
}

void uart0_put_raw_data(uint8_t* sendData, uint16_t Length)
{
	// copy to buffer - MUST be in SRAM as PDC is not connected to Flash
	for (uint16_t count = 0; count < Length; count++)
	{
		TXBUFFER[count] = sendData[count];
	}
	UART->UART_TPR = (uint32_t)TXBUFFER; 	// set Trasmission pointer in PDC register
	UART->UART_TCR = Length; 				// set count of characters to be sent; starts transmission (since UART_PTCR_TXTEN is already set)
	UART->UART_IER = UART_IER_ENDTX;			//activate Transmit done interrupt
}

ERROR_CODE UART0_put_data(uint8_t* sendData, uint16_t Length)
{
	if(sendData == NULL)
	{
		return UART0_ERROR_GOT_NULL_POINTER;
	}
	if(Length == 0)
	{
		return UART0_ERROR_ARGUMENT_OUT_OF_RANGE;
	}
	if(uart0_transmitInProgress == false)
	{
		uart0_transmitInProgress = true;
		uart0_put_raw_data(sendData,Length);
		return SUCCESS;
	} else if(queue_has_space(uart0SendQueue))
	{
		queue_write(uart0SendQueue,sendData,Length);
		return SUCCESS;
	}else{
		return UART0_ERROR_NOT_READY_FOR_OPERATION;
	}
}

ERROR_CODE UART0_set_receiver_length(uint32_t Length)
{
	if(Length == 0)
	{
		return UART0_ERROR_ARGUMENT_OUT_OF_RANGE;
	}
	if(uart0_ReceivePtr != NULL)
	{
		free(uart0_ReceivePtr);
	}
	uart0_ReceiveLength = Length;
	UART->UART_RCR = uart0_ReceiveLength;
	uart0_ReceivePtr = malloc(Length*sizeof(uint8_t));
	if(uart0_ReceivePtr == NULL)
	{
		return UART0_ERROR_MALLOC_RETURNED_NULL;
	}
	UART->UART_RPR = (uint32_t)uart0_ReceivePtr;
	return SUCCESS;
}

ERROR_CODE UART0_register_received_callback(UART_RECV_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return UART0_ERROR_GOT_NULL_POINTER;
	}
	uart_reciveCallBack = callBack;
	return SUCCESS;
}

void UART_Handler(void)
{
	UART->UART_CR = UART_CR_RSTSTA; // Clear receiver errors (if any)
	
	if (UART->UART_SR & UART_SR_ENDRX)
	{
		
		if(uart_reciveCallBack != NULL)
		{
			uart_reciveCallBack(uart0_ReceivePtr,uart0_ReceiveLength);
		}
		UART->UART_RCR = uart0_ReceiveLength;
		UART->UART_RPR = (uint32_t)uart0_ReceivePtr;
	}
	if(UART->UART_SR & UART_SR_ENDTX)
	{
		if(queue_is_empty(uart0SendQueue))
		{
			uart0_transmitInProgress = false;
			UART->UART_IDR = UART_IER_ENDTX;		//deactivate transmit done interrupt to keep it from firing constantly...
		}else{
			queue_node qData = queue_read(uart0SendQueue);
			uart0_put_raw_data(qData.data, qData.Length);
		}
	}
}