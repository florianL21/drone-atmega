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

ErrorCode UART0_init(uint32_t BaudRate, uint32_t RecvLength)
{
	if(BaudRate < MIN_BAUD_RATE || BaudRate > MAX_BAUD_RATE)
	{
		return MODULE_UART0 | FUNCTION_Init | ERROR_ARGUMENT_OUT_OF_RANGE;
	}
	DEFAULT_ERROR_HANDLER(UART0_set_receiver_length(RecvLength), MODULE_UART0, FUNCTION_Init);
	
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
		return MODULE_UART0 | FUNCTION_Init | ERROR_MALLOC_RETURNED_NULL;
	return SUCCESS;
}

bool UART0_is_idle()
{
	return queue_is_empty(uart0SendQueue) == true;
}

bool UART0_has_space()
{
	return queue_has_space(uart0SendQueue);
}

uint8_t UART0_get_space()
{
	return UART0_QUEUE_MAX_ITEMS - queue_get_item_count(uart0SendQueue);
}

ErrorCode UART0_puts(char Data[])
{
	return ErrorHandling_set_top_level(UART0_put_data((uint8_t*)Data, strlen((char*)Data)), MODULE_UART0, FUNCTION_puts);
}

ErrorCode UART0_put_float(float num)
{
	char buffer[20] = "";
	sprintf(buffer,"%.4f",num);
	return ErrorHandling_set_top_level(UART0_puts(buffer), MODULE_UART0, FUNCTION_put_float);
}

ErrorCode UART0_put_int(int num)
{
	char buffer[20] = "";
	sprintf(buffer,"%d",num);
	return ErrorHandling_set_top_level(UART0_puts(buffer), MODULE_UART0, FUNCTION_put_int);
}

void uart0_put_raw_data(uint8_t* sendData, uint16_t Length)
{
	// copy to buffer - MUST be in SRAM as PDC is not connected to Flash
	for (uint16_t count = 0; count < Length; count++)
	{
		TXBUFFER[count] = sendData[count];
	}
	UART->UART_TPR = (uint32_t)TXBUFFER; 	// set Transmission pointer in PDC register
	UART->UART_TCR = Length; 				// set count of characters to be sent; starts transmission (since UART_PTCR_TXTEN is already set)
	UART->UART_IER = UART_IER_ENDTX;		//activate Transmit done interrupt
}

ErrorCode UART0_put_data(uint8_t* sendData, uint16_t Length)
{
	if(sendData == NULL)
	{
		return MODULE_UART0 | FUNCTION_put_data | ERROR_GOT_NULL_POINTER;
	}
	if(Length == 0)
	{
		return MODULE_UART0 | FUNCTION_put_data | ERROR_ARGUMENT_OUT_OF_RANGE;
	}
	if(uart0_transmitInProgress == false)
	{
		uart0_transmitInProgress = true;
		uart0_put_raw_data(sendData, Length);
		return SUCCESS;
	} else {
		if(queue_has_space(uart0SendQueue) == true)
		{
			return ErrorHandling_set_top_level(queue_write(uart0SendQueue, sendData, Length), MODULE_UART0, FUNCTION_put_data);
		}else
		{
			return MODULE_UART0 | FUNCTION_put_data | ERROR_NOT_READY_FOR_OPERATION;
		}
	}
}

ErrorCode UART0_put_int_blocking(int num)
{
	char buffer[50] = "";
	sprintf(buffer,"%d",num);
	return ErrorHandling_set_top_level(UART0_puts_blocking(buffer), MODULE_UART0, FUNCTION_put_int_blocking);
}

ErrorCode UART0_puts_blocking(char sendData[])
{
	if(sendData == NULL)
	{
		return MODULE_UART0 | FUNCTION_puts_blocking | ERROR_GOT_NULL_POINTER;
	}
	while(uart0_transmitInProgress);
	uart0_transmitInProgress = true;
	uart0_put_raw_data((uint8_t*)sendData, strlen((char*)sendData));
	return SUCCESS;
}

ErrorCode UART0_set_receiver_length(uint32_t Length)
{
	if(Length == 0)
	{
		return MODULE_UART0 | FUNCTION_set_receiver_length | ERROR_ARGUMENT_OUT_OF_RANGE;
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
		return MODULE_UART0 | FUNCTION_set_receiver_length | ERROR_MALLOC_RETURNED_NULL;
	}
	UART->UART_RPR = (uint32_t)uart0_ReceivePtr;
	return SUCCESS;
}

ErrorCode UART0_register_received_callback(UART_RECV_CALLBACK callBack)
{
	if(callBack == NULL)
	{
		return MODULE_UART0 | FUNCTION_register_received_callback | ERROR_GOT_NULL_POINTER;
	}
	uart_reciveCallBack = callBack;
	return SUCCESS;
}

uint32_t uart0_calculateTicks(uint32_t start, uint32_t stop)
{
	if(stop > start) // no overflow
		return (stop - start);
	else if(stop < start) // overflow
		return (stop + (4294967295 - start));
	else if(stop == start) // perfect overflow
		return 4294967295;
	return 0;
}

uint32_t uart0_getTicks()
{
	return TC0->TC_CHANNEL[0].TC_CV;
}

void Uart0_check_for_timeout()
{
	static uint32_t RCR_old = 0;
	static uint32_t time_old = 0;
	if(RCR_old != UART->UART_RCR)
	{
		RCR_old = UART->UART_RCR;
		time_old = uart0_getTicks();
	} else if(uart0_calculateTicks(time_old, uart0_getTicks()) >= 16800000)
	{
		//Timeout Error
		//TODO: fire UART_error callback
	}
}

void UART_Handler(void)
{
	UART->UART_CR = UART_CR_RSTSTA; // Clear receiver errors (if any)
	
	if (UART->UART_SR & UART_SR_ENDRX)
	{
		if(uart_reciveCallBack != NULL)
		{
			uart_reciveCallBack(uart0_ReceivePtr, uart0_ReceiveLength);
		}
		UART->UART_RCR = uart0_ReceiveLength;
		UART->UART_RPR = (uint32_t)uart0_ReceivePtr;
	}
	if(UART->UART_SR & UART_SR_ENDTX)
	{
		if(queue_is_empty(uart0SendQueue) == true)
		{
			uart0_transmitInProgress = false;
			UART->UART_IDR = UART_IER_ENDTX;		//deactivate transmit done interrupt to keep it from firing constantly...
		}else
		{
			queue_node qData;
			//TODO: ERROR Handling
			qData = queue_read(uart0SendQueue);
			if(qData.Length != 0)
				uart0_put_raw_data(qData.data, qData.Length);
			free(qData.data);
		}
	}
}