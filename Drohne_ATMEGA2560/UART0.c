/*
 * UART0.c
 *
 * Created: 29.09.2017 20:07:25
 *  Author: flola
 */ 

#include "UART0.h"

volatile bool sendingActive = false;
volatile Queue* uartSendBuffer;

UART_RECV_CALLBACK reciveCallBack = NULL;

void uart0_init(uint32_t BaudRate)
{
	UBRR0  = F_CPU / 16 / BaudRate - 1; 
	UCSR0B |= (1 << TXEN0); //Activate UART send
	UCSR0B |= (1 << RXEN0); //Activate UART receive
	//8 bit no parity, 1 stop bit
	UCSR0C &= ~((1<<USBS0) | (1<<UPM00) | (1<<UPM01));
	UCSR0B &= ~(1<<UCSZ02);
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
	
	UCSR0B |= (1<<TXCIE0); //transmit interrupt
	UCSR0B |= (1<<RXCIE0); //receive interrupt
	uartSendBuffer = queue_new();
}


void uart0_puts(char Data[])
{
	uint8_t Length = strlen(Data);
	for (uint8_t i = 0; i < Length; i++)
	{
		uart0_putc(Data[i]);
	}
}

void uart0_putc(uint8_t character)
{
	queue_write(uartSendBuffer, character);
	if(sendingActive == false)
	{
		UDR0 = queue_read(uartSendBuffer);
		sendingActive = true;
	}
}

void uart0_register_recived_callback(UART_RECV_CALLBACK callBack)
{
	reciveCallBack = callBack;
}

ISR(USART0_TX_vect)
{
	if(queue_is_empty(uartSendBuffer))
	{
		sendingActive = false;
	} else
	{
		UDR0 = queue_read(uartSendBuffer);
	}
}

ISR(USART0_RX_vect)
{
	if(reciveCallBack != NULL)
	{
		reciveCallBack(UDR0);
	}
}
