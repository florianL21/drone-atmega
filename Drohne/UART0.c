/*
 * UART0.c
 *
 * Created: 29.09.2017 20:07:25
 *  Author: flola
 */ 

#include "UART0.h"

volatile uint8_t sendBuffer[MAX_UART_SEND_BUFFER] = {0};
volatile bool sendingActive = false;
volatile uint16_t bufferSpace = MAX_UART_SEND_BUFFER;

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
}

bool uart0_buffer_has_space() // returns true if buffer has space
{
	return bufferSpace != 0;
}

void uart0_puts(char* Data)
{
	uint8_t Length = sizeof(Data)/sizeof(Data[0]);
	for (uint8_t i = 0; i < Length; i++)
	{
		uart0_putc(Data[i]);
	}
}

uint16_t get_write_index()
{
	static uint16_t writeIndex = 0;
	if(writeIndex + 1 >= MAX_UART_SEND_BUFFER)
	{
		writeIndex = 1;
		return 0;
	}else
	{
		writeIndex++;
		return writeIndex-1;
	}
}

uint16_t get_read_index()
{
	static uint16_t readIndex = 0;
	if(readIndex + 1 >= MAX_UART_SEND_BUFFER)
	{
		readIndex = 1;
		return 0;
	}else
	{
		readIndex++;
		return readIndex-1;
	}
}

void uart0_putc(uint8_t character)
{
	if(uart0_buffer_has_space()) //check if buffer has space, if not simply do nothing
	{
		sendBuffer[get_write_index()] = character;
		bufferSpace--;
		if(sendingActive == false)
		{
			UDR0 = sendBuffer[get_read_index()];
			
			sendingActive = true;
		}
	}
}

void uart0_register_recived_callback(UART_RECV_CALLBACK callBack)
{
	reciveCallBack = callBack;
}

ISR(USART0_TX_vect)
{
	bufferSpace++;
	if(bufferSpace == MAX_UART_SEND_BUFFER)
	{
		sendingActive = false;
	} else 
	{
		UDR0 = sendBuffer[get_read_index()];
	}
}


ISR(USART0_RX_vect)
{
	if(reciveCallBack != NULL)
	{
		reciveCallBack(UDR0);
	}
}