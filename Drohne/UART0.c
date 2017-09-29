/*
 * UART0.c
 *
 * Created: 29.09.2017 20:07:25
 *  Author: flola
 */ 


void uart0_init(uint16_t BaudRate)
{
	UBRR0  = F_OSC / 16 / BaudRate - 1; //round function
	UCSR0B |= (1 << TXEN0); //Activate UART send
	UCSR0B |= (1 << RXEN0); //Activate UART receive
	//8 bit no parity, 1 stop bit
	UCSR0C &= ~((1<<USBS0) | (1<<UPM00) | (1<<UPM01));
	UCSR0B &= ~(1<<UCSZ02);
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
	
	UCSR0B |= (1<<TXCIE0); //transmit interrupt
	UCSR0B |= (1<<RXCIE0); //receve interrupt
}