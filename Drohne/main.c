/*
 * Drohne.c
 *
 * Created: 29.09.2017 19:08:07
 * Author : Markus Lorenz
 */ 

#include <avr/io.h>
#include "UART0.h"
#include "HelperFunctions.h"
#include "RCReader.h"
#include "ESCControl.h"
#include <util/delay.h>

int main(void)
{
	//uint8_t dataArray[] = {0,0};
	//UARTCOM_init(57600);
	sei();
	//UARTCOM_sendDebug("START");
	rc_init();
	esc_init();
	esc_set_m1(0);
	esc_set_m2(0);
	esc_set_m3(0);
	esc_set_m4(0);

	uart0_init(9600);

	uint16_t throttle = 0;
	char str[16];

    while (1) 
    {
		throttle = rc_read_throttle();
		itoa((int)throttle, str, 10);
		str[15] = '\0';
		uart0_puts(str);
		uart0_putc('\n');
		uart0_putc('\r');
		_delay_ms(100);
		esc_set_m1(throttle);
    }
}