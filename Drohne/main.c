/*
 * Drohne.c
 *
 * Created: 29.09.2017 19:08:07
 * Author : Markus Lorenz
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "ESCControl.h"
#include "RCReader.h"

int main(void)
{
	// Initialize modules
	rc_init();
	esc_init();
	
	
    /* Replace with your application code */
    while (1) 
    {
    }
}

