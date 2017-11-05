/*
 * Drone.c
 *
 * Created: 15.10.2017 14:14:19
 * Author : flola
 */ 

#include "sam.h"
#include "uart0.h"
#include "ESCControl.h"
#include "RCReader.h"
#include "HelperFunctions.h"




void configure_wdt(void)
{
	WDT->WDT_MR = 0x00000000; // disable WDT
}

int main(void)
{
	/* Initialize the SAM system */
	SystemInit();
	configure_wdt();
	uart0_init(115200);
	rc_init();
	esc_init();	
	RemoteControlValues Values;
	int16_t MappedPitch=0;
	int16_t MappedRole=0;
	int16_t MappedYaw=0;
	uint16_t Motor_speeds[4] = {0};
	float ValueMapFactor = 0.3;
	while (1)
	{
		Values = rc_read_values();
		if(Values.error != true)
		{
			MappedPitch = map(Values.Pitch,0,RC_CONTROL_CENTER__PITCH*2,-(Values.Throttle*ValueMapFactor),(Values.Throttle*ValueMapFactor));
			MappedRole  = map(Values.Role,0,RC_CONTROL_CENTER__PITCH*2,-(Values.Throttle*ValueMapFactor),(Values.Throttle*ValueMapFactor));
			MappedYaw   = map(Values.Yaw,0,RC_CONTROL_CENTER__PITCH*2,-(Values.Throttle*ValueMapFactor),(Values.Throttle*ValueMapFactor));
			
			
			Motor_speeds[0] = Values.Throttle-MappedPitch-MappedRole-MappedYaw;
			Motor_speeds[1] = Values.Throttle-MappedPitch+MappedRole+MappedYaw;
			Motor_speeds[2] = Values.Throttle+MappedPitch-MappedRole-MappedYaw;
			Motor_speeds[3] = Values.Throttle+MappedPitch+MappedRole+MappedYaw;
			
			esc_set(1,Motor_speeds[0]);
			esc_set(2,Motor_speeds[1]);
			esc_set(3,Motor_speeds[2]);
			esc_set(4,Motor_speeds[3]);
			
			if(uart0_has_space())
			{
				uint8_t numBuf[20] = "";
				uint8_t buffer[50] = "";
				strcat(buffer,"1: ");
				itoa(Motor_speeds[0],numBuf,10);
				strcat(buffer,numBuf);
				itoa(Motor_speeds[1],numBuf,10);
				strcat(buffer,"\t2: ");
				strcat(buffer,numBuf);
				itoa(Motor_speeds[2],numBuf,10);
				strcat(buffer,"\t3: ");
				strcat(buffer,numBuf);
				itoa(Motor_speeds[3],numBuf,10);
				strcat(buffer,"\t4: ");
				strcat(buffer,numBuf);
				strcat(buffer,"\n\r");
				uart0_puts(buffer);
			}
		}
		
	}
}



/*
void PIOB_Handler(void)
{
	// Save all triggered interrupts
	uint32_t status = PIOB->PIO_ISR;
	
	if (status & PIO_PB26)
	{
		uart0_puts("t");
	}
}*/
/*
void configure_int(void)
{
	// Enable Clock for PIOB - needed for sampling falling edge
	PMC->PMC_PCER0 = 1 << ID_PIOB;
	// Enable IO pin control
	PIOB->PIO_PER = PIO_PB26;
	// Disable output (set to High Z)
	PIOB->PIO_ODR = PIO_PB26;
	// Enable pull-up
	PIOB->PIO_PUER = PIO_PB26;
	// Enable Glitch/Debouncing filter
	//PIOB->PIO_IFER = PIO_PB26;
	// Select Debouncing filter
	//PIOB->PIO_DIFSR = PIO_PB26;
	// Set Debouncing clock divider
	//PIOB->PIO_SCDR = 0x4FF;
	// Select additional detection mode (for single edge detection)
	PIOB->PIO_AIMER = PIO_PB26;
	// The interrupt source is an Edge detection event.
	PIOB->PIO_ESR = PIO_PB26;
	// The interrupt source is set to a falling and rising Edge detection
	PIOB->PIO_FELLSR = PIO_PB26;
	PIOB->PIO_REHLSR = PIO_PB26;
	// Enables the Input Change Interrupt on the I/O line.
	PIOB->PIO_IER = PIO_PB26;
	// Enable Interrupt Handling in NVIC
	NVIC_EnableIRQ(PIOB_IRQn);
}*/