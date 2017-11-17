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
#include "PID.h"


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