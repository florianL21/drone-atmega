/*
 * Drone.c
 *
 * Created: 15.10.2017 14:14:19
 * Author : flola
 */ 


/*

// Enable IO
PIOB->PIO_PER = PIO_PB27;
// Set to output
PIOB->PIO_OER = PIO_PB27;
// Disable pull-up
PIOB->PIO_PUDR = PIO_PB27;
PIOB->PIO_CODR = PIO_PB27;

*/


#include "sam.h"
#include "uart0.h"
#include "USART0.h"
#include "BNO055.h"
#include "BNO055_reg_table.h"

//#include "ESCControl.h"
//#include "RCReader.h"
//#include "HelperFunctions.h"
//#include "PID.h"

void _delay_not_ms(uint32_t loops)
{
	for (uint32_t i = 0; i < loops; i++)
	{
		asm("nop");
	}
}


void configure_wdt(void)
{
	WDT->WDT_MR = 0x00000000; // disable WDT
}

/*
void Test(uint8_t* Data, uint16_t Length)
{
	uint8_t test[3]="\r\n";
	USART0_put_data(Data,Length);
	USART0_put_data(test,2);
	USART0_set_receiver_length(1);
}

int main(void)
{
	
	SystemInit();
	configure_wdt();
	PIOB->PIO_PER = PIO_PB27;
	// Set to output
	PIOB->PIO_OER = PIO_PB27;
	// Disable pull-up
	PIOB->PIO_PUDR = PIO_PB27;
	PIOB->PIO_CODR = PIO_PB27;
	USART0_init(115200,3);
	USART0_register_received_callback(Test);
	
	while(1)
	{
	}
}*/

void Test(uint8_t* Data, uint8_t Length)
{
	//uint16_t data = 0x0000;
	//data = (Data[1] << 8 ) | (Data[0] & 0xff);
	char numBuf[20] = "";
	char buffer[50] = "";
	//strcat(buffer,"ID: ");
	itoa(Data[0],numBuf,10);
	strcat(buffer,numBuf);
	strcat(buffer,"\n\r");
	if(UART0_has_space())
		UART0_puts(buffer);
}

void BNO_Error(BNO_STATUS_BYTES Error, ERROR_CODE Transmit_error_code)
{
	UART0_puts("ERROR");
}

int main(void)
{
	SystemInit();
	configure_wdt();
	BNO055_Init(Test);
	BNO055_register_error_callback(BNO_Error);
	UART0_init(115200,1);
	//uart0_puts("Start:\n\r");
	BNO055_register_read(BNO055_reg_table0[BNO_REG][BNO_REG_CHIP_ID],BNO055_reg_table0[BNO_REG_LEN][BNO_REG_CHIP_ID]);
	//uart0_puts("END\n\r\n\r");
	while(1)
	{
		/*if(BNO055_is_idle() && UART0_is_idle())
		{
			BNO055_register_read(0x10,2);
		}
		_delay_not_ms(320000);*/
	}
}



/* PID Test Program:*/
/*

float Input=0;
float Output=0;
float SetPoint=0;
float Kp=0.5;
float Ki=0.3;
float Kd=0.008;

pidData myPid;

int main(void)
{
	SystemInit();
	configure_wdt();
	uart0_init(115200);
	rc_init();
	PID_Init();
	
	PID_Initialize(&myPid, &Input, &Output, &SetPoint, Kp, Ki, Kd,-1000,1000,10);
	
	RemoteControlValues Values;
	while(1)
	{
		Values = rc_read_values();
		SetPoint = Values.Throttle;
		if (PID_need_compute(&myPid))
		{
			// Compute new PID output value
			PID_Compute(&myPid);
			//Change actuator value
			if(uart0_has_space())
			{
				uint8_t numBuf[20] = "";
				uint8_t buffer[50] = "";
				strcat(buffer,"In: ");
				itoa(Input,numBuf,10);
				strcat(buffer,numBuf);
				strcat(buffer,"\tOut: ");
				itoa(Output,numBuf,10);
				strcat(buffer,numBuf);
				strcat(buffer,"\tSet: ");
				itoa(SetPoint,numBuf,10);
				strcat(buffer,numBuf);
				strcat(buffer,"\n\r");
				uart0_puts(buffer);
			}
			Input += Output;
		}
		
	}
}
*/

/*Drone Test Program:*/
/*
int main(void)
{
	/ * Initialize the SAM system * /
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
}*/