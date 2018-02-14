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
#include "BNO055.h"
#include "ESCControl.h"
#include "RCReader.h"
#include "PID.h"
#include "SerialCOM.h"

BNO055_eulerData SensorValues;
//Init variables for the drone programm
RemoteControlValues RemoteValues;
uint16_t Motor_speeds[4] = {0};
float ValueMapFactor = 0.3;
uint8_t maximumControlDegree = 10;

//PID Config:
float PID_PitchInput = 0,	PID_PitchOutput = 0,	PID_PitchSetPoint = 0;
float PID_RoleInput = 0,	PID_RoleOutput = 0,		PID_RoleSetPoint = 0;
float PID_YawInput = 0,		PID_YawOutput = 0,		PID_YawSetPoint = 0;

float PitchKp = 0.5,	PitchKi = 0.3,		PitchKd = 0.8;
float RoleKp = 0.5,		RoleKi = 0.3,		RoleKd = 0.8;
float YawKp = 0.5,		YawKi = 0.3,		YawKd = 0.8;
pidData PitchPid;
pidData RolePid;

#define MAX_ERROR_COUNT 20
StatusCode errorStack[MAX_ERROR_COUNT] = {0};
uint8_t errorCount = 0;

//Status variables for whitch values are printing:
bool printSensorValues = 0;
bool printRCValues = 0;
bool printMotorValues = 0;
uint8_t sendCount = 0;

void error_handler_in(StatusCode errorIn)
{
	if(errorIn != SUCCESS && errorCount < MAX_ERROR_COUNT)
	{
		errorStack[errorCount++] = errorIn;
	}
}

void error_handler_print()
{
	if(errorCount != 0)
	{
		char ErrorToCharBuffer[10] = "";
		for (uint8_t i = 0; i < errorCount; i++)
		{
			itoa(errorStack[i], ErrorToCharBuffer, 16);
			ErrorToCharBuffer[2]='\0';
			SerialCOM_put_error(ErrorToCharBuffer);
		}
		errorCount = 0;
	}
}

void configure_wdt(void)
{
	WDT->WDT_MR = 0x00000000; // disable WDT
}

void BNO_Error(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code)
{
	if(Error == BNO_STATUS_BUS_OVER_RUN_ERROR && Transmit_error_code == BNO055_ERROR)
		error_handler_in(BNO055_start_euler_measurement(true,true));
	else 
	{
		_Delay(840000);
		error_handler_in(BNO055_ERROR);
		error_handler_in(Error);
	}
}

void DataReady()
{
	bool needComputePitch = PID_need_compute(&PitchPid);
	bool needComputeRole = PID_need_compute(&RolePid);
	if(needComputeRole == true || needComputePitch == true)
	{
		
		//read Values from sensor and remote control:
		SensorValues = BNO055_get_euler_measurement_data();
		RemoteValues = rc_read_values();
		if(RemoteValues.error != true)
		{
			
			//UART0_puts("Values OK\n\r");
			//int16_t MappedYaw   = map(RemoteValues.Yaw, 0, RC_CONTROL_CENTER__PITCH * 2, -(RemoteValues.Throttle * ValueMapFactor), (RemoteValues.Throttle * ValueMapFactor));
			
			//pitch --> role: 180;-180
			//role --> pitch: 90;-90
			
			PID_PitchInput		= SensorValues.role;
			PID_RoleInput		= SensorValues.pitch;
			PID_PitchSetPoint	= map(RemoteValues.Pitch, 0, RC_CONTROL_CENTER__PITCH * 2, -maximumControlDegree, maximumControlDegree);
			PID_RoleSetPoint	= map(RemoteValues.Role, 0, RC_CONTROL_CENTER__PITCH * 2, -maximumControlDegree, maximumControlDegree);
			
			// Compute new PID output value
			if (needComputePitch)
				error_handler_in(PID_Compute(&PitchPid));
			if (needComputeRole)
				error_handler_in(PID_Compute(&RolePid));

			int16_t MappedPitch = map(PID_PitchOutput, -180, 180, -(RemoteValues.Throttle * ValueMapFactor), (RemoteValues.Throttle * ValueMapFactor));
			int16_t MappedRole  = map(PID_RoleOutput, -90, 90, -(RemoteValues.Throttle * ValueMapFactor), (RemoteValues.Throttle * ValueMapFactor));
			//int16_t MappedYaw   = map(PID_YawOutput, -1000, 1000, -(RemoteValues.Throttle * ValueMapFactor), (RemoteValues.Throttle * ValueMapFactor));
			Motor_speeds[0] = RemoteValues.Throttle - MappedPitch - MappedRole;// - MappedYaw;
			Motor_speeds[1] = RemoteValues.Throttle - MappedPitch + MappedRole;// + MappedYaw;
			Motor_speeds[2] = RemoteValues.Throttle + MappedPitch - MappedRole;// - MappedYaw;
			Motor_speeds[3] = RemoteValues.Throttle + MappedPitch + MappedRole;// + MappedYaw;
		
			error_handler_in(esc_set(1, Motor_speeds[0]));
			error_handler_in(esc_set(2, Motor_speeds[1]));
			error_handler_in(esc_set(3, Motor_speeds[2]));
			error_handler_in(esc_set(4, Motor_speeds[3]));
			
			if(SerialCOM_get_free_space() >= printMotorValues + printRCValues + printSensorValues && sendCount++ >= 10)
			{
				sendCount = 0;
				//char test = SerialCOM_get_free_space()+'0';
				//SerialCOM_put_debug_n(&test,1);
				//SerialCOM_put_debug("OK");
				if(printMotorValues == true)
				{
					uint8_t buffer[12] = {0};
					buffer[0] = '0';
					buffer[1] = (Motor_speeds[0] >> 8) & 0x00FF;
					buffer[2] = Motor_speeds[0] & 0x00FF;
					buffer[3] = '1';
					buffer[4] = (Motor_speeds[1] >> 8) & 0x00FF;
					buffer[5] = Motor_speeds[1] & 0x00FF;
					buffer[6] = '2';
					buffer[7] = (Motor_speeds[2] >> 8) & 0x00FF;
					buffer[8] = Motor_speeds[2] & 0x00FF;
					buffer[9] = '3';
					buffer[10] = (Motor_speeds[3] >> 8) & 0x00FF;
					buffer[11] = Motor_speeds[3] & 0x00FF;
					//SerialCOM_put_debug("OK");
					SerialCOM_put_message(buffer, 0x01, 12);
				}
				
				if(printRCValues == true)
				{
					uint8_t buffer[14] = {0};
					buffer[0] = 'T';
					buffer[1] = (RemoteValues.Throttle >> 8) & 0x00FF;
					buffer[2] = RemoteValues.Throttle & 0x00FF;
					buffer[3] = 'R';
					buffer[4] = (RemoteValues.Role >> 8) & 0x00FF;
					buffer[5] = RemoteValues.Role & 0x00FF;
					buffer[6] = 'P';
					buffer[7] = (RemoteValues.Pitch >> 8) & 0x00FF;
					buffer[8] = RemoteValues.Pitch & 0x00FF;
					buffer[9] = 'Y';
					buffer[10] = (RemoteValues.Yaw >> 8) & 0x00FF;
					buffer[11] = RemoteValues.Yaw & 0x00FF;
					buffer[12] = 'G';
					buffer[13] = RemoteValues.Gear;
					SerialCOM_put_message(buffer, 0x02, 14);
				}
				
				if(printSensorValues == true)
				{
					//multiplying float values with factor 100 for transmission
					bool roleIsNegative, pitchIsNegative, yawIsNegative;
					roleIsNegative = SensorValues.pitch < 0;
					pitchIsNegative = SensorValues.role < 0;
					yawIsNegative = SensorValues.heading < 0;
					
					uint16_t role, pitch, yaw;
					
					if(roleIsNegative)
						role = SensorValues.pitch*100*-1;
					else
						role = SensorValues.pitch*100;
					if(pitchIsNegative)
						pitch = SensorValues.role*100*-1;
					else
						pitch = SensorValues.role*100;
					if(yawIsNegative)
						yaw = SensorValues.heading*100*-1;
					else
						yaw = SensorValues.heading*100;
					
					uint8_t buffer[12] = {0};
					buffer[0] = 'R';
					buffer[1] = roleIsNegative;
					buffer[2] = (role >> 8) & 0x00FF;
					buffer[3] = role & 0x00FF;
					buffer[4] = 'P';
					buffer[5] = pitchIsNegative;
					buffer[6] = (pitch >> 8) & 0x00FF;
					buffer[7] = pitch & 0x00FF;
					buffer[8] = 'Y';
					buffer[9] = yawIsNegative;
					buffer[10] = (yaw >> 8) & 0x00FF;
					buffer[11] = yaw & 0x00FF;
					SerialCOM_put_message(buffer, 0x03, 12);
				}
			}
		}
	}
}


//0x01: Motor
//0x02: RC
//0x03: Sensor
void message_from_PC(uint8_t* message, uint8_t Type)
{
	
	switch(Type)
	{
		case 0x01:
			if(message[0] == 'I')
				printMotorValues = true;
			else if(message[0] == 'O')
				printMotorValues = false;
		break;
		case 0x02:
			if(message[0] == 'I')
				printRCValues = true;
			else if(message[0] == 'O')
				printRCValues = false;
		break;
		case 0x03:
			if(message[0] == 'I')
				printSensorValues = true;
			else if(message[0] == 'O')
				printSensorValues = false;
		break;
		default:
		break;
	}
}

int main(void)
{
	SystemInit();
	configure_wdt();
	error_handler_in(SerialCOM_init());
	error_handler_in(SerialCOM_register_call_back(message_from_PC));
	//BNO init:
	error_handler_in(SerialCOM_put_debug("Start BNO Init"));
	error_handler_in(BNO055_init(false));
	error_handler_in(SerialCOM_put_debug("Calib OK"));
	error_handler_in(BNO055_register_error_callback(BNO_Error));
	
	error_handler_in(BNO055_register_data_ready_callback(DataReady));
	//rc control and esc init:
	rc_init();
	esc_init();
	
	PID_Init();
	error_handler_in(PID_Initialize(&PitchPid, &PID_PitchInput, &PID_PitchOutput, &PID_PitchSetPoint, PitchKp, PitchKi, PitchKd,-180,180,10));
	error_handler_in(PID_Initialize(&RolePid, &PID_RoleInput, &PID_RoleOutput, &PID_RoleSetPoint, RoleKp, RoleKi, RoleKd,-90,90,10));
	error_handler_in(BNO055_start_euler_measurement(true,true));
	error_handler_print();
	error_handler_in(SerialCOM_put_debug("Init Done!"));
	while(1)
	{
		error_handler_print();
		//if(UART0_has_space())
		{
			//error_handler_in(SerialCOM_put_debug("This message tests the correctness of the memmory management"));
			
		}
		//_Delay(1000000);
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


/*

/ *Drone Test Program:* /

int main(void)
{
	/ * Initialize the SAM system * /
	SystemInit();
	configure_wdt();
	rc_init();
	esc_init();
	RemoteControlValues RemoteValues;
	int16_t MappedPitch=0;
	int16_t MappedRole=0;
	int16_t MappedYaw=0;
	uint16_t Motor_speeds[4] = {0};
	float ValueMapFactor = 0.3;
	while (1)
	{
		RemoteValues = rc_read_values();
		if(RemoteValues.error != true)
		{
			MappedPitch = map(RemoteValues.Pitch,0,RC_CONTROL_CENTER__PITCH*2,-(RemoteValues.Throttle*ValueMapFactor),(RemoteValues.Throttle*ValueMapFactor));
			MappedRole  = map(RemoteValues.Role,0,RC_CONTROL_CENTER__PITCH*2,-(RemoteValues.Throttle*ValueMapFactor),(RemoteValues.Throttle*ValueMapFactor));
			MappedYaw   = map(RemoteValues.Yaw,0,RC_CONTROL_CENTER__PITCH*2,-(RemoteValues.Throttle*ValueMapFactor),(RemoteValues.Throttle*ValueMapFactor));
			
			
			Motor_speeds[0] = RemoteValues.Throttle-MappedPitch-MappedRole-MappedYaw;
			Motor_speeds[1] = RemoteValues.Throttle-MappedPitch+MappedRole+MappedYaw;
			Motor_speeds[2] = RemoteValues.Throttle+MappedPitch-MappedRole-MappedYaw;
			Motor_speeds[3] = RemoteValues.Throttle+MappedPitch+MappedRole+MappedYaw;
			
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
*/






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