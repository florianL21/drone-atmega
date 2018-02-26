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

#define BNO_MEASURE BNO_REG_GRV_DATA

BNO055_Data SensorValues;
//Init variables for the drone programm
RemoteControlValues RemoteValues;
int16_t Motor_speeds[4] = {0};
float ValueMapFactor = 0.3;
uint8_t maximumControlDegree = 10;

//PID Config:
float PID_PitchInput = 0,	PID_PitchOutput = 0,	PID_PitchSetPoint = 0;
float PID_RollInput = 0,	PID_RollOutput = 0,		PID_RollSetPoint = 0;
float PID_YawInput = 0,		PID_YawOutput = 0,		PID_YawSetPoint = 0;

float PitchKp = 0.0,	PitchKi = 0.1,		PitchKd = 0.2;
float RollKp = 12.34,		RollKi = 0.4,		RollKd = 0.5;
float YawKp = 0.6,		YawKi = 0.7,		YawKd = 0.8;
pidData PitchPid;
pidData RolePid;

#define MAX_ERROR_COUNT 20
StatusCode errorStack[MAX_ERROR_COUNT] = {0};
uint8_t errorCount = 0;

//sonsor offsetValues:
float sensorOffsetY = 0;
float sensorOffsetX = 0;

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
		error_handler_in(BNO055_start_measurement(true,true,BNO_MEASURE));
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
		SensorValues = BNO055_get_measurement_data();

		RemoteValues = rc_read_values();
		if(RemoteValues.error != true)
		{
			BNO055_Data CorrectedValues;
			CorrectedValues.X = SensorValues.X - sensorOffsetX;
			CorrectedValues.Y = SensorValues.Y - sensorOffsetY;
			CorrectedValues.Z = SensorValues.Z;
			
			PID_PitchInput		= CorrectedValues.X;
			PID_RollInput		= CorrectedValues.Y;
			PID_PitchSetPoint	= 0;
			PID_RollSetPoint	= 0;
			
			// Compute new PID output value
			if (needComputePitch)
			error_handler_in(PID_Compute(&PitchPid));
			if (needComputeRole)
			error_handler_in(PID_Compute(&RolePid));
			float factor = 0.0005;
			int16_t PitchAdjust = PID_PitchInput*2;//*(factor*RemoteValues.Throttle); 
			int16_t RoleAdjust = PID_RollInput*2;//*(factor*RemoteValues.Throttle);
			
			Motor_speeds[0] = RemoteValues.Throttle + PitchAdjust + RoleAdjust;// - MappedYaw;
			Motor_speeds[1] = RemoteValues.Throttle + PitchAdjust - RoleAdjust;// + MappedYaw;
			Motor_speeds[2] = RemoteValues.Throttle - PitchAdjust + RoleAdjust;// - MappedYaw;
			Motor_speeds[3] = RemoteValues.Throttle - PitchAdjust - RoleAdjust;// + MappedYaw;
			
			if(Motor_speeds[0] < 0)
				Motor_speeds[0] = 0;
			if(Motor_speeds[1] < 0)
				Motor_speeds[1] = 0;
			if(Motor_speeds[2] < 0)
				Motor_speeds[2] = 0;
			if(Motor_speeds[3] < 0)
				Motor_speeds[3] = 0;
			
			error_handler_in(esc_set(1, Motor_speeds[0]));
			error_handler_in(esc_set(2, Motor_speeds[1]));
			error_handler_in(esc_set(3, Motor_speeds[2]));
			error_handler_in(esc_set(4, Motor_speeds[3]));
			
			//PID_PitchSetPoint	= map(RemoteValues.Pitch, 0, RC_CONTROL_CENTER__PITCH * 2, -maximumControlDegree, maximumControlDegree);
			//PID_RoleSetPoint	= map(RemoteValues.Roll, 0, RC_CONTROL_CENTER__PITCH * 2, -maximumControlDegree, maximumControlDegree);
			
			/*char buffer[100] = "";
			sprintf(buffer,"X:%6d\tY:%6d\t%6d\n\r",SensorValues.X,SensorValues.Y,SensorValues.Z);
			UART0_puts(buffer);*/
			
			//int16_t MappedYaw   = map(RemoteValues.Yaw, 0, RC_CONTROL_CENTER__PITCH * 2, -(RemoteValues.Throttle * ValueMapFactor), (RemoteValues.Throttle * ValueMapFactor));
			
			//pitch --> role: 180;-180
			//role --> pitch: 90;-90
			
			/*PID_PitchInput		= correctedPitch;
			PID_RoleInput		= correctedRoll;
			PID_PitchSetPoint	= map(RemoteValues.Pitch, 0, RC_CONTROL_CENTER__PITCH * 2, -maximumControlDegree, maximumControlDegree);
			PID_RoleSetPoint	= map(RemoteValues.Roll, 0, RC_CONTROL_CENTER__PITCH * 2, -maximumControlDegree, maximumControlDegree);
			
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
			error_handler_in(esc_set(4, Motor_speeds[3]));*/
			
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
					buffer[1] = (Motor_speeds[0] & 0xFF00) >> 8;
					buffer[2] = Motor_speeds[0] & 0x00FF;
					buffer[3] = '1';
					buffer[4] = (Motor_speeds[1] & 0xFF00) >> 8;
					buffer[5] = Motor_speeds[1] & 0x00FF;
					buffer[6] = '2';
					buffer[7] = (Motor_speeds[2] & 0xFF00) >> 8;
					buffer[8] = Motor_speeds[2] & 0x00FF;
					buffer[9] = '3';
					buffer[10] = (Motor_speeds[3] & 0xFF00) >> 8;
					buffer[11] = Motor_speeds[3] & 0x00FF;
					//SerialCOM_put_debug("OK");
					SerialCOM_put_message(buffer, 0x01, 12);
				}
				
				if(printRCValues == true)
				{
					uint8_t buffer[14] = {0};
					buffer[0] = 'T';
					buffer[1] = (RemoteValues.Throttle & 0xFF00) >> 8;
					buffer[2] = RemoteValues.Throttle & 0x00FF;
					buffer[3] = 'R';
					buffer[4] = (RemoteValues.Roll & 0xFF00) >> 8;
					buffer[5] = RemoteValues.Roll & 0x00FF;
					buffer[6] = 'P';
					buffer[7] = (RemoteValues.Pitch & 0xFF00) >> 8;
					buffer[8] = RemoteValues.Pitch & 0x00FF;
					buffer[9] = 'Y';
					buffer[10] = (RemoteValues.Yaw & 0xFF00) >> 8;
					buffer[11] = RemoteValues.Yaw & 0x00FF;
					buffer[12] = 'G';
					buffer[13] = RemoteValues.Gear;
					SerialCOM_put_message(buffer, 0x02, 14);
				}
				
				if(printSensorValues == true)
				{
					uint8_t buffer[12] = {0};
					bool XIsNegative = CorrectedValues.X < 0;
					bool YIsNegative = CorrectedValues.Y < 0;
					bool ZIsNegative = CorrectedValues.Z < 0;
					
					buffer[0] = 'X';
					buffer[1] = XIsNegative;
					if(XIsNegative)
					{
						buffer[2] = (CorrectedValues.X*-1 & 0xFF00) >> 8;
						buffer[3] = CorrectedValues.X*-1 & 0x00FF;
					}
					else
					{
						buffer[2] = (CorrectedValues.X & 0xFF00) >> 8;
						buffer[3] = CorrectedValues.X & 0x00FF;
					}
					
					buffer[4] = 'Y';
					buffer[5] = YIsNegative;
					if(YIsNegative)
					{
						buffer[6] = (CorrectedValues.Y*-1 & 0xFF00) >> 8;
						buffer[7] = CorrectedValues.Y*-1 & 0x00FF;
					}
					else
					{
						buffer[6] = (CorrectedValues.Y & 0xFF00) >> 8;
						buffer[7] = CorrectedValues.Y & 0x00FF;
					}
					
					buffer[8] = 'Z';
					buffer[9] = ZIsNegative;
					if(ZIsNegative)
					{
						buffer[10] = (CorrectedValues.Z*-1 & 0xFF00) >> 8;
						buffer[11] = CorrectedValues.Z*-1 & 0x00FF;
					}
					else
					{
						buffer[10] = (CorrectedValues.Z& 0xFF00) >> 8;
						buffer[11] = CorrectedValues.Z & 0x00FF;
					}
					
					SerialCOM_put_message(buffer, 0x03, 12);
				}
			}
		}
	}
}

void sendPIDValuesToPC(uint8_t PIDIdentifier, float kp, float ki, float kd)
{
	uint8_t buffer[13] = {0};
	bool IsNegative = kp < 0;
	uint16_t NumValue;
	
	buffer[0] = PIDIdentifier;
	buffer[1] = 'P';
	buffer[2] = IsNegative;
	if(IsNegative)
	{
		NumValue = kp * -100;
	}
	else
	{
		NumValue = kp * 100;
	}
	buffer[3] = (NumValue & 0xFF00) >> 8;
	buffer[4] = NumValue & 0x00FF;
	
	buffer[5] = 'I';
	IsNegative = ki < 0;
	buffer[6] = IsNegative;
	if(IsNegative)
	{
		NumValue = ki * -100;
	}
	else
	{
		NumValue = ki * 100;
	}
	buffer[7] = (NumValue & 0xFF00) >> 8;
	buffer[8] = NumValue & 0x00FF;
	
	buffer[9] = 'D';
	IsNegative = kd < 0;
	buffer[10] = IsNegative;
	if(IsNegative)
	{
		NumValue = kd * -100;
	}
	else
	{
		NumValue = kd * 100;
	}
	buffer[11] = (NumValue & 0xFF00) >> 8;
	buffer[12] = NumValue & 0x00FF;
	SerialCOM_put_message(buffer, 0x04, 13);
}

void gotValueFromPC(uint8_t Identifier, float recValue)
{
	switch(Identifier)
	{
		case 0x01: //Roll P
			RollKp = recValue;
		break;
		case 0x02: //Roll I
			RollKi = recValue;
		break;
		case 0x03: //Roll D
			RollKd = recValue;
		break;
		case 0x04: //Pitch P
			PitchKp = recValue;
		break;
		case 0x05: //Pitch I
			PitchKi = recValue;
		break;
		case 0x06: //Pitch D
			PitchKd = recValue;
		break;
		case 0x07: //Yaw P
			YawKp = recValue;
		break;
		case 0x08: //Yaw I
			YawKi = recValue;
		break;
		case 0x09: //Yaw D
			YawKd = recValue;
		break;
	}
}

//0x01: Motor
//0x02: RC
//0x03: Sensor
//0x04: Set current values as Sensor offset
//0x05: Send PID Values to PC
//0x06: Get Value from PC
void message_from_PC(uint8_t* message, uint8_t Type)
{
	float recValue;
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
		case 0x04:
			if(message[0] == 'S')
			{
				sensorOffsetX = SensorValues.X;
				sensorOffsetY = SensorValues.Y;
			}
		break;
		case 0x05:
			if(message[0] == 'R')
			{
				sendPIDValuesToPC('R', RollKp, RollKi, RollKd);
			} else if(message[0] == 'P')
			{
				sendPIDValuesToPC('P', PitchKp, PitchKi, PitchKd);
			} else if(message[0] == 'Y')
			{
				sendPIDValuesToPC('Y', YawKp, YawKi, YawKd);
			}
		break;
		case 0x06:
			recValue = (message[2] << 8) | message[3];
			recValue /= 100;
			if(message[1] == 1)
				recValue *= -1;
			gotValueFromPC(message[0], recValue);
		break;
		default:
		break;
	}
}

void config_BNO()
{
	error_handler_in(SerialCOM_put_debug("Start BNO Init"));
	error_handler_in(BNO055_init_fusion_mode(false));
	error_handler_in(SerialCOM_put_debug("Calib OK"));
	error_handler_in(BNO055_register_error_callback(BNO_Error));
	error_handler_in(BNO055_register_data_ready_callback(DataReady));
	//configure MAG
	/*error_handler_in(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PAGE_ID, 0, BNO_PAGE_ID1));
	uint8_t config = 0x07	//30Hz data rate
					|0x18	//High accuracy data
					|0x00;	//power mode normal
	error_handler_in(BNOCOM_write_and_wait_for_response_1byte(BNO_REG1_MAG_CONFIG, 1, config));
	error_handler_in(BNOCOM_write_and_wait_for_response_1byte(BNO_REG_PAGE_ID, 1, BNO_PAGE_ID0));*/
}

int main(void)
{
	SystemInit();
	configure_wdt();
	error_handler_in(SerialCOM_init());
	error_handler_in(SerialCOM_register_call_back(message_from_PC));
	config_BNO();
	
	//rc control and esc init:
	rc_init();
	esc_init();
	
	PID_Init();
	error_handler_in(PID_Initialize(&PitchPid, &PID_PitchInput, &PID_PitchOutput, &PID_PitchSetPoint, PitchKp, PitchKi, PitchKd,-250,250,10));
	error_handler_in(PID_Initialize(&RolePid, &PID_RollInput, &PID_RollOutput, &PID_RollSetPoint, RollKp, RollKi, RollKd,-250,250,10));
	error_handler_in(BNO055_start_measurement(true, true, BNO_MEASURE));
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