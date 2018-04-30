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
#include "Modules/BNO055/BNO055.h"
#include "Modules/ESCControl/ESCControl.h"
#include "Modules/RCReader/RCReader.h"
#include "Modules/PID/PID.h"
#include "Modules/SerialCOM/SerialCOM.h"
#include "Modules/FlashStorage/FlashStorage.h"
#include "Modules/WDT/WDT.h"
#include "Modules/GPT/GPT.h"

#define BNO_MEASURE BNO055_read(BNO_REG_QUA_DATA_W, 8)	//request 14 for acc data; 8 for quaternions only
#define SERIALCOM_SEND_INTERVALL_MS 20

//Init variables for the drone programm
BNO055_Quat LastSensorMeasurement;
BNO055_Quat CorrectedValues;
BNO055_Data YPR_Angles;
RemoteControlValues RemoteValues;
float PosX = 0, PosY = 0, PosZ = 0;
int16_t Motor_speeds[4] = {0};
bool bno_contionous_measurement_active = true;
bool ArmMotors = false;
float timeOfLastMeasurement = 0;
Timer Wdt_resetTimer;


//PID Config:
float PID_XInput = 0,	PID_XOutput = 0,	PID_XSetPoint = 0;
float PID_YInput = 0,	PID_YOutput = 0,		PID_YSetPoint = 0;
float PID_ZInput = 0,		PID_ZOutput = 0,		PID_ZSetPoint = 0;
float PID_WInput = 0,		PID_WOutput = 0,		PID_WSetPoint = 0;
float PID_XKp = 1.3,  PID_XKi = 0.2,    PID_XKd = 1.2;
float PID_YKp = 1.3,  PID_YKi = 0.2,    PID_YKd = 1.2;
float PID_ZKp = 1.3,  PID_ZKi = 0.2,    PID_ZKd = 1.2;
float PID_WKp = 1.3,  PID_WKi = 0.2,    PID_WKd = 1.2;
pidData XPid; //PitchPid
pidData YPid; //RollPid
pidData ZPid; //YawPid
pidData WPid;

//sonsor offsetValues:
float sensorOffsetPitch = 0;
float sensorOffsetRoll = 0;
float sensorOffsetYaw  = 0;

//Status variables for which values are printing:
bool printSensorValues = 0;
bool printRCValues = 0;
bool printMotorValues = 0;
float TimeOfLastTransmission = 0;

void BNO_Error(ErrorCode Error)
{
	if(BNO055_IsCalibrating() == false)
	{
		ErrorHandling_throw(BNO_MEASURE);																//restart measurement
		if((Error & 0xFF) != ERROR_BUS_OVER_RUN)
		{
			ErrorHandling_throw(ErrorHandling_set_top_level(Error, MODULE_MAIN, FUNCTION_error));				//throw the error
		}
	}
}

void ErrorHandling_print()
{
	ErrorCode Error;
	char ErrorMessage [200] = "";
	
	if(ErrorHandling_catch(&Error) == true)
	{
		ErrorHandling_get_error_description(Error, ErrorMessage);								//max 32 chars
		strcat(ErrorMessage, " error: \n\toriginates at: \"");									//26 chars
		ErrorHandling_get_module_description(Error, ErrorMessage);								//max 15 chars
		strcat(ErrorMessage, "_");																//1 chars
		ErrorHandling_get_function_description(Error, ErrorMessage);							//max 33 chars
		strcat(ErrorMessage, "\"\n\tTop call function: \"");									//23 chars
		ErrorHandling_get_top_module_description(Error, ErrorMessage);		//max 15 chars
		strcat(ErrorMessage, "_");																//1 chars
		ErrorHandling_get_top_function_description(Error, ErrorMessage);		//max 33 chars
		strcat(ErrorMessage, "\"");																//1 chars
		SerialCOM_put_error(ErrorMessage);
	}
}

void SaveValuesToFlash(uint8_t type)
{
	if(type == 0x00 || type == 0x01)
		ErrorHandling_throw(FlashStorage_write_float(4, PID_YKp));
	if(type == 0x00 || type == 0x02)
		ErrorHandling_throw(FlashStorage_write_float(8, PID_YKi));
	if(type == 0x00 || type == 0x03)
		ErrorHandling_throw(FlashStorage_write_float(12, PID_YKd));
	if(type == 0x00 || type == 0x04)
		ErrorHandling_throw(FlashStorage_write_float(16, PID_XKp));
	if(type == 0x00 || type == 0x05)
		ErrorHandling_throw(FlashStorage_write_float(20, PID_XKi));
	if(type == 0x00 || type == 0x06)
		ErrorHandling_throw(FlashStorage_write_float(24, PID_XKd));
	if(type == 0x00 || type == 0x07)
		ErrorHandling_throw(FlashStorage_write_float(28, PID_ZKp));
	if(type == 0x00 || type == 0x08)
		ErrorHandling_throw(FlashStorage_write_float(32, PID_ZKi));
	if(type == 0x00 || type == 0x09)
		ErrorHandling_throw(FlashStorage_write_float(36, PID_ZKd));
	if(type == 0x00 || type == 0x0A)
		ErrorHandling_throw(FlashStorage_write_float(40, sensorOffsetRoll));
	if(type == 0x00 || type == 0x0B)
		ErrorHandling_throw(FlashStorage_write_float(44, sensorOffsetPitch));
}

void LoadValuesFromFlash()
{
	PID_YKp = FlashStorage_read_float(4);
	PID_YKi = FlashStorage_read_float(8);
	PID_YKd = FlashStorage_read_float(12);
	PID_XKp = FlashStorage_read_float(16);
	PID_XKi = FlashStorage_read_float(20);
	PID_XKd = FlashStorage_read_float(24);
	PID_ZKp = FlashStorage_read_float(28);
	PID_ZKi = FlashStorage_read_float(32);
	PID_ZKd = FlashStorage_read_float(36);
	sensorOffsetRoll = FlashStorage_read_float(40);
	sensorOffsetPitch = FlashStorage_read_float(44);
}

float CorrectSensorOffsets(float UncorrectedValue, float ValueOffset, float min_max)
{
	float CorrectedValue = UncorrectedValue - ValueOffset;
	if(CorrectedValue > min_max)
	{
		CorrectedValue = CorrectedValue - min_max*2;
	}else if(CorrectedValue < -min_max)
	{
		CorrectedValue = CorrectedValue + min_max*2;
	}
	return CorrectedValue;
}

float _CorrectSensorOffsets(float UncorrectedValue, float ValueOffset, float min_max)//TODO
{
	float CorrectedValue = UncorrectedValue - ValueOffset;
	/*if(CorrectedValue > min_max)
	{
		CorrectedValue = min_max - (CorrectedValue - min_max);
	}else if(CorrectedValue < -min_max)
	{
		CorrectedValue = (-min_max) - (CorrectedValue + min_max);
	}*/
	return CorrectedValue;
}


void DataReady(uint8_t Data[], uint8_t Length)
{
	if(SerialCOM_get_free_space() >= 1 && BNO055_IsCalibrating() == true && Length == 1 && GPT_GetPreciseTime() - TimeOfLastTransmission >= SERIALCOM_SEND_INTERVALL_MS)
	{
		TimeOfLastTransmission = GPT_GetPreciseTime();
		ErrorHandling_throw(SerialCOM_put_message(Data, 0x07, 1));
	}
	else
	{
		LastSensorMeasurement = BNO055_GetQuat(Data);
	}
	timeOfLastMeasurement = GPT_GetPreciseTime();
	
	//start the next measurement
	if(bno_contionous_measurement_active)
		ErrorHandling_throw(BNO_MEASURE);
}

void SendLogData()
{
	if(SerialCOM_get_free_space() >= printMotorValues + printRCValues + printSensorValues && GPT_GetPreciseTime() - TimeOfLastTransmission >= SERIALCOM_SEND_INTERVALL_MS)
	{
		TimeOfLastTransmission = GPT_GetPreciseTime();
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
			ErrorHandling_throw(SerialCOM_put_message(buffer, 0x01, 12));
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
			ErrorHandling_throw(SerialCOM_put_message(buffer, 0x02, 14));
		}
		
		if(printSensorValues == true)
		{
			uint8_t buffer[15] = {0};
			
			buffer[0] = 'R';
			SerialCOM_serializeFloat(&YPR_Angles.Roll, &buffer[1]);
			
			buffer[5] = 'P';
			SerialCOM_serializeFloat(&YPR_Angles.Pitch, &buffer[6]);
			
			buffer[10] = 'Y';
			SerialCOM_serializeFloat(&YPR_Angles.Yaw, &buffer[11]);
			
			ErrorHandling_throw(SerialCOM_put_message(buffer, 0x03, 15));
		}
	}
}

void sendPIDValuesToPC(uint8_t PIDIdentifier, float kp, float ki, float kd)
{
	uint8_t buffer[16] = {0};
	
	buffer[0] = PIDIdentifier;
	buffer[1] = 'P';
	SerialCOM_serializeFloat(&kp,&buffer[2]);

	buffer[6] = 'I';
	SerialCOM_serializeFloat(&ki,&buffer[7]);
	
	buffer[11] = 'D';
	SerialCOM_serializeFloat(&kd,&buffer[12]);
	
	ErrorHandling_throw(SerialCOM_put_message(buffer, 0x04, 16));
}

void message_from_PC(uint8_t* message, uint8_t Type)
{
	//Receive:
	//0x01: Motor
	//0x02: RC
	//0x03: Sensor
	//0x04: Commands:
	//				  - S: Set current values as Sensor offset
	//				  - R: Reset Arduino
	//0x05: Send PID Values to PC
	//0x06: Get Value from PC
	//0x07: Save Values to Flash

	//Send:
	//0x00: Debug message
	//0x01: Motor Values
	//0x02: RC Values
	//0x03: Sensor Values
	//0x04: PID Values
	//0x05: Reset GUI
	//0x06: ACK
	//0x07: calib status
	//0x64: Error Message
	
	//error_handler_in(SerialCOM_put_debug("GR"));
	float kp, ki, kd;
	uint32_t Value;
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
			switch(message[0])
			{
				case 'S':
					//sensorOffsetRoll = LastSensorMeasurement.Roll;
					//sensorOffsetPitch = LastSensorMeasurement.Pitch;
					//sensorOffsetYaw = LastSensorMeasurement.Yaw;
					
					PID_Reset(&YPid);
					PID_Reset(&XPid);
					PID_Reset(&ZPid);
				break;
				case 'R':
					if(ArmMotors == false)
						GPT_TimerSetEnabled(Wdt_resetTimer, false);
					else
						ErrorHandling_throw(SerialCOM_put_error("Motors have to be disarmt before reset!"));
				break;
				default:
					ErrorHandling_throw(SerialCOM_put_error("SetSensorOffsets command has errors"));
				break;
			}
		break;
		case 0x05:
			if(message[0] == 'R')
			{
				sendPIDValuesToPC('R', PID_YKp, PID_YKi, PID_YKd);
			} else if(message[0] == 'P')
			{
				sendPIDValuesToPC('P', PID_XKp, PID_XKi, PID_XKd);
			} else if(message[0] == 'Y')
			{
				sendPIDValuesToPC('Y', PID_ZKp, PID_ZKi, PID_ZKd);
			}
			else
			{
				ErrorHandling_throw(SerialCOM_put_error("SendPIDValues command has errors"));
			}
		break;
		case 0x06:
			if(ArmMotors == true)
			{
				ErrorHandling_throw(SerialCOM_put_error("Disarm Motors before changing values"));
				return;
			}
			//check message integrity:
			if(message[1] == 'P' && message[6] == 'I' && message[11] == 'D')
			{
				Value  = ((message[2] << 24) | (message[3] << 16) | (message[4] << 8) | message[5]);
				memcpy(&kp, &Value, 4);
				Value  = ((message[7] << 24) | (message[8] << 16) | (message[9] << 8) | message[10]);
				memcpy(&ki, &Value, 4);
				Value  = ((message[12] << 24) | (message[13] << 16) | (message[14] << 8) | message[15]);
				memcpy(&kd, &Value, 4);
				
				bno_contionous_measurement_active = false;
				while(BNO055_IsReady() == false);		//wait for a bit to make sure that all measurements are finished
				switch(message[0])
				{
					case 'P':
						PID_XKd = kd;
						PID_XKi = ki;
						PID_XKp = kp;
						PID_SetTunings(&XPid, PID_XKd, PID_XKi, PID_XKp);
						SaveValuesToFlash(0x04);
						SaveValuesToFlash(0x05);
						SaveValuesToFlash(0x06);
					break;
					case 'R':
						PID_YKd = kd;
						PID_YKi = ki;
						PID_YKp = kp;
						PID_SetTunings(&YPid, PID_YKd, PID_YKi, PID_YKp);
						SaveValuesToFlash(0x01);
						SaveValuesToFlash(0x02);
						SaveValuesToFlash(0x03);
					break;
					case 'Y':
						PID_ZKd = kd;
						PID_ZKi = ki;
						PID_ZKp = kp;
						PID_SetTunings(&ZPid, PID_ZKd, PID_ZKi, PID_ZKp);
						SaveValuesToFlash(0x07);
						SaveValuesToFlash(0x08);
						SaveValuesToFlash(0x09);
					break;
					default:
						ErrorHandling_throw(SerialCOM_put_error("SetPIDValues command has a wrong PID identifier"));
					break;
				}
				bno_contionous_measurement_active = true;
				ErrorHandling_throw(BNO_MEASURE);				//resume Measurement
			} 
			else
			{
				ErrorHandling_throw(SerialCOM_put_error("SetPIDValues command has errors"));
			}
		break;
		case 0x07:
			if(message[0] == 'S')
			{
				bno_contionous_measurement_active = false;
				while(BNO055_IsReady() == false);		//wait for a bit to make sure that all measurements are finished
				SaveValuesToFlash(0x00);				//save Values
				bno_contionous_measurement_active = true;
				ErrorHandling_throw(BNO_MEASURE);				//resume Measurement
				ErrorHandling_throw(SerialCOM_put_debug("Saved values to flash"));
			}else
			{
				ErrorHandling_throw(SerialCOM_put_error("SaveToFlash command has errors"));
			}
		break;
		default:
			ErrorHandling_throw(SerialCOM_put_error("Unknown command!"));
		break;
	}
}

void Init()
{
	SystemInit();

	GPT_Init();//Init the gpt timer for using system time functions and delays
	Wdt_resetTimer = GPT_TimerSetup(300, WDT_restart, true);	//call wdt reset every 300 ms
	
	//rc control and esc init:
	esc_init();
	rc_init();
	//Setup SerialCOM:
	ErrorHandling_throw(SerialCOM_init());
	ErrorHandling_throw(SerialCOM_register_call_back(message_from_PC));
	ErrorHandling_throw(SerialCOM_put_Command('R', 0x05));			//Reset GUI
	
	ErrorHandling_throw(SerialCOM_put_debug("MCU RESET!"));
	
	//Flash Storage Init:
	ErrorHandling_throw(FlashStorage_init());
	if(FlashStorage_read(0))
	{
		ErrorHandling_throw(SerialCOM_print_debug("Running for the first time, populating Flash with default values"));
		SaveValuesToFlash(0x00);
		ErrorHandling_throw(FlashStorage_write_uint8_t(0,0));
	}else
	{
		ErrorHandling_throw(SerialCOM_print_debug("loading values from flash"));
		LoadValuesFromFlash();
	}
	GPT_Delay(1000); //Wait a bit for the BNO to start up
	WDT_init(660); //about 1s
	
	//BNO Init:
	ErrorCode BNOErrors = ERROR_GENERIC;
	ErrorHandling_throw(SerialCOM_put_debug("Start BNO Init:"));
	ErrorHandling_throw(BNO055_register_error_callback(BNO_Error));
	ErrorHandling_throw(BNO055_register_data_ready_callback(DataReady));
	BNOErrors = BNO055_init(Do_not_calibrate);
	ErrorHandling_throw(BNOErrors);
	ErrorHandling_print();	//Print out all occured errors
	if(BNOErrors == SUCCESS)
		ErrorHandling_throw(SerialCOM_put_debug("BNO Init Succsessful"));
	else
		ErrorHandling_throw(SerialCOM_put_debug("BNO Init has errors, starting without IMU"));
	
	//Config PID controllers:
	PID_Init();
	ErrorHandling_throw(PID_Initialize(&XPid, &PID_XInput, &PID_XOutput, &PID_XSetPoint, PID_XKp, PID_XKi, PID_XKd, -1000, 1000, 4));
	ErrorHandling_throw(PID_Initialize(&YPid, &PID_YInput, &PID_YOutput, &PID_YSetPoint, PID_YKp, PID_YKi, PID_YKd, -1000, 1000, 4));
	ErrorHandling_throw(PID_Initialize(&ZPid, &PID_ZInput, &PID_ZOutput, &PID_ZSetPoint, PID_ZKp, PID_ZKi, PID_ZKd, -1000, 1000, 4));
	ErrorHandling_throw(PID_Initialize(&WPid, &PID_WInput, &PID_WOutput, &PID_WSetPoint, PID_WKp, PID_WKi, PID_WKd, -1000, 1000, 4));
	
	bno_contionous_measurement_active = true;
	if(BNOErrors == SUCCESS)	//Start the first BNO055 measurement if init was sucessful
		ErrorHandling_throw(BNO_MEASURE);
	ErrorHandling_print();	//Print out all occured errors
	ErrorHandling_throw(SerialCOM_put_debug("Init Done!"));
}


//calculate PID Controllers and positions, also send serial data
void Dont()
{
	
	//BNO data calculation:
	
	//calculate sensor offsets:
	CorrectedValues.x	= LastSensorMeasurement.x;
	CorrectedValues.y	= LastSensorMeasurement.y;
	CorrectedValues.z	= LastSensorMeasurement.z;
	CorrectedValues.w	= LastSensorMeasurement.w;
	
	PID_XInput			= CorrectedValues.x;
	PID_YInput			= CorrectedValues.y;
	PID_ZInput			= CorrectedValues.z;
	PID_WInput			= CorrectedValues.w;
	
	//for now there is no remote control...
	PID_XSetPoint		= 0;
	PID_YSetPoint		= 0.7071067812;
	PID_ZSetPoint		= 0;
	PID_WSetPoint		= 0.7071067812;
	
	static bool GearStateOld = false;
	RemoteValues = rc_read_values();
	if(GearStateOld == false && RemoteValues.Gear == true && RemoteValues.Throttle == 0)
	{
		GearStateOld = true;
		ArmMotors = true;
		//reset PID Controllers
		PID_Reset(&XPid);
		PID_Reset(&YPid);
		PID_Reset(&ZPid);
		PID_Reset(&WPid);
	} else if(RemoteValues.Gear == false)
	{
		ArmMotors = false;
		GearStateOld = false;
	}
	// Compute new PID output value
	ErrorHandling_throw(PID_Compute(&XPid));
	ErrorHandling_throw(PID_Compute(&YPid));
	ErrorHandling_throw(PID_Compute(&ZPid));
	ErrorHandling_throw(PID_Compute(&WPid));
	//adjust output values to the current thrust:
	BNO055_Quat Adjustments;
	Adjustments.x = map_float(PID_XOutput,-1000,1000,-1, 1);
	Adjustments.y = map_float(PID_YOutput,-1000,1000,-1, 1);
	Adjustments.z = map_float(PID_ZOutput,-1000,1000,-1, 1);
	Adjustments.w = map_float(PID_WOutput,-1000,1000,-1, 1);
	
	YPR_Angles = ConvertQuaToYPR(Adjustments);

	//calculate motor speeds:
	Motor_speeds[0] = RemoteValues.Throttle - YPR_Angles.Pitch - YPR_Angles.Roll;// - MappedYaw;
	Motor_speeds[1] = RemoteValues.Throttle - YPR_Angles.Pitch + YPR_Angles.Roll;// + MappedYaw;
	Motor_speeds[2] = RemoteValues.Throttle + YPR_Angles.Pitch - YPR_Angles.Roll;// - MappedYaw;
	Motor_speeds[3] = RemoteValues.Throttle + YPR_Angles.Pitch + YPR_Angles.Roll;// + MappedYaw;
	
	//-----Data Logging:
	SendLogData();
	
	//Set motor speeds:
	if(ArmMotors == true && RemoteValues.error != true)
	{
		/*
		ErrorHandling_throw(esc_set(1, Motor_speeds[0]));
		ErrorHandling_throw(esc_set(2, Motor_speeds[1]));
		ErrorHandling_throw(esc_set(3, Motor_speeds[2]));
		ErrorHandling_throw(esc_set(4, Motor_speeds[3]));
		*/
		ErrorHandling_throw(esc_set(1, 0));
		ErrorHandling_throw(esc_set(2, 0));
		ErrorHandling_throw(esc_set(3, 0));
		ErrorHandling_throw(esc_set(4, 0));
	}
	else
	{
		ErrorHandling_throw(esc_set(1, 0));
		ErrorHandling_throw(esc_set(2, 0));
		ErrorHandling_throw(esc_set(3, 0));
		ErrorHandling_throw(esc_set(4, 0));
	}
}

bool aboutToCrash()
{
	bool needComputeX = PID_need_compute(&XPid);
	bool needComputeY = PID_need_compute(&YPid);
	bool needComputeZ = PID_need_compute(&ZPid);
	bool needComputeW = PID_need_compute(&WPid);
	return needComputeY == true || needComputeX == true || needComputeZ == true || needComputeW == true;
}

int main(void)
{
	Init();
	while(1)
	{
		ErrorHandling_print();		//print out errors when there is time for it
		if(aboutToCrash())
		{
			Dont();
		}
	}
}