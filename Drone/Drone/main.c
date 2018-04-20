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
#include "Modules/BNO055/BNO_055.h"
#include "Modules/ESCControl/ESCControl.h"
#include "Modules/RCReader/RCReader.h"
#include "Modules/PID/PID.h"
#include "Modules/SerialCOM/SerialCOM.h"
#include "Modules/FlashStorage/FlashStorage.h"
#include "Modules/WDT/WDT.h"
#include "Modules/GPT/GPT.h"

#define BNO_MEASURE BNO055_read(BNO_REG_QUA_DATA_W, 14)
#define SERIALCOM_SEND_INTERVALL_MS 20

//Init variables for the drone programm
BNO055_Data LastSensorMeasurement;
BNO055_Data CorrectedValues;
RemoteControlValues RemoteValues;
float PosX = 0, PosY = 0, PosZ = 0;
int16_t Motor_speeds[4] = {0};
bool bno_contionous_measurement_active = true;
bool ArmMotors = false;
float timeOfLastMeasurement = 0;
Timer Wdt_resetTimer;


//PID Config:
float PID_PitchInput = 0,	PID_PitchOutput = 0,	PID_PitchSetPoint = 0;
float PID_RollInput = 0,	PID_RollOutput = 0,		PID_RollSetPoint = 0;
float PID_YawInput = 0,		PID_YawOutput = 0,		PID_YawSetPoint = 0;
float PitchKp = 1.3,  PitchKi = 0.2,    PitchKd = 1.2;
float RollKp = 0.8,  RollKi = 0.1,    RollKd = 0.7;
float YawKp = 0,    YawKi = 0,    YawKd = 0;
pidData PitchPid;
pidData RollPid;
pidData YawPid;

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
		ErrorHandling_throw(ErrorHandling_set_top_level(Error, MODULE_MAIN, FUNCTION_error));				//throw the error
		ErrorHandling_throw(BNO_MEASURE);																//restart measurement
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
		ErrorHandling_throw(FlashStorage_write_float(4, RollKp));
	if(type == 0x00 || type == 0x02)
		ErrorHandling_throw(FlashStorage_write_float(8, RollKi));
	if(type == 0x00 || type == 0x03)
		ErrorHandling_throw(FlashStorage_write_float(12, RollKd));
	if(type == 0x00 || type == 0x04)
		ErrorHandling_throw(FlashStorage_write_float(16, PitchKp));
	if(type == 0x00 || type == 0x05)
		ErrorHandling_throw(FlashStorage_write_float(20, PitchKi));
	if(type == 0x00 || type == 0x06)
		ErrorHandling_throw(FlashStorage_write_float(24, PitchKd));
	if(type == 0x00 || type == 0x07)
		ErrorHandling_throw(FlashStorage_write_float(28, YawKp));
	if(type == 0x00 || type == 0x08)
		ErrorHandling_throw(FlashStorage_write_float(32, YawKi));
	if(type == 0x00 || type == 0x09)
		ErrorHandling_throw(FlashStorage_write_float(36, YawKd));
	if(type == 0x00 || type == 0x0A)
		ErrorHandling_throw(FlashStorage_write_float(40, sensorOffsetRoll));
	if(type == 0x00 || type == 0x0B)
		ErrorHandling_throw(FlashStorage_write_float(44, sensorOffsetPitch));
}

void LoadValuesFromFlash()
{
	RollKp = FlashStorage_read_float(4);
	RollKi = FlashStorage_read_float(8);
	RollKd = FlashStorage_read_float(12);
	PitchKp = FlashStorage_read_float(16);
	PitchKi = FlashStorage_read_float(20);
	PitchKd = FlashStorage_read_float(24);
	YawKp = FlashStorage_read_float(28);
	YawKi = FlashStorage_read_float(32);
	YawKd = FlashStorage_read_float(36);
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
		LastSensorMeasurement = ConvertQuaToYPR(Data);

		if(Length >= 14)
		{
			//unit: 1m/s^2
			LastSensorMeasurement.AccX = ((uint16_t)Data[8])  | (((uint16_t)Data[9] ) << 8);
			LastSensorMeasurement.AccY = ((uint16_t)Data[10]) | (((uint16_t)Data[11]) << 8);
			LastSensorMeasurement.AccZ = ((uint16_t)Data[12]) | (((uint16_t)Data[13]) << 8);
		}
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
			SerialCOM_serializeFloat(&CorrectedValues.Roll, &buffer[1]);
			
			buffer[5] = 'P';
			SerialCOM_serializeFloat(&CorrectedValues.Pitch, &buffer[6]);
			
			buffer[10] = 'Y';
			SerialCOM_serializeFloat(&CorrectedValues.Yaw, &buffer[11]);
			
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
void message_from_PC(uint8_t* message, uint8_t Type)
{
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
					sensorOffsetRoll = LastSensorMeasurement.Roll;
					sensorOffsetPitch = LastSensorMeasurement.Pitch;
					sensorOffsetYaw = LastSensorMeasurement.Yaw;
					
					PID_Reset(&RollPid);
					PID_Reset(&PitchPid);
					PID_Reset(&YawPid);
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
				sendPIDValuesToPC('R', RollKp, RollKi, RollKd);
			} else if(message[0] == 'P')
			{
				sendPIDValuesToPC('P', PitchKp, PitchKi, PitchKd);
			} else if(message[0] == 'Y')
			{
				sendPIDValuesToPC('Y', YawKp, YawKi, YawKd);
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
						PitchKd = kd;
						PitchKi = ki;
						PitchKp = kp;
						PID_SetTunings(&PitchPid, PitchKd, PitchKi, PitchKp);
						SaveValuesToFlash(0x04);
						SaveValuesToFlash(0x05);
						SaveValuesToFlash(0x06);
					break;
					case 'R':
						RollKd = kd;
						RollKi = ki;
						RollKp = kp;
						PID_SetTunings(&RollPid, RollKd, RollKi, RollKp);
						SaveValuesToFlash(0x01);
						SaveValuesToFlash(0x02);
						SaveValuesToFlash(0x03);
					break;
					case 'Y':
						YawKd = kd;
						YawKi = ki;
						YawKp = kp;
						PID_SetTunings(&YawPid, YawKd, YawKi, YawKp);
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
	rc_init();
	esc_init();
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
	ErrorHandling_throw(PID_Initialize(&PitchPid, &PID_PitchInput, &PID_PitchOutput, &PID_PitchSetPoint, PitchKp, PitchKi, PitchKd, -1000, 1000, 4));
	ErrorHandling_throw(PID_Initialize(&RollPid, &PID_RollInput, &PID_RollOutput, &PID_RollSetPoint, RollKp, RollKi, RollKd, -1000, 1000, 4));
	//ErrorHandling_throw(PID_Initialize(&YawPid, &PID_YawInput, &PID_YawOutput, &PID_YawSetPoint, YawKp, YawKi, YawKd, -360, 360, 4));
	
	bno_contionous_measurement_active = true;
	if(BNOErrors == SUCCESS)	//Start the first BNO055 measurement if init was sucessful
		ErrorHandling_throw(BNO_MEASURE);
	ErrorHandling_print();	//Print out all occured errors
	ErrorHandling_throw(SerialCOM_put_debug("Init Done!"));
}



//calculate PID Controllers and positions, also send serial data
void Dont()
{
	//Relative Position calculation:
	/* - Not needed for now
	float LinAccX = ((float)LastSensorMeasurement.AccX);
	float LinAccY = ((float)LastSensorMeasurement.AccY);
	
	float timePassed = (GPT_GetPreciseTime() - timeOfLastMeasurement) / 1000.0;
	float timePassedPower2 = timePassed * timePassed;
	
	PosX += timePassedPower2*LinAccX;
	PosY += timePassedPower2*LinAccY;
	*/
	
	//BNO data calculation:
	
	CorrectedValues.Roll	= -CorrectSensorOffsets(LastSensorMeasurement.Roll, sensorOffsetRoll, 180);
	CorrectedValues.Pitch	= _CorrectSensorOffsets(LastSensorMeasurement.Pitch, sensorOffsetPitch, 90);
	CorrectedValues.Yaw		= CorrectSensorOffsets(LastSensorMeasurement.Yaw, sensorOffsetYaw, 180);
	
	PID_PitchInput			= CorrectedValues.Pitch;
	PID_RollInput			= CorrectedValues.Roll;
	PID_YawInput			= CorrectedValues.Yaw;
	
	//a full range of stick movement represents a +- 20 degree tilt
	PID_PitchSetPoint		= map_float(RemoteValues.Pitch, 0, 2200, -20, 20);
	PID_RollSetPoint		= map_float(RemoteValues.Roll, 0, 2200, -20, 20);
	PID_YawSetPoint			= 0;// TODO: Yaw Logic
	
	static bool GearStateOld = false;
	RemoteValues = rc_read_values();
	if(GearStateOld == false && RemoteValues.Gear == true && RemoteValues.Throttle == 0)
	{
		GearStateOld = true;
		ArmMotors = true;
		//reset PID Controllers
		PID_Reset(&PitchPid);
		PID_Reset(&RollPid);
		PID_Reset(&YawPid);
	} else if(RemoteValues.Gear == false)
	{
		ArmMotors = false;
		GearStateOld = false;
	}
	// Compute new PID output value
	ErrorHandling_throw(PID_Compute(&PitchPid));
	ErrorHandling_throw(PID_Compute(&RollPid));
	//ErrorHandling_throw(PID_Compute(&YawPid));
	//adjust output values to the current thrust:
	float PitchAdjust, RollAdjust;
	PitchAdjust = map_float(PID_PitchOutput,-1000,1000,-RemoteValues.Throttle, RemoteValues.Throttle);
	RollAdjust = map_float(PID_RollOutput,-1000,1000,-RemoteValues.Throttle, RemoteValues.Throttle);
	
	//calculate motor speeds:
	Motor_speeds[0] = RemoteValues.Throttle - PitchAdjust - RollAdjust;// - MappedYaw;
	Motor_speeds[1] = RemoteValues.Throttle - PitchAdjust + RollAdjust;// + MappedYaw;
	Motor_speeds[2] = RemoteValues.Throttle + PitchAdjust - RollAdjust;// - MappedYaw;
	Motor_speeds[3] = RemoteValues.Throttle + PitchAdjust + RollAdjust;// + MappedYaw;
	
	//-----Data Logging:
	SendLogData();
	
	//Set motor speeds:
	if(ArmMotors == true && RemoteValues.error != true)
	{
		ErrorHandling_throw(esc_set(1, Motor_speeds[0]));
		ErrorHandling_throw(esc_set(2, Motor_speeds[1]));
		ErrorHandling_throw(esc_set(3, Motor_speeds[2]));
		ErrorHandling_throw(esc_set(4, Motor_speeds[3]));
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
	bool needComputePitch = PID_need_compute(&PitchPid);
	bool needComputeRoll = PID_need_compute(&RollPid);
	//bool needComputeYaw = PID_need_compute(&YawPid);
	return needComputeRoll == true || needComputePitch == true; //|| needComputeYaw == true
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