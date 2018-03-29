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
#include "BNO_055.h"
#include "ESCControl.h"
#include "RCReader.h"
#include "PID.h"
#include "SerialCOM.h"
#include "FlashStorage.h"
#include "WDT.h"

#define BNO_MEASURE BNO055_read(BNO_REG_QUA_DATA_W,8)

//Init variables for the drone programm
BNO055_Data SensorValues;
RemoteControlValues RemoteValues;
int16_t Motor_speeds[4] = {0};
bool bno_contionous_measurement_active = true;


//PID Config:
float PID_PitchInput = 0,	PID_PitchOutput = 0,	PID_PitchSetPoint = 0;
float PID_RollInput = 0,	PID_RollOutput = 0,		PID_RollSetPoint = 0;
float PID_YawInput = 0,		PID_YawOutput = 0,		PID_YawSetPoint = 0;
float PitchKp = 0.1,  PitchKi = 0.05,    PitchKd = 0.05;
float RollKp = 0.1,  RollKi = 0.05,    RollKd = 0.05;
float YawKp = 0.1,    YawKi = 0.1,    YawKd = 0.1;
pidData PitchPid;
pidData RollPid;
pidData YawPid;

//sonsor offsetValues:
float sensorOffsetPitch = 0;
float sensorOffsetRoll = 0;
float sensorOffsetYaw  = 0;

//Status variables for whitch values are printing:
bool printSensorValues = 0;
bool printRCValues = 0;
bool printMotorValues = 0;
uint8_t sendCount = 0;

bool ArmMotors = false;


void serializeFloat(float* Value, uint8_t* startptr);


void BNO_Error(ErrorCode Error)
{
	if((Error & 0xFF) == ERROR_BUS_OVER_RUN)		//BNO was not ready for operation, try again
	{
		ErrorHandling_throw(BNO_MEASURE);
	}
	else																							//Some kind of other error
	{
		ErrorHandling_throw(ErrorHandling_set_top_level(Error, MODULE_MAIN, FUNCTION_error));				//throw the error
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

void DataReady(uint8_t Data[], uint8_t Length)
{
	SensorValues = ConvertQuaToYPR(Data);
	
	//BNO data calc:

	BNO055_Data CorrectedValues;
	CorrectedValues.Roll	= SensorValues.Roll - sensorOffsetRoll;
	CorrectedValues.Pitch	= SensorValues.Pitch - sensorOffsetPitch;
	CorrectedValues.Yaw		= SensorValues.Yaw - sensorOffsetYaw;
	
	PID_PitchInput		= CorrectedValues.Roll;
	PID_RollInput		= CorrectedValues.Pitch;
	PID_YawInput		= CorrectedValues.Yaw;
	
	//a full range of stick movement represents a +- 10 degree tilt
	PID_PitchSetPoint	= map(RemoteValues.Pitch, 0, 2200, -10, 10);
	PID_RollSetPoint	= map(RemoteValues.Roll, 0, 2200, -10, 10);
	
	//PID Loop calc:
	
	static bool GearStateOld = false;
	bool needComputePitch = PID_need_compute(&PitchPid);
	bool needComputeRoll = PID_need_compute(&RollPid);
	if(needComputeRoll == true || needComputePitch == true)
	{
		
		//read Values from sensor and remote control:
		//---SensorValues = BNO055_get_measurement_data();

		RemoteValues = rc_read_values();
		if(GearStateOld == false && RemoteValues.Gear == true && RemoteValues.Throttle == 0)
		{
			GearStateOld = true;
			ArmMotors = true;
		} else if(RemoteValues.Gear == false)
		{
			ArmMotors = false;
			GearStateOld = false;
		}
		
		// Compute new PID output value
		if (needComputePitch)
			ErrorHandling_throw(PID_Compute(&PitchPid));
		if (needComputeRoll)
			ErrorHandling_throw(PID_Compute(&RollPid));
		//float factor = 0.0005;
		//int16_t PitchAdjust = PID_PitchInput*2;//*(factor*RemoteValues.Throttle); 
		//int16_t RollAdjust = PID_RollInput*2;//*(factor*RemoteValues.Throttle);
			
		Motor_speeds[0] = RemoteValues.Throttle - PID_PitchOutput - PID_RollOutput;// - MappedYaw;
		Motor_speeds[1] = RemoteValues.Throttle - PID_PitchOutput + PID_RollOutput;// + MappedYaw;
		Motor_speeds[2] = RemoteValues.Throttle + PID_PitchOutput - PID_RollOutput;// - MappedYaw;
		Motor_speeds[3] = RemoteValues.Throttle + PID_PitchOutput + PID_RollOutput;// + MappedYaw;
		/*
		Motor_speeds[0] = RemoteValues.Throttle;
		Motor_speeds[1] = RemoteValues.Throttle;
		Motor_speeds[2] = RemoteValues.Throttle;
		Motor_speeds[3] = RemoteValues.Throttle;
		*/			
		if(Motor_speeds[0] < 0)
			Motor_speeds[0] = 0;
		if(Motor_speeds[1] < 0)
			Motor_speeds[1] = 0;
		if(Motor_speeds[2] < 0)
			Motor_speeds[2] = 0;
		if(Motor_speeds[3] < 0)
			Motor_speeds[3] = 0;
		
		if(RemoteValues.error != true)
		{
			if(ArmMotors == true)
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
	}
	
	//-----Data Logging:
	if(SerialCOM_get_free_space() >= printMotorValues + printRCValues + printSensorValues && sendCount++ >= 10)
	{
		sendCount = 0;
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
			uint8_t buffer[15] = {0};
			
			buffer[0] = 'R';
			serializeFloat(&CorrectedValues.Roll, &buffer[1]);
			
			buffer[5] = 'P';
			serializeFloat(&CorrectedValues.Pitch, &buffer[6]);
			
			buffer[10] = 'Y';
			serializeFloat(&CorrectedValues.Yaw, &buffer[11]);
			
			ErrorHandling_throw(SerialCOM_put_message(buffer, 0x03, 15));
		}
	}
	if(bno_contionous_measurement_active)
		ErrorHandling_throw(BNO_MEASURE);
}

void serializeFloat(float* Value, uint8_t* startptr)
{
	uint32_t NumValue;
	memcpy(&NumValue, Value, 4);
	startptr[0] = (NumValue & 0xFF000000) >> 24;
	startptr[1] = (NumValue & 0x00FF0000) >> 16;
	startptr[2] = (NumValue & 0x0000FF00) >> 8;
	startptr[3]=  NumValue & 0x000000FF;
}

void sendPIDValuesToPC(uint8_t PIDIdentifier, float kp, float ki, float kd)
{
	uint8_t buffer[16] = {0};
	
	buffer[0] = PIDIdentifier;
	buffer[1] = 'P';
	serializeFloat(&kp,&buffer[2]);

	buffer[6] = 'I';
	serializeFloat(&ki,&buffer[7]);
	
	buffer[11] = 'D';
	serializeFloat(&kd,&buffer[12]);
	
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
					sensorOffsetRoll = SensorValues.Roll;
					sensorOffsetPitch = SensorValues.Pitch;
					sensorOffsetYaw = SensorValues.Yaw;
					//SaveValuesToFlash(0x0A);
					//SaveValuesToFlash(0x0B);
					PID_Reset(&RollPid);
					PID_Reset(&PitchPid);
					PID_Reset(&YawPid);
				break;
				case 'R':
					if(RemoteValues.Gear == false)
						while(1);
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
			//check message integrity:
			if(message[1] == 'P' && message[6] == 'I' && message[11] == 'D')
			{
				Value  = ((message[2] << 24) | (message[3] << 16) | (message[4] << 8) | message[5]);
				memcpy(&kp, &Value, 4);
				Value  = ((message[7] << 24) | (message[8] << 16) | (message[9] << 8) | message[10]);
				memcpy(&ki, &Value, 4);
				Value  = ((message[12] << 24) | (message[13] << 16) | (message[14] << 8) | message[15]);
				memcpy(&kd, &Value, 4);
				
				switch(message[0])
				{
					case 'P':
						PitchKd = kd;
						PitchKi = ki;
						PitchKp = kp;
						PID_SetTunings(&PitchPid, PitchKd, PitchKi, PitchKp);
					break;
					case 'R':
						RollKd = kd;
						RollKi = ki;
						RollKp = kp;
						PID_SetTunings(&RollPid, RollKd, RollKi, RollKp);
					break;
					case 'Y':
						YawKd = kd;
						YawKi = ki;
						YawKp = kp;
						PID_SetTunings(&YawPid, YawKd, YawKi, YawKp);
					break;
					default:
						ErrorHandling_throw(SerialCOM_put_error("SetPIDValues command has a wrong PID identifier"));
					break;
				}
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



void config_BNO()
{

	ErrorHandling_throw(SerialCOM_put_debug("Start BNO Init"));
	ErrorHandling_throw(BNO055_init(Do_not_calibrate));
	ErrorHandling_throw(SerialCOM_put_debug("Calib OK"));
	ErrorHandling_throw(BNO055_register_error_callback(BNO_Error));
	ErrorHandling_throw(BNO055_register_data_ready_callback(DataReady));

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
	
	ErrorHandling_throw(SerialCOM_init());
	ErrorHandling_throw(SerialCOM_register_call_back(message_from_PC));
	ErrorHandling_throw(SerialCOM_put_Command('R', 0x05));			//Reset GUI
	ErrorHandling_throw(FlashStorage_init());
	
	ErrorHandling_throw(SerialCOM_put_debug("MCU RESET!"));
	if(FlashStorage_read(0))
	{
		ErrorHandling_throw(SerialCOM_print_debug("Running for the first time, populating Flash with default values"));
		SaveValuesToFlash(0x00);
		//LoadValuesFromFlash();
		ErrorHandling_throw(FlashStorage_write_uint8_t(0,0));
	}else
	{
		ErrorHandling_throw(SerialCOM_print_debug("loading values from flash"));		
		LoadValuesFromFlash();
	}
	_Delay(8400000);
	config_BNO();
	
	//rc control and esc init:
	rc_init();
	esc_init();
	
	PID_Init();
	ErrorHandling_throw(PID_Initialize(&PitchPid, &PID_PitchInput, &PID_PitchOutput, &PID_PitchSetPoint, PitchKp, PitchKi, PitchKd, -250, 250, 10));
	ErrorHandling_throw(PID_Initialize(&RollPid, &PID_RollInput, &PID_RollOutput, &PID_RollSetPoint, RollKp, RollKi, RollKd, -250, 250, 10));
	ErrorHandling_throw(BNO_MEASURE);
	ErrorHandling_print();
	ErrorHandling_throw(SerialCOM_put_debug("Init Done!"));
	WDT_init(330); //about 1s
	while(1)
	{
		WDT_restart();
		ErrorHandling_print();		//print out errors when there is time for it
	}
}