/*
 * BNO055.c
 *
 * Created: 23.11.2017 21:44:11
 *  Author: flola
 */ 

#include "BNO055.h"


/*Predefined bno constants*/
#define BNO055_ID			0xA0
#define BNO_CONFIG_MODE		0x00
#define BNO_PWR_MODE_NORMAL	0x00
#define BNO_PAGE_ID0		0x00
#define BNO_INTERNAL_OSC	0x00
#define FUSION_MODE_NDOF	0x0C


StatusCode bno_register_read_1byte(uint8_t Register);
StatusCode bno_register_read_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage);
StatusCode bno_register_write_1byte(uint8_t Register, uint8_t Data);
StatusCode bno_register_write_1byte_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Data);
/***************************************************************************************************
* BNO Top Level interaction:
****************************************************************************************************/

void bno_runtime_error(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code);
void bno_runtime_success(uint8_t* Data, uint8_t Length);

uint8_t bno_init_response_value[2] = {0,0};
uint8_t bno_init_response_length = 0;
bool bno_init_waiting_for_response = false;
StatusCode bno_init_response_status = SUCCESS;

void bno_init_success(uint8_t* Data, uint8_t Length)
{
	bno_init_waiting_for_response = false;
	bno_init_response_length = Length;
	if(Data == NULL || Length == 0)
	{
		bno_init_response_value[0] = 0x00;
		bno_init_response_value[1] = 0x00;
	}else
	{
		bno_init_response_value[0] = Data[0];
		bno_init_response_value[1] = Data[1];
	}
	bno_init_response_status = SUCCESS;
}

void bno_init_error(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code)
{
	bno_init_waiting_for_response = false;
	bno_init_response_status = BNO055_ERROR_INVALID_ARGUMENT;
}

StatusCode BNO055_Setup(bool calibrationNeeded)
{
	StatusCode error_return;
	error_return = BNOCOM_Init(bno_init_success);
	if(error_return != SUCCESS)
		return error_return;
	error_return = BNOCOM_register_error_callback(bno_init_error);
	if(error_return != SUCCESS)
		return error_return;

	//Wait for USART0 to clear its backlog if there is any
	while(!USART0_is_idle());
	//Check for right device
	//Read Chip-id 
	
	bno_init_waiting_for_response = true;
	error_return = bno_register_read_by_table(BNO_REG_CHIP_ID, 0);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_init_waiting_for_response);	//wait for transmission response
	if(bno_init_response_value[0] != BNO055_ID && bno_init_response_length == 1)
		return BNO055_ERROR_WRONG_DEVICE_ID;
	/*//sensor defaults to OPR_MODE -> config mode
	bno_init_waiting_for_response = true;
	error_return = bno_register_write_1byte_by_table(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_init_waiting_for_response);	//wait for transmission response
	if(bno_init_response_status != SUCCESS)
		return bno_init_response_status;
	*/

	/*//sensor defaults to PWR_MODE -> normal mode
	bno_init_waiting_for_response = true;
	error_return = bno_register_write_1byte_by_table(BNO_REG_PWR_MODE, 0, BNO_PWR_MODE_NORMAL);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_init_waiting_for_response);	//wait for transmission response
	if(bno_init_response_status != SUCCESS)
		return bno_init_response_status;
	*/

	/*//sensor defaults to PAGE_ID -> PAGE0
	bno_init_waiting_for_response = true;
	error_return = bno_register_write_1byte_by_table(BNO_REG_PAGE_ID, 0, BNO_PAGE_ID0);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_init_waiting_for_response);	//wait for transmission response
	if(bno_init_response_status != SUCCESS)
		return bno_init_response_status;
	*/
	//Set output units:
	uint8_t data;
	data =	(0<<7) | //Orientation = Windows
			(0<<4) | //Temperature = Celsius
			(0<<2) | //Euler = Degrees
			(1<<1) | //Gyro = Rads
			(0<<0);  //Accelerometer = m/s^2
	bno_init_waiting_for_response = true;
	error_return = bno_register_write_1byte_by_table(BNO_REG_UNIT_SEL, 0, data);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_init_waiting_for_response);	//wait for transmission response
	if(bno_init_response_status != SUCCESS)
		return bno_init_response_status;

	/*//sensor defaults to SYS_TRIGGER -> Internal oscillator
	bno_init_waiting_for_response = true;
	error_return = bno_register_write_1byte_by_table(BNO_REG_SYS_TRIGGER, 0, BNO_INTERNAL_OSC);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_init_waiting_for_response);	//wait for transmission response
	if(bno_init_response_status != SUCCESS)
		return bno_init_response_status;
	*/
	if(calibrationNeeded)
	{
		//Switch to Fusion Mode
		bno_init_waiting_for_response = true;
		error_return = bno_register_write_1byte_by_table(BNO_REG_OPR_MODE, 0, FUSION_MODE_NDOF);
		if(error_return != SUCCESS)
			return error_return;
		while(bno_init_waiting_for_response);	//wait for transmission response
		if(bno_init_response_status != SUCCESS)
			return bno_init_response_status;
		
		//Calibrating
		uint8_t sys = 0;
		uint8_t gyro = 0;
		uint8_t accel = 0;
		uint8_t mag = 0;
		while(sys != 3 || gyro != 3 || accel != 3 || mag != 3)
		{
			bno_init_waiting_for_response = true;
			error_return = bno_register_read_1byte(BNO_REG_CALIB_STAT);
			if(error_return != SUCCESS)
				return error_return;
			while(bno_init_waiting_for_response);	//wait for transmission response
			if(bno_init_response_status != SUCCESS)
				return bno_init_response_status;
			DEFUALT_ERROR_HANDLER(BNO055_calculate_calibration(bno_init_response_value[0],&sys, &gyro, &accel, &mag),calib_return);
		}




		//Switch to Config Mode
		bno_init_waiting_for_response = true;
		error_return = bno_register_write_1byte_by_table(BNO_REG_OPR_MODE, 0, BNO_CONFIG_MODE);
		if(error_return != SUCCESS)
			return error_return;
		while(bno_init_waiting_for_response);	//wait for transmission response
		if(bno_init_response_status != SUCCESS)
			return bno_init_response_status;
		
		/*//read Calibration data
		IIC_RegisterReadStart(BNO055_LOW_ADDRESS, ACC_OFFSET_X_LSB, 22, calibrationData);
		while(!IIC_busFree());
		
		//write Data to EEPROM
		for(int i=0; i<=(CALIB_DATA_END-CALIB_DATA_START); i++)
			EEPROM_write(CALIB_DATA_START+i, calibrationData[i]);*/
			
		
	}else{
		/*//read calibration data from EEPROM
		for(int i=0; i<=(CALIB_DATA_END-CALIB_DATA_START); i++)
		{
			calibrationData[i] = EEPROM_read(CALIB_DATA_START+i);
		}
			
		#ifdef DEBUG_BNO055
			for(int i=0; i<22; i++)
			{
				uart0_putChar(calibrationData[i]);
				uart0_putc('\t');
			}
			uart0_newline();
		#endif
	
		//switch to config mode
		BNO055_setMode(CONFIG_MODE);
		
		//write calibration data
		IIC_RegisterWriteStart(BNO055_LOW_ADDRESS, ACC_OFFSET_X_LSB, 22, calibrationData);
		while(!IIC_busFree());
		*/
	}
	
	//Set Operation Mode to NDOF (nine degrees of freedom)
	bno_init_waiting_for_response = true;
	error_return = bno_register_write_1byte_by_table(BNO_REG_OPR_MODE, 0, FUSION_MODE_NDOF);
	if(error_return != SUCCESS)
		return error_return;
	while(bno_init_waiting_for_response);	//wait for transmission response
	if(bno_init_response_status != SUCCESS)
		return bno_init_response_status;
	
	//Initialization finished
	DEFUALT_ERROR_HANDLER(BNOCOM_register_success_callback(bno_runtime_success), error_returnX);
	DEFUALT_ERROR_HANDLER(BNOCOM_register_error_callback(bno_runtime_error), error_returnY);
	return SUCCESS;
}

StatusCode BNO055_calculate_calibration(uint8_t CalibrationData, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
	if(sys == NULL || gyro == NULL || accel == NULL || mag == NULL)
		return BNO055_ERROR_GOT_NULL_POINTER;
	//Write the data to the proper variables
	*sys = (CalibrationData >> 6) & 0x03;
	*gyro = (CalibrationData >> 4) & 0x03;
	*accel = (CalibrationData >> 2) & 0x03;
	*mag = CalibrationData & 0x03;
	return SUCCESS;
}

void bno_runtime_success(uint8_t* Data, uint8_t Length)
{
	
}

void bno_runtime_error(BNO_STATUS_BYTES Error, StatusCode Transmit_error_code)
{
	
}

StatusCode BNO055_get_euler_data(bool measureContinous)
{
	return SUCCESS;
}



/***************************************************************************************************
* BNO Low Level stuff:
****************************************************************************************************/

StatusCode bno_register_write_1byte_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage, uint8_t Data)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
	return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	
	if(RegisterTablePage == 0)
	{
		if(BNO055_reg_table0[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY || BNO055_reg_table0[BNO_REG_LEN][RegisterTableOffset] == 2)
		return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_write(BNO055_reg_table0[BNO_REG][RegisterTableOffset], 1, &Data);
	}
	else
	{
		if(BNO055_reg_table1[BNO_REG][RegisterTableOffset] == BNO_REG_READ_ONLY || BNO055_reg_table1[BNO_REG_LEN][RegisterTableOffset] == 2)
		return BNO055_ERROR_INVALID_ARGUMENT;
		return BNOCOM_register_write(BNO055_reg_table1[BNO_REG][RegisterTableOffset], 1, &Data);
	}
}

StatusCode bno_register_write_1byte(uint8_t Register, uint8_t Data)
{
	return BNOCOM_register_write(Register,1,&Data);
}

StatusCode bno_register_read_by_table(uint8_t RegisterTableOffset, uint8_t RegisterTablePage)
{
	if(RegisterTableOffset > BNO_NUM_REG_ADDRESSES0 || RegisterTablePage > 1)
		return BNO055_ERROR_ARGUMENT_OUT_OF_RANGE;
	if(RegisterTablePage == 0)
		return BNOCOM_register_read(BNO055_reg_table0[BNO_REG][RegisterTableOffset], BNO055_reg_table0[BNO_REG_LEN][RegisterTableOffset]);
	else
		return BNOCOM_register_read(BNO055_reg_table1[BNO_REG][RegisterTableOffset], BNO055_reg_table1[BNO_REG_LEN][RegisterTableOffset]);
}

StatusCode bno_register_read_1byte(uint8_t Register)
{
	return BNOCOM_register_read(Register,1);
}