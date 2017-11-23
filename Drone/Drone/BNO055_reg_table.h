/*
 * BNO055_reg_table.h
 *
 * Created: 20.11.2017 18:44:01
 *  Author: flola
 */ 


#ifndef BNO055_REG_TABLE_H_
#define BNO055_REG_TABLE_H_


#define BNO_REG									0
#define BNO_REG_LEN								1
#define BNO_REG_RW_SATUS						2

#define BNO_REG_READ_ONLY						0
#define BNO_REG_WRITE_ONLY						1
#define BNO_REG_READ_AND_WRITE					2

//PAGE 0
#define BNO_REG_CHIP_ID							0//r
#define BNO_REG_ACC_ID							1//r
#define BNO_REG_MAG_ID							2//r
#define BNO_REG_GYR_ID							3//r
#define BNO_REG_SW_REV_ID						4//r
#define BNO_REG_BL_REV_ID						5//r
#define BNO_REG_PAGE_ID							6//rw
#define BNO_REG_ACC_DATA_X						7//r
#define BNO_REG_ACC_DATA_Y						8//r
#define BNO_REG_ACC_DATA_Z						9//r
#define BNO_REG_MAG_DATA_X						10//r
#define BNO_REG_MAG_DATA_Y						11//r
#define BNO_REG_MAG_DATA_Z						12//r
#define BNO_REG_GYR_DATA_X						13//r
#define BNO_REG_GYR_DATA_Y						14//r
#define BNO_REG_GYR_DATA_Z						15//r
#define BNO_REG_EUL_DATA_X						16//r
#define BNO_REG_EUL_DATA_Y						17//r
#define BNO_REG_EUL_DATA_Z						18//r
#define BNO_REG_QUA_DATA_W						19//r
#define BNO_REG_QUA_DATA_X						20//r
#define BNO_REG_QUA_DATA_Y						21//r
#define BNO_REG_QUA_DATA_Z						22//r
#define BNO_REG_LIA_DATA_X						23//r
#define BNO_REG_LIA_DATA_Y						24//r
#define BNO_REG_LIA_DATA_Z						25//r
#define BNO_REG_GRV_DATA_X						26//r
#define BNO_REG_GRV_DATA_Y						27//r
#define BNO_REG_GRV_DATA_Z						28//r
#define BNO_REG_TEMP							29//r
#define BNO_REG_CALIB_STAT						30//r
#define BNO_REG_ST_RESULT						31//r
#define BNO_REG_INT_STA							32//r
#define BNO_REG_SYS_CLK_STATUS					33//r
#define BNO_REG_SYS_STATUS						34//r
#define BNO_REG_SYS_ERR							35//r
#define BNO_REG_UNIT_SEL						36//r
#define BNO_REG_OPR_MODE						37//rw
#define BNO_REG_PWR_MODE						38//rw
#define BNO_REG_SYS_TRIGGER						39//w
#define BNO_REG_TEMP_SOURCE						40//rw
#define BNO_REG_AXIS_MAP_CONFIG					41//rw
#define BNO_REG_AXIS_MAP_SIGN					42//rw
#define BNO_REG_ACC_OFFSET_X					43//rw
#define BNO_REG_ACC_OFFSET_Y					44//rw
#define BNO_REG_ACC_OFFSET_Z					45//rw
#define BNO_REG_MAG_OFFSET_X					46//rw
#define BNO_REG_MAG_OFFSET_Y					47//rw
#define BNO_REG_MAG_OFFSET_Z					48//rw
#define BNO_REG_GYR_OFFSET_X					49//rw
#define BNO_REG_GYR_OFFSET_Y					50//rw
#define BNO_REG_GYR_OFFSET_Z					51//rw
#define BNO_REG_ACC_RADIUS						52//rw
#define BNO_REG_MAG_RADIUS						53//rw

#define BNO_NUM_REG_ADDRESSES0					BNO_REG_MAG_RADIUS+1
//PAGE 1
#define BNO_REG1_PAGE_ID						0//rw
#define BNO_REG1_ACC_CONFIG						1//rw
#define BNO_REG1_MAG_CONFIG						2//rw
#define BNO_REG1_GYR_CONFIG_0					3//rw
#define BNO_REG1_GYR_CONFIG_1					4//rw
#define BNO_REG1_ACC_SLEEP_CONFIG				5//rw
#define BNO_REG1_GYR_SLEEP_CONFIG				6//rw
#define BNO_REG1_INT_MSK						7//rw
#define BNO_REG1_INT_EN							8//rw
#define BNO_REG1_ACC_AM_THRES					9//rw
#define BNO_REG1_ACC_INT_SETTINGS				10//rw
#define BNO_REG1_ACC_HG_DURATION				11//rw
#define BNO_REG1_ACC_HG_THRES					12//rw
#define BNO_REG1_ACC_NM_THRES					13//rw
#define BNO_REG1_ACC_NM_SET						14//rw
#define BNO_REG1_GYR_INT_SETTING				15//rw
#define BNO_REG1_GYR_HR_X_SET					16//rw
#define BNO_REG1_GYR_DUR_X						17//rw
#define BNO_REG1_GYR_HR_Y_SET					18//r
#define BNO_REG1_GYR_DUR_Y						19//rw
#define BNO_REG1_GYR_HR_Z_SET					20//r
#define BNO_REG1_GYR_DUR_Z						21//rw
#define BNO_REG1_GYR_AM_THRES					22//rw
#define BNO_REG1_GYR_AM_SET						23//rw

#define BNO_NUM_REG_ADDRESSES1					BNO_REG1_GYR_AM_SET+1

//Register adresses PAGE 0
static const uint8_t BNO055_reg_table0[3][BNO_NUM_REG_ADDRESSES0] =
{	{
	0x00,	0x01,	0x02,	0x03,	0x04,	0x06,	0x07,	0x08,
	0x0A,	0x0C,	0x0E,	0x10,	0x12,	0x14,	0x16,	0x18,
	0x1A,	0x1C,	0x1E,	0x20,	0x22,	0x24,	0x26,	0x28,
	0x2A,	0x2C,	0x2E,	0x30,	0x32,	0x34,	0x35,	0x36,
	0x37,	0x38,	0x39,	0x3A,	0x3B,	0x3D,	0x3E,	0x3F,
	0x40,	0x41,	0x42,	0x55,	0x57,	0x59,	0x5B,	0x5D,
	0x5F,	0x61,	0x63,	0x65,	0x67,	0x69
},
//Register Length
{	1,		1,		1,		1,		2,		1,		1,		2,
	2,		2,		2,		2,		2,		2,		2,		2,
	2,		2,		2,		2,		2,		2,		2,		2,
	2,		2,		2,		2,		2,		1,		1,		1,
	1,		1,		1,		1,		1,		1,		1,		1,
	1,		1,		1,		2,		2,		2,		2,		2,
	2,		2,		2,		2,		2,		2
},
//Register Read/Write
{	0,		0,		0,		0,		0,		0,		2,		0,
	0,		0,		0,		0,		0,		0,		0,		0,
	0,		0,		0,		0,		0,		0,		0,		0,
	0,		0,		0,		0,		0,		0,		0,		0,
	0,		0,		0,		0,		2,		2,		2,		1,
	2,		2,		2,		2,		2,		2,		2,		2,
	2,		2,		2,		2,		2,		2
}
};

//Register adresses PAGE 1
static const uint8_t BNO055_reg_table1[3][BNO_NUM_REG_ADDRESSES1]=
{	{
	0x07,	0x08,	0x09,	0x0A,	0x0B,	0x0C,	0x0D,	0x0F,
	0x10,	0x11,	0x12,	0x13,	0x14,	0x15,	0x16,	0x17,
	0x18,	0x19,	0x1A,	0x1B,	0x1C,	0x1D,	0x1E,	0x1F
},
//Register Length
{	1,		1,		1,		1,		1,		1,		1,		1,
	1,		1,		1,		1,		1,		1,		1,		1,
	1,		1,		1,		1,		1,		1,		1,		1
},
//Register Read/Write
{	2,		2,		2,		2,		2,		2,		2,		2,
	2,		2,		2,		2,		2,		2,		2,		2,
	2,		2,		0,		2,		0,		2,		2,		2
}
};


#endif /* BNO055_REG_TABLE_H_ */