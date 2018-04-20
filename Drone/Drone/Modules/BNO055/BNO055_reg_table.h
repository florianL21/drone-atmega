/*
 * BNO055_reg_table.h
 *
 * Created: 20.11.2017 18:44:01
 *  Author: flola
 */ 


#ifndef BNO055_REG_TABLE_H_
#define BNO055_REG_TABLE_H_

//PAGE 0
#define BNO_REG_CHIP_ID							0x00//r
#define BNO_REG_ACC_ID							0x01//r
#define BNO_REG_MAG_ID							0x02//r
#define BNO_REG_GYR_ID							0x03//r
#define BNO_REG_SW_REV_ID						0x04//r
#define BNO_REG_BL_REV_ID						0x06//r
#define BNO_REG_PAGE_ID							0x07//rw
#define BNO_REG_ACC_DATA_X						0x08//r
#define BNO_REG_ACC_DATA_Y						0x0A//r
#define BNO_REG_ACC_DATA_Z						0x0C//r
#define BNO_REG_MAG_DATA_X						0x0E//r
#define BNO_REG_MAG_DATA_Y						0x10//r
#define BNO_REG_MAG_DATA_Z						0x12//r
#define BNO_REG_GYR_DATA_X						0x14//r
#define BNO_REG_GYR_DATA_Y						0x16//r
#define BNO_REG_GYR_DATA_Z						0x18//r
#define BNO_REG_EUL_DATA_X						0x1A//r
#define BNO_REG_EUL_DATA_Y						0x1C//r
#define BNO_REG_EUL_DATA_Z						0x1E//r
#define BNO_REG_QUA_DATA_W						0x20//r
#define BNO_REG_QUA_DATA_X						0x22//r
#define BNO_REG_QUA_DATA_Y						0x24//r
#define BNO_REG_QUA_DATA_Z						0x26//r
#define BNO_REG_LIA_DATA_X						0x28//r
#define BNO_REG_LIA_DATA_Y						0x2A//r
#define BNO_REG_LIA_DATA_Z						0x2C//r
#define BNO_REG_GRV_DATA_X						0x2E//r
#define BNO_REG_GRV_DATA_Y						0x30//r
#define BNO_REG_GRV_DATA_Z						0x32//r
#define BNO_REG_TEMP							0x34//r
#define BNO_REG_CALIB_STAT						0x35//r
#define BNO_REG_ST_RESULT						0x36//r
#define BNO_REG_INT_STA							0x37//r
#define BNO_REG_SYS_CLK_STATUS					0x38//r
#define BNO_REG_SYS_STATUS						0x39//r
#define BNO_REG_SYS_ERR							0x3A//r
#define BNO_REG_UNIT_SEL						0x3B//r
#define BNO_REG_OPR_MODE						0x3D//rw
#define BNO_REG_PWR_MODE						0x3E//rw
#define BNO_REG_SYS_TRIGGER						0x3F//w
#define BNO_REG_TEMP_SOURCE						0x40//rw
#define BNO_REG_AXIS_MAP_CONFIG					0x41//rw
#define BNO_REG_AXIS_MAP_SIGN					0x42//rw
#define BNO_REG_ACC_OFFSET_X					0x55//rw
#define BNO_REG_ACC_OFFSET_Y					0x57//rw
#define BNO_REG_ACC_OFFSET_Z					0x59//rw
#define BNO_REG_MAG_OFFSET_X					0x5B//rw
#define BNO_REG_MAG_OFFSET_Y					0x5D//rw
#define BNO_REG_MAG_OFFSET_Z					0x5F//rw
#define BNO_REG_GYR_OFFSET_X					0x61//rw
#define BNO_REG_GYR_OFFSET_Y					0x63//rw
#define BNO_REG_GYR_OFFSET_Z					0x65//rw
#define BNO_REG_ACC_RADIUS						0x67//rw
#define BNO_REG_MAG_RADIUS						0x69//rw

//PAGE 1
#define BNO_REG1_PAGE_ID						0x07//rw
#define BNO_REG1_ACC_CONFIG						0x08//rw
#define BNO_REG1_MAG_CONFIG						0x09//rw
#define BNO_REG1_GYR_CONFIG_0					0x0A//rw
#define BNO_REG1_GYR_CONFIG_1					0x0B//rw
#define BNO_REG1_ACC_SLEEP_CONFIG				0x0C//rw
#define BNO_REG1_GYR_SLEEP_CONFIG				0x0D//rw
#define BNO_REG1_INT_MSK						0x0F//rw
#define BNO_REG1_INT_EN							0x10//rw
#define BNO_REG1_ACC_AM_THRES					0x11//rw
#define BNO_REG1_ACC_INT_SETTINGS				0x12//rw
#define BNO_REG1_ACC_HG_DURATION				0x13//rw
#define BNO_REG1_ACC_HG_THRES					0x14//rw
#define BNO_REG1_ACC_NM_THRES					0x15//rw
#define BNO_REG1_ACC_NM_SET						0x16//rw
#define BNO_REG1_GYR_INT_SETTING				0x17//rw
#define BNO_REG1_GYR_HR_X_SET					0x18//rw
#define BNO_REG1_GYR_DUR_X						0x19//rw
#define BNO_REG1_GYR_HR_Y_SET					0x1A//r
#define BNO_REG1_GYR_DUR_Y						0x1B//rw
#define BNO_REG1_GYR_HR_Z_SET					0x1C//r
#define BNO_REG1_GYR_DUR_Z						0x1D//rw
#define BNO_REG1_GYR_AM_THRES					0x1E//rw
#define BNO_REG1_GYR_AM_SET						0x1F//rw

#endif /* BNO055_REG_TABLE_H_ */