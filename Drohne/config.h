/*
 * config.h
 *
 * Created: 29.09.2017 19:37:39
 *  Author: Markus Lorenz
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

/*
 *
 *	General Configuration
 *
 */

// CPU frequency
#define F_CPU 16000000
// activate UART debugging?
//#define DEBUG


/*
 *
 *	ESC Configuration
 *
 */

// M1 ESC Timer OCR Offset
#define ESC_OFFSET_M1 50
// M2 ESC Timer OCR Offset
#define ESC_OFFSET_M2 295
// M3 ESC Timer OCR Offset
#define ESC_OFFSET_M3 298
// M4 ESC Timer OCR Offset
#define ESC_OFFSET_M4 294

// M1 ESC Slope Compensation Factor
#define ESC_SLOPE_COMP_M1 0.38
// M2 ESC Slope Compensation Factor
#define ESC_SLOPE_COMP_M2 0
// M3 ESC Slope Compensation Factor
#define ESC_SLOPE_COMP_M3 0
// M4 ESC Slope Compensation Factor
#define ESC_SLOPE_COMP_M4 0

// Maximum ESC Speed
#define ESC_MAX_POWER 960 // half of maximum motor power

/*
 *	
 *	RCReader Configuration
 *
 */

// Maximum Control Value Input (2000 = 1ms, 4000 = 2ms)
#define RC_ControlMax 3861
// Minimum Control Value Input (2000 = 1ms, 4000 = 2ms)
#define RC_ControlMin 2180
// Difference of the above defined values
#define RC_ControlDiff (RC_ControlMax - RC_ControlMin)


/*
 *
 *	UART Configuration
 *
 */

// Maximum amount of chars for the send buffer
#define MAX_UART_SEND_BUFFER 270


/*
 *	
 *	I2C (TWI) Configuration
 *
 */

// I2C (TWI) SCL Frequency
#define I2C_FREQ 100000
// I2C (TWI) Buffer Length
#define  I2C_BUFFER_LENGTH 32

#endif /* CONFIG_H_ */