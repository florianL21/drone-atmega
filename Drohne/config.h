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
#ifndef DEBUG
	//#define DEBUG
#endif


/*
 *
 *	ESC Configuration
 *
 */

// M1 ESC Timer OCR Offset
#define ESC_Offset_M1 50
// M2 ESC Timer OCR Offset
#define ESC_Offset_M2 295
// M3 ESC Timer OCR Offset
#define ESC_Offset_M3 298
// M4 ESC Timer OCR Offset
#define ESC_Offset_M4 294

// M1 ESC Slope Compensation Factor
#define ESC_SlopeComp_M1 0.38
// M2 ESC Slope Compensation Factor
#define ESC_SlopeComp_M2 0
// M3 ESC Slope Compensation Factor
#define ESC_SlopeComp_M3 0
// M4 ESC Slope Compensation Factor
#define ESC_SlopeComp_M4 0

// Maximum ESC Speed
#define ESC_MaxLimit 960 // half of maximum motor power



/*
 *
 *	UART Configuration
 *
 */

// Maximum amount of chars for the send buffer
#define MAX_UART_SEND_BUFFER 270

#endif /* CONFIG_H_ */