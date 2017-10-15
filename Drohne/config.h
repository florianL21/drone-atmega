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
 *	Debugging settings
 *
 */

#ifndef DEBUG
	//#define DEBUG
#endif
#ifndef DEBUG_UARTCOM
	//#define DEBUG_UARTCOM
#endif


/*
 *
 *	General Configuration
 *
 */

// CPU frequency
#define F_CPU 16000000
// activate UART debugging?


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


/*
 *
 *	UARTCOM Configuration
 *
 */

// Maximum amount of different reciver types that are used in the program
#define MAX_RECIVE_TYPES 20

#endif /* CONFIG_H_ */
