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


//#define DEBUG
#define DEBUG_UARTCOM
#define DEBUG_UART0


/*
 *
 *	General Configuration
 *
 */

// CPU frequency
#define F_CPU 16000000


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

//maximum number of entrys in the send queue (uint16_t)	
#define UART_QUEUE_MAX_ITEMS	50	

/*
 *
 *	UARTCOM Configuration
 *
 */

// Maximum amount of different reciver types that are used in the program
#define UARTCOM_MAX_RECIVE_TYPES 20

#endif /* CONFIG_H_ */
