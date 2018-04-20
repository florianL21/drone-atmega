/*
 * config.h
 *
 * Created: 29.09.2017 19:37:39
 *  Author: Markus Lorenz
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdbool.h>

/*
 *
 *	General Configuration
 *
 */

// CPU frequency
#define F_CPU 84000000

//interrupt priority:
#define ISR_PRIORITY_RCREADER	1
#define ISR_PRIORITY_GPT		2
#define ISR_PRIORITY_USART0		3
#define ISR_PRIORITY_UART0		4


/*
 *
 *	ESC Configuration
 *
 */

/*	MOTOR LAYOUT

|----|        |----|
| M1 |        | M2 |
|----|        |----|



|----|        |----|
| M3 |        | M4 |
|----|        |----|

 */

// M1 ESC Timer OCR Offset
#define ESC_Offset_M1 400
// M2 ESC Timer OCR Offset
#define ESC_Offset_M2 400
// M3 ESC Timer OCR Offset
#define ESC_Offset_M3 400
// M4 ESC Timer OCR Offset
#define ESC_Offset_M4 400

// M1 ESC Slope Compensation Factor
#define ESC_SlopeComp_M1 0
// M2 ESC Slope Compensation Factor
#define ESC_SlopeComp_M2 0
// M3 ESC Slope Compensation Factor
#define ESC_SlopeComp_M3 0
// M4 ESC Slope Compensation Factor
#define ESC_SlopeComp_M4 0

//PWM Times:
#define ESC_PWM_PERIOD 52500
#define ESC_PWM_MIN_DUTY_CYCLE 2625	// 1ms duty cycle
// Maximum ESC Speed
#define ESC_MAX_LIMIT 3937 // half of maximum motor power

//this is ESC_MAX_LIMIT - ESC_PWM_MIN_DUTY_CYCLE precalculated.
#define ESC_MAX_ALLOWED_SPEED	1312
//#define ESC_MinLimit ESC_PWM_MIN_DUTY_CYCLE 


/*
 *	
 *	RCReader Configuration
 *
 */

// Difference of the above defined values
#define RC_CONTROL_DIFF (RC_ControlMax - RC_ControlMin)

//Dead spot of the joystick before values begin to change
#define RC_CONTROL_DEAD_SPOT__THROTTLE	30
// Minimum Control Value Input (2625 = 1ms, 5250 = 2ms)
#define RC_CCONTROL_MIN__THROTTLE		2840
// Maximum Control Value Input (2625 = 1ms, 5250 = 2ms)
#define RC_CONTROL_MAX__THROTTLE		5200

//Defines the value of the Input when the joystick is centered
#define RC_CONTROL_CENTER__PITCH		1075
#define RC_CONTROL_DEAD_SPOT__PITCH		20
#define RC_CONTROL_MIN__PITCH			2850
#define RC_CONTROL_MAX__PITCH			5200

#define RC_CONTROL_CENTER__ROLE			1075
#define RC_CONTROL_DEAD_SPOT__ROLE		20
#define RC_CONTROL_MIN__ROLE			2875
#define RC_CONTROL_MAX__ROLE			5200

#define RC_CONTROL_CENTER__YAW			1075
#define RC_CONTROL_DEAD_SPOT__YAW		20
#define RC_CONTROL_MIN__YAW				2865
#define RC_CONTROL_MAX__YAW				5200

#define RC_CONTROL_MIN__GEAR			2830
#define RC_CONTROL_MAX__GEAR			5200
#define RC_CONTROL_THRESCHHOLD__GEAR	2000


/*
RC_ControlDeadSpot
   _____|_____ ---------- RC_ControlMax
  /     |     \			^
 /   ___|___   \		|
|   |<----->|   |		|
|   |   O   |   |		| RC_ControlDiff
|   |___|___|   |		|
 \      |      /		v
  \_____|_____/ --------- RC_ControlMin
	    |     
RC_ControlCenter


*/

//PORT Configuration:
#define RCREADER_PIO_PORT	PIOB				//IO Port
#define RCREADER_PIO_IRQN	PIOB_IRQn			//interrupt routine
#define RCREADER_PIO_ID		ID_PIOB				//Port ID
#define RCREADER_INTERRUPT	PIOB_Handler		//name of the interrupt handler

#define THROTTLE_PIN		PIO_PB24		//thro
#define ROLE_PIN			PIO_PB25		//AILE
#define PITCH_PIN			PIO_PB26		//ELEV
#define YAW_PIN				PIO_PB27		//RUDO
#define GEAR_PIN			PIO_PB28		//GEAR

#define RCREADER_ENABLED_PINS	THROTTLE_PIN | ROLE_PIN | PITCH_PIN | YAW_PIN | GEAR_PIN;

/*
 *
 *	BNO055 Configuration
 *
 */

#define BNO_TRANSMISSION_TIMEOUT_MS	300
/*
 *
 *	UART Configuration
 *
 */

//maximum number of entrys in the send queue (uint16_t)	
#define UART0_QUEUE_MAX_ITEMS	50	

/*
 *
 *	USART0 Configuration
 *
 */

//maximum number of entrys in the send queue (uint16_t)	
#define USART0_QUEUE_MAX_ITEMS	50	

/*
 *
 *	SerialCOM
 *
 */

//Maximum amount  of chars allowed for the print function
#define SERIALCOM_MAX_PRINT_CHARS	300

/*
 *
 *	ErrorHandling
 *
 */

//defines the maximum amount of errors that get stored in the error array
#define MAX_ERROR_COUNT 20

/*
 *
 *	GPT
 *
 */

//defines the maximum amount of General Purpose Timers that can be used
#define MAX_NUM_GPT 20

#endif /* CONFIG_H_ */
