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

//Main debug switch
//#define DEBUG_SWITCH
//debug switch for UARTCOM
//#define DEBUG_UARTCOM
//force the UARTCOM debug to send its messages through the force debug command. DEBUG_UARTCOM Has to be enabled!!!
//#define FORCE_UARTCOM_DEBUG
//debug switch for UART0
//#define DEBUG_UART0


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

/*	MOTOR LAYOUT

|----|        |----|
| M1 |        | M2 |
|----|        |----|



|----|        |----|
| M3 |        | M4 |
|----|        |----|

 */

// M1 ESC Timer OCR Offset
#define ESC_Offset_M1 10
// M2 ESC Timer OCR Offset
#define ESC_Offset_M2 390
// M3 ESC Timer OCR Offset
#define ESC_Offset_M3 390
// M4 ESC Timer OCR Offset
#define ESC_Offset_M4 390

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
#define ESC_MaxLimit 3937 // half of maximum motor power
//#define ESC_MinLimit ESC_PWM_MIN_DUTY_CYCLE 


/*
 *	
 *	RCReader Configuration
 *
 */

// Difference of the above defined values
#define RC_CONTROL_DIFF (RC_ControlMax - RC_ControlMin)

//Dead spot of the joystick before values begin to change
#define RC_CONTROL_DEAD_SPOT__THROTTLE	60
// Minimum Control Value Input (2625 = 1ms, 5250 = 2ms)
#define RC_CCONTROL_MIN__THROTTLE		2840
// Maximum Control Value Input (2625 = 1ms, 5250 = 2ms)
#define RC_CONTROL_MAX__THROTTLE		5200

//Defines the value of the Input when the joystick is centered
#define RC_CONTROL_CENTER__PITCH		1075
#define RC_CONTROL_DEAD_SPOT__PITCH		40
#define RC_CONTROL_MIN__PITCH			2850
#define RC_CONTROL_MAX__PITCH			5200

#define RC_CONTROL_CENTER__ROLE			1075
#define RC_CONTROL_DEAD_SPOT__ROLE		40
#define RC_CONTROL_MIN__ROLE			2875
#define RC_CONTROL_MAX__ROLE			5200

#define RC_CONTROL_CENTER__YAW			1075
#define RC_CONTROL_DEAD_SPOT__YAW		40
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
