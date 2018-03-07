/*
 * ESCControl.h
 *
 * Created: 29.09.2017 19:14:02
 *  Author: Markus Lorenz
 */ 


#ifndef ESCCONTROL_H_
#define ESCCONTROL_H_

#include "sam.h"
#include <stdbool.h>
#include "config.h"
#include "ErrorHandling.h"

/*	MOTOR LAYOUT

|----|        |----|
| M1 |        | M2 |
|----|        |----|



|----|        |----|
| M3 |        | M4 |
|----|        |----|

 */


/*
 *
 *	Configures the 4 ESC pins as output and configures
 *	Timer 0-3
 *
 */
void esc_init();

/*
 *	Sets the duty cycle of the corresponding Timer from the motor
 */
ErrorCode esc_set(uint8_t MotorNum,int16_t speed);



#endif /* ESCCONTROL_H_ */