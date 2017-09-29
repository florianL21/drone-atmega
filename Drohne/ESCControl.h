/*
 * ESCControl.h
 *
 * Created: 29.09.2017 19:14:02
 *  Author: Markus Lorenz
 */ 


#ifndef ESCCONTROL_H_
#define ESCCONTROL_H_

/*
 *
 *	Configures the 4 ESC pins as output and
 *	Timer 3 & 4
 *
 */
void esc_init();

/*
 *	Sets the Timer OCR register for ESC M1
 *	Implements value validation (min-max)
 *
 */
void esc_set_m1(int16_t speed);

/*
 *	Sets the Timer OCR register for ESC M2
 *	Implements value validation (min-max)
 *
 */
void esc_set_m2(int16_t speed);

/*
 *	Sets the Timer OCR register for ESC M3
 *	Implements value validation (min-max)
 *
 */
void esc_set_m3(int16_t speed);

/*
 *	Sets the Timer OCR register for ESC M4
 *	Implements value validation (min-max)
 *
 */
void esc_set_m4(int16_t speed);


/*	MOTOR LAYOUT

|----|        |----|
| M1 |        | M2 |
|----|        |----|



|----|        |----|
| M3 |        | M4 |
|----|        |----|

 */

#endif /* ESCCONTROL_H_ */