/*
 * HelperFunctions.h
 *
 * Created: 29.09.2017 21:44:29
 *  Author: Markus Lorenz
 */ 


#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

/*
 *	Maps the value x from an input range to an output range
 *	Parameters:
 *		x		- the value to be mapped
 *		in_min	- lower bound of input range
 *		in_max	- upper bound of input range
 *		out_min	- lower bound of output range
 *		out_max	- upper bound of output range
 */
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);


#endif /* HELPERFUNCTIONS_H_ */