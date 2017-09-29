/*
 * HelperFunctions.c
 *
 * Created: 29.09.2017 21:44:56
 *  Author: Markus Lorenz
 */ 

#include "HelperFunctions.h"

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}