/*
 * ErrorHandling.c
 *
 * Created: 07.03.2018 16:23:54
 *  Author: flola
 */ 
#include "ErrorHandling.h"

uint64_t errorStack[MAX_ERROR_COUNT] = {0};
uint8_t errorCount = 0;

void ErrorHandling_throw(ErrorCode Error)
{
	if(Error != 0 && errorCount < MAX_ERROR_COUNT)
	{
		errorStack[errorCount++] = Error;
	}
}

void ErrorHandling_throw_b(Modules Module, Functions Function, Errors Error)
{
	if((Module | Function | Error) != 0 && errorCount < MAX_ERROR_COUNT)
	{
		errorStack[errorCount++] = Module | Function | Error;
	}
}

bool ErrorHandling_catch(ErrorCode* Error)
{
	if(errorCount != 0)
	{
		*Error = errorStack[--errorCount];
		return true;
	} 
	else 
	{
		return false;
	}
}

ErrorCode ErrorHandling_set_top_level(ErrorCode Error, Modules LastModule, Functions LastFunction)
{
	if(Error != SUCCESS)
	{
		Error &= 0xFFFFFF;
		Error |= LastModule << 16;
		Error |= LastFunction << 16;
	}
	return Error;
}