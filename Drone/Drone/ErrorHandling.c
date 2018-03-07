/*
 * ErrorHandling.c
 *
 * Created: 07.03.2018 16:23:54
 *  Author: flola
 */ 
#include "ErrorHandling.h"

uint32_t errorStack[MAX_ERROR_COUNT] = {0};
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

bool ErrorHandling_catch(uint32_t* Error)
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
/*
void error_handler_print()
{
	if(errorCount != 0)
	{
		char ErrorToCharBuffer[10] = "";
		for (uint8_t i = 0; i < errorCount; i++)
		{
			itoa(errorStack[i], ErrorToCharBuffer, 16);
			ErrorToCharBuffer[2]='\0';
			SerialCOM_put_error(ErrorToCharBuffer);
		}
		errorCount = 0;
	}
}
*/