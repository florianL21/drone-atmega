/*
 * ErrorHandling.c
 *
 * Created: 07.03.2018 16:23:54
 *  Author: flola
 */ 
#include "ErrorHandling.h"

ErrorCode errorStack[MAX_ERROR_COUNT] = {0};
uint8_t errorCount = 0;

void ErrorHandling_throw(ErrorCode Error)
{
	if(Error != SUCCESS && errorCount < MAX_ERROR_COUNT)
	{
		errorStack[errorCount++] = Error;
	}
}

void ErrorHandling_throw_b(Modules Module, Functions Function, Errors Error)
{
	if((Module | Function | Error) != SUCCESS && errorCount < MAX_ERROR_COUNT)
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
		Error |= ((ErrorCode)LastModule) << 16;
		Error |= LastFunction << 16;
	}
	return Error;
}


void ErrorHandling_get_error_description(ErrorCode Error, char errorDescription[])
{
	//error
	switch(Error)
	{
		case SUCCESS:
			strcat(errorDescription, "Undefined");
		break;
		case ERROR_GENERIC:
			strcat(errorDescription, "Generic");
		break;
		case ERROR_ARGUMENT_OUT_OF_RANGE:
			strcat(errorDescription, "Argument out of range");
		break;
		case ERROR_GOT_NULL_POINTER:
			strcat(errorDescription, "Got null pointer");
		break;
		case ERROR_MALLOC_RETURNED_NULL:
			strcat(errorDescription, "Malloc returned NULL");
		break;
		case ERROR_NOT_READY_FOR_OPERATION:
			strcat(errorDescription, "Not ready for operation");
		break;
		case ERROR_INVALID_ARGUMENT:
			strcat(errorDescription, "Invalid argument");
		break;
		case ERROR_WRONG_DEVICE_ID:
			strcat(errorDescription, "Wrong device ID");
		break;
		case ERROR_LENGTH_MISSMATCH:
			strcat(errorDescription, "Length missmatch");
		break;
		case ERROR_QUEUE_WAS_EMPTY:
			strcat(errorDescription, "Queue was empty");
		break;
		case ERROR_WRITE_FAILED:
			strcat(errorDescription, "Write failed");
		break;
		case ERROR_ADDRESS_TOO_LOW:
			strcat(errorDescription, "Address too low");
		break;
		case ERROR_ADDRESS_TOO_HIGH:
			strcat(errorDescription, "Address too high");
		break;
		case ERROR_FAILED_TO_LOCK_FLASH:
			strcat(errorDescription, "Failed to lock flash");
		break;
		case ERROR_FAILED_TO_UNLOCK_FLASH:
			strcat(errorDescription, "Failed to unlock flash");
		break;
		case ERROR_ADDRESS_NOT_4_BYTE_BOUDARY:
			strcat(errorDescription, "Address not in a 4 byte boundary");
		break;
		case ERROR_TRANSMISSION_ERROR:
			strcat(errorDescription, "Transmission error");
		break;
		case ERROR_READ_FAIL:
			strcat(errorDescription, "Read fail");
		break;
		case ERROR_WRITE_FAIL:
			strcat(errorDescription, "Write fail");
		break;
		case ERROR_REGMAP_INVALID_ADDRESS:
			strcat(errorDescription, "Regmap invalid address");
		break;
		case ERROR_WRONG_START_BYTE:
			strcat(errorDescription, "Wrong start byte");
		break;
		case ERROR_BUS_OVER_RUN:
			strcat(errorDescription, "Bus overrun");
		break;
		case ERROR_MAX_LENGTH:
			strcat(errorDescription, "Max length");
		break;
		case ERROR_MIN_LENGTH:
			strcat(errorDescription, "min length");
		break;
		case ERROR_RECEIVE_CHARACTER_TIMEOUT:
			strcat(errorDescription, "Receive character timeout");
		break;
		default:
			strcat(errorDescription, "Unknown");
		break;
	} //32
}

void ErrorHandling_get_function_description(ErrorCode Error, char errorDescription[])
{
	//Function
	switch(Error)
	{
		case FUNCTION_SUCCESS:
			strcat(errorDescription, "Undefined");
		break;
		case FUNCTION_GENERIC:
			strcat(errorDescription, "Generic");
		break;
		case FUNCTION_init_fusion_mode:
			strcat(errorDescription, "init_fusion_mode");
		break;
		case FUNCTION_init_non_fusion_mode:
			strcat(errorDescription, "init_non_fusion_mode");
		break;
		case FUNCTION_start_measurement:
			strcat(errorDescription, "start_measurement");
		break;
		case FUNCTION_calculate_calibration:
			strcat(errorDescription, "calculate_calibration");
		break;
		case FUNCTION_register_data_ready_callback:
			strcat(errorDescription, "register_data_ready_callback");
		break;
		case FUNCTION_register_error_callback:
			strcat(errorDescription, "register_error_callback");
		break;
		case FUNCTION_success:
			strcat(errorDescription, "success");
		break;
		case FUNCTION_error:
			strcat(errorDescription, "error");
		break;
		case FUNCTION_write_and_wait_for_response_1byte:
			strcat(errorDescription, "write_and_wait_for_response_1byte");
		break;
		case FUNCTION_read_and_wait_for_response_1byte:
			strcat(errorDescription, "read_and_wait_for_response_1byte");
		break;
		case FUNCTION_write_and_wait_for_response:
			strcat(errorDescription, "write_and_wait_for_response");
		break;
		case FUNCTION_read_and_wait_for_response:
			strcat(errorDescription, "read_and_wait_for_response");
		break;
		case FUNCTION_register_write_1byte_by_table:
			strcat(errorDescription, "register_write_1byte_by_table");
		break;
		case FUNCTION_register_write_by_table:
			strcat(errorDescription, "register_write_by_table");
		break;
		case FUNCTION_register_read_by_table:
			strcat(errorDescription, "register_read_by_table");
		break;
		case FUNCTION_register_read_1byte_by_table:
			strcat(errorDescription, "register_read_1byte_by_table");
		break;
		case FUNCTION_Init:
			strcat(errorDescription, "Init");
		break;
		case FUNCTION_register_success_callback:
			strcat(errorDescription, "register_success_callback");
		break;
		case FUNCTION_register_read:
			strcat(errorDescription, "register_read");
		break;
		case FUNCTION_register_write:
			strcat(errorDescription, "register_write");
		break;
		case FUNCTION_response_received:
			strcat(errorDescription, "response_received");
		break;
		case FUNCTION_runtime_success:
			strcat(errorDescription, "runtime_success");
		break;
		case FUNCTION_set:
			strcat(errorDescription, "set");
		break;
		case FUNCTION_lock:
			strcat(errorDescription, "lock");
		break;
		case FUNCTION_unlock:
			strcat(errorDescription, "unlock");
		break;
		case FUNCTION_write:
			strcat(errorDescription, "write");
		break;
		case FUNCTION_write_unlocked:
			strcat(errorDescription, "write_unlocked");
		break;
		case FUNCTION_queue_delete:
			strcat(errorDescription, "queue_delete");
		break;
		case FUNCTION_queue_write:
			strcat(errorDescription, "queue_write");
		break;
		case FUNCTION_median_filter_new:
			strcat(errorDescription, "median_filter_new");
		break;
		case FUNCTION_median_filter_add:
			strcat(errorDescription, "median_filter_add");
		break;
		case FUNCTION_Initialize:
			strcat(errorDescription, "Initialize");
		break;
		case FUNCTION_Compute:
			strcat(errorDescription, "Compute");
		break;
		case FUNCTION_SetTunings:
			strcat(errorDescription, "SetTunings");
		break;
		case FUNCTION_SetSampleTime:
			strcat(errorDescription, "SetSampleTime");
		break;
		case FUNCTION_SetOutputLimits:
			strcat(errorDescription, "SetOutputLimits");
		break;
		case FUNCTION_SetControllerDirection:
			strcat(errorDescription, "SetControllerDirection");
		break;
		case FUNCTION_register_call_back:
			strcat(errorDescription, "register_call_back");
		break;
		case FUNCTION_put_message:
			strcat(errorDescription, "put_message");
		break;
		case FUNCTION_force_put_message:
			strcat(errorDescription, "force_put_message");
		break;
		case FUNCTION_put_data:
			strcat(errorDescription, "put_data");
		break;
		case FUNCTION_puts_blocking:
			strcat(errorDescription, "puts_blocking");
		break;
		case FUNCTION_set_receiver_length:
			strcat(errorDescription, "set_receiver_length");
		break;
		case FUNCTION_register_received_callback:
			strcat(errorDescription, "register_received_callback");
		break;
		case FUNCTION_get_calibration:
			strcat(errorDescription, "get_calibration");
		break;
		case FUNCTION_register_write_1byte:
			strcat(errorDescription, "register_write_1byte");
		break;
		case FUNCTION_register_read_1byte:
			strcat(errorDescription, "register_read_1byte");
		break;
		case FUNCTION_write_uint8_t:
			strcat(errorDescription, "write_uint8_t");
		break;
		case FUNCTION_write_float:
			strcat(errorDescription, "write_float");
		break;
		case FUCNTION_put_Command:
			strcat(errorDescription, "put_Command");
		break;
		case FUCNTION_put_debug:
			strcat(errorDescription, "put_debug");
		break;
		case FUCNTION_put_debug_n:
			strcat(errorDescription, "put_debug_n");
		break;
		case FUCNTION_put_error:
			strcat(errorDescription, "put_error");
		break;
		case FUCNTION_force_put_error:
			strcat(errorDescription, "force_put_error");
		break;
		case FUNCTION_print_debug:
			strcat(errorDescription, "print_debug");
		break;
		case FUNCTION_print_error:
			strcat(errorDescription, "print_error");
		break;
		case FUNCTION_puts:
			strcat(errorDescription, "puts");
		break;
		case FUNCTION_put_float:
			strcat(errorDescription, "put_float");
		break;
		case FUNCTION_put_int:
			strcat(errorDescription, "put_int");
		break;
		case FUNCTION_put_int_blocking:
			strcat(errorDescription, "put_int_blocking");
		break;
		default:
			strcat(errorDescription, "Unknown");
		break;
	} //33
}

void ErrorHandling_get_module_description(ErrorCode Error, char errorDescription[])
{
	//Module
	switch(Error)
	{
		case MODULE_SUCCESS:
			strcat(errorDescription, "Undefined");
		break;
		case MODULE_GENERIC:
			strcat(errorDescription, "Generic");
		break;
		case MODULE_MAIN:
			strcat(errorDescription, "Main");
		break;
		case MODULE_BNO055:
			strcat(errorDescription, "BNO055");
		break;
		case MODULE_BNOCOM:
			strcat(errorDescription, "BNOCOM");
		break;
		case MODULE_ESCCONTROL:
			strcat(errorDescription, "ESCControl");
		break;
		case MODULE_FLASHSTORAGE:
			strcat(errorDescription, "FlashStorage");
		break;
		case MODULE_HELPERFUNCTIONS:
			strcat(errorDescription, "HelperFunctions");
		break;
		case MODULE_PID:
			strcat(errorDescription, "PID");
		break;
		case MODULE_RCREADER:
			strcat(errorDescription, "RCReader");
		break;
		case MODULE_SERIALCOM:
			strcat(errorDescription, "SerialCOM");
		break;
		case MODULE_UART0:
			strcat(errorDescription, "UART0");
		break;
		case MODULE_USART0:
			strcat(errorDescription, "USART0");
		break;
		default:
			strcat(errorDescription, "Unknown");
		break;
	} //15
}