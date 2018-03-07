/*
 * SerialCOM.h
 *
 * Created: 12.02.2018 08:59:14
 *  Author: flola
 */ 


#ifndef SERIALCOM_H_
#define SERIALCOM_H_

#include "sam.h"
#include <stdlib.h>
#include <stdarg.h>
#include "config.h"
#include "uart0.h"
#include "ErrorHandling.h"

/*
* Transmission Types:
* 0x01		-		Debug message
* 0x10		-		Error
*/

typedef void (*SerialCOM_RECV_CALLBACK)(uint8_t* message, uint8_t Type);

ErrorCode SerialCOM_init();
ErrorCode SerialCOM_put_debug(char Text[]);
ErrorCode SerialCOM_put_debug_n(char Text[], uint8_t Length);
ErrorCode SerialCOM_register_call_back(SerialCOM_RECV_CALLBACK callback);
ErrorCode SerialCOM_put_error(char Text[]);
ErrorCode SerialCOM_force_put_error(char Text[]);
uint8_t SerialCOM_get_free_space();
ErrorCode SerialCOM_put_message(uint8_t message[], uint8_t Type, uint8_t Length);
ErrorCode SerialCOM_put_Command(char CommandChar, uint8_t Type);
ErrorCode SerialCOM_print_error(const char *fmt, ...);
ErrorCode SerialCOM_print_debug(const char *fmt, ...);


#endif /* SERIALCOM_H_ */