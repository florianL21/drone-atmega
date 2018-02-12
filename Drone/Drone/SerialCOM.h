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
#include "config.h"
#include "uart0.h"

/*
* Transmission Types:
* 0x01		-		Debug message
* 0x10		-		Error
*/

typedef void (*SerialCOM_RECV_CALLBACK)(uint8_t* message, uint8_t Type);

StatusCode SerialCOM_init();
StatusCode SerialCOM_put_debug(char Text[]);
StatusCode SerialCOM_put_debug_n(char Text[], uint8_t Length);



#endif /* SERIALCOM_H_ */