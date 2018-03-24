/*
 * WDT_config.c
 *
 * Created: 24.03.2018 08:17:31
 *  Author: flola
 */ 

#include "WDT.h"

void WDT_init(uint16_t wdt_count_register)
{
	WDT->WDT_MR = WDT_MR_WDV(wdt_count_register) | WDT_MR_WDD(wdt_count_register) | WDT_MR_WDRSTEN; //About 1s
	
}

void WDT_disable()
{
	WDT->WDT_MR = 0x00000000; // disable WDT
}

void WDT_restart()
{
	WDT->WDT_CR = 0xA5000000 | WDT_CR_WDRSTT;
}
