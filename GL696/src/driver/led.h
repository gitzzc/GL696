#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>

#include "timer.h"
#include "gpio.h"

typedef struct _LED_CONFIG_ 
{
	uint32_t led;
	uint32_t on_time;
	uint32_t off_time;
	uint32_t count;
	uint32_t default_on_time;
	uint32_t default_off_time;
	sTIMEOUT stimer;
} sLED_CONFIG,*psLED_CONFIG;

#define SYSTEM_LED_ON() 	/*DIO_Write(DIO_LED,pdLOW)*/
#define SYSTEM_LED_OFF() 	/*DIO_Write(DIO_LED,pdHIGH)*/


void LED_SetStartup(psLED_CONFIG cfg,uint32_t on_time,uint32_t off_time,uint32_t off_count);
void LED_SetDefault(psLED_CONFIG cfg,uint32_t on_time,uint32_t off_time);
void LED_Task(psLED_CONFIG cfg);

#endif

