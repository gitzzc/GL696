
#ifndef __TOUCHSCREEN_H__
#define __TOUCHSCREEN_H__

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"

extern xTaskHandle xTCTaskHandle;

portBASE_TYPE TouchScreen_Calibrate(portBASE_TYPE cmd);
void vTouchTask( void *pvParameters );

#endif

