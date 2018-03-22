/*
 * FreeModbus Libary: 
 * Copyright (C) 
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portevent.c
 */

/* ----------------------- System includes ----------------------------------*/
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

/* ----------------------- Start implementation -----------------------------*/

BOOL
xMBPortEventInit( xQueueHandle* queue )
{
    *queue = xQueueCreate( 1, ( unsigned portBASE_TYPE ) sizeof( eMBEventType ) );
	return *queue != NULL ? TRUE : FALSE;
}

BOOL
xMBPortEventPost( xQueueHandle queue, eMBEventType eEvent )
{
    portBASE_TYPE   xEventSent = pdFALSE;

    xQueueSendFromISR( queue, &eEvent, &xEventSent );
    return xEventSent == pdTRUE ? TRUE : FALSE;
}

BOOL
xMBPortEventGet( xQueueHandle queue, eMBEventType * eEvent )
{
    BOOL xEventHappened = FALSE;

    if( xQueueReceive( queue, eEvent, portMAX_DELAY ) == pdTRUE )
    {
        xEventHappened = TRUE;
    }
    return xEventHappened;
}

void vMBPortEventClose( xQueueHandle queue )
{
	if ( queue != NULL )
    	free(queue);
}



