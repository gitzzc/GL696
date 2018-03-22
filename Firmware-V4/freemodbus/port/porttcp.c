/*
 * FreeModbus Libary: lwIP Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
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
 * File: $Id: porttcp.c,v 1.1 2006/08/30 23:18:07 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "port.h"
#include "modbus.h"
/* ----------------------- uIP includes ------------------------------------*/
#include "uip.h"
#include "webserver.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- MBAP Header --------------------------------------*/
#define MB_TCP_UID          6
#define MB_TCP_LEN          4
#define MB_TCP_FUNC         7

/* ----------------------- Defines  -----------------------------------------*/
#define MB_TCP_DEFAULT_PORT 502 /* TCP listening port. */
#define MB_TCP_BUF_SIZE     ( 256 + 7 ) /* Must hold a complete Modbus TCP frame. */

#define uxModbusQueueLength	1

typedef struct
{
  unsigned short len;
  char 	buf[MB_TCP_BUF_SIZE];
} xModbusMessage;


/* ----------------------- Prototypes ---------------------------------------*/
void            vMBPortLog( eMBPortLogLevel eLevel, const CHAR * szModule,
                            const CHAR * szFmt, ... );

/* ----------------------- Static variables ---------------------------------*/
/* The queue used to send messages to the modbus task. */
xQueueHandle xModbusTCPRxQueue, xModbusTCPTxQueue;
xQueueHandle xMBTCPEventQueue;
xModbusMessage xMessage;

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Begin implementation -----------------------------*/
BOOL
xMBTCPPortInit( USHORT usTCPPort )
{
	/* Create the queues used to hold Rx/Tx Msg. */
	xModbusTCPRxQueue = xQueueCreate( uxModbusQueueLength, ( unsigned portBASE_TYPE ) sizeof( xModbusMessage ) );
	xModbusTCPTxQueue = xQueueCreate( uxModbusQueueLength, ( unsigned portBASE_TYPE ) sizeof( xModbusMessage ) );
	
	return TRUE;
}

void
vMBTCPPortClose(  )
{
    /* Release resources for the event queue. */
    vMBPortEventClose( xMBTCPEventQueue );
}

void
vMBTCPPortDisable( void )
{
}


BOOL
xMBTCPPortGetRequest( UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength )
{
	if ( xQueueReceive( xModbusTCPRxQueue, &xMessage, 10/portTICK_RATE_MS ) == pdTRUE ){
	    *ppucMBTCPFrame = (UCHAR*)xMessage.buf;
	    *usTCPLength = xMessage.len;
	
	    return TRUE;
	}
	return FALSE;
}

BOOL
xMBTCPPortSendResponse( const UCHAR * pucMBTCPFrame, USHORT usTCPLength )
{
    BOOL bFrameSent = FALSE;

	if( xQueueSend( xModbusTCPTxQueue, pucMBTCPFrame, configTICK_RATE_HZ*10 ) == pdPASS )
	{
		bFrameSent = TRUE;
	#ifdef MB_TCP_DEBUG
	//	prvvMBTCPLogFrame( "MBTCP-SENT", xMessage.buf, usTCPLength );
	#endif
	}
    return bFrameSent;
}


/*---------------------------------------------------------------------------*/
//static struct modbus_tcp_state s;
#if 1 
void
modbus_tcp_init(void)
{
  	uip_listen(HTONS(502));
}

/*---------------------------------------------------------------------------*/
static void
senddata(void)
{
	if ( xQueueReceive( xModbusTCPTxQueue, uip_appdata, 10/portTICK_RATE_MS ) == pdPASS ){
		uip_send(uip_appdata, ((((u8_t*)uip_appdata)[MB_TCP_LEN]<<8) | ((u8_t*)uip_appdata)[MB_TCP_LEN+1])-1+MB_TCP_FUNC );
	}
}
/*---------------------------------------------------------------------------*/
static void
newdata(void)
{
	u16_t len,usLength;

	len = uip_datalen();	
    if( len >= MB_TCP_FUNC )
    {
        /* Length is a byte count of Modbus PDU (function code + data) and the
         * unit identifier. */
        usLength  = ((u8_t*)uip_appdata)[MB_TCP_LEN] << 8U;
        usLength |= ((u8_t*)uip_appdata)[MB_TCP_LEN + 1];

        /* Is the frame already complete. */
        if( len >= ( MB_TCP_UID + usLength ) ) {
			*((u8_t*)uip_appdata - 2) = len;
			if( xQueueSend( xModbusTCPRxQueue, (u8_t*)uip_appdata-2, 1000/portTICK_RATE_MS ) == pdPASS ){
				if ( xMBTCPEventQueue )
					( void )xMBPortEventPost( xMBTCPEventQueue, EV_FRAME_RECEIVED );
			}
		}
	}
}
/*---------------------------------------------------------------------------*/

void
modbus_tcp_appcall(void)
{
  //struct modbus_tcp_state *s = (struct modbus_tcp_state *)&(uip_conn->appstate);
  
  if(uip_connected()) {
    /*    tcp_markconn(uip_conn, &s);*/
    //s->state = STATE_NORMAL;
  }

  //if(s->state == STATE_CLOSE) {
    //s->state = STATE_NORMAL;
  //  uip_close();
  //  return;
  //}
  
  if(uip_closed() ||
     uip_aborted() ||
     uip_timedout()) {
  }
  
  if(uip_acked()) {
  }
  
  if(uip_newdata()) {
    newdata();
  }
  
  if(uip_rexmit() ||
     uip_newdata() ||
     uip_acked() ||
     uip_connected() ||
     uip_poll()) {
    senddata();
  }
}
#endif

