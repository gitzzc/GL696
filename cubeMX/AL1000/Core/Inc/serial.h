/*
    FreeRTOS V6.0.5 - Copyright (C) 2010 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS eBook                                  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#define TBUFSIZE	270
#define RBUFSIZE	270
#define MAXPORT		3

typedef struct _SerialQueue_
{
	uint8_t  tbuf[TBUFSIZE];	//串口数据发送缓冲
	uint8_t  rbuf[RBUFSIZE];	//串口数据接收缓冲
	uint16_t tin;				//发送缓冲区队列指针
	uint16_t tout;
	uint16_t rin;				//接收缓冲区队列指针
	uint16_t rout;
	uint8_t  tx_complete;
	UART_HandleTypeDef* handler;
}sSerialQueue,*psSerialQueue;


extern uint8_t tbuf[MAXPORT][TBUFSIZE];	//串口数据发送缓冲
extern uint8_t rbuf[MAXPORT][RBUFSIZE];	//串口数据接收缓冲
extern sSerialQueue serQueue[MAXPORT];

uint8_t SerialBufInit( UART_HandleTypeDef *UartHandle );
uint8_t SerialGetChar( UART_HandleTypeDef *UartHandle, uint8_t *pcRxedChar );
uint8_t SerialPutBuf(UART_HandleTypeDef *UartHandle,uint8_t* buf,uint16_t len);


#endif

