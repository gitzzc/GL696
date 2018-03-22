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
 * File: $Id: portserial.c
 */

/* ----------------------- System includes ----------------------------------*/

/* ----------------------- Platform includes --------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- STM32 includes ----------------------------------*/
#include "stm32f10x.h"
#include "serials.h"

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
USART_TypeDef* MB_UART_Dev;

/* ----------------------- -------------------- -----------------------------*/

BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity Parity )
{
	eCOMPort ePort;
	eBaud eWantedBaud;
	eParity eWantedParity;
	eDataBits eWantedDataBits;
	eStopBits eWantedStopBits = serSTOP_1;
	BOOL ret;

    switch ( ucPort ){
		case 0: 	ePort = serCOM1;MB_UART_Dev = USART1; break;
		case 1: 	ePort = serCOM2;MB_UART_Dev = USART2; break;
		case 2: 	ePort = serCOM3;MB_UART_Dev = USART3; break;
		default :	ePort = serCOM1;MB_UART_Dev = USART1; break;
	}
	
	switch ( ulBaudRate ){
		case 1200:	eWantedBaud = ser1200;	break;
		case 2400:	eWantedBaud = ser2400;	break;
		case 4800:	eWantedBaud = ser4800;	break;
		case 9600:	eWantedBaud = ser9600;	break;
		case 19200:	eWantedBaud = ser19200;	break;
		case 38400:	eWantedBaud = ser38400;	break;
		case 57600:	eWantedBaud = ser57600;	break;
		case 115200:eWantedBaud = ser115200;break;
		default :	eWantedBaud = ser115200;break;
	}

    switch ( Parity ) {
	    case MB_PAR_EVEN: 	eWantedParity = serEVEN_PARITY;	break;
		case MB_PAR_ODD:	eWantedParity = serODD_PARITY;	break;
	    case MB_PAR_NONE:	eWantedParity = serNO_PARITY;	break;
	    default :			eWantedParity = serNO_PARITY;	break;
    }

    switch ( ucDataBits ) {
	    case 7:	eWantedDataBits = serBITS_7; break;
	    case 8:	eWantedDataBits = serBITS_8; break;
	    default:eWantedDataBits = serBITS_8; break;
    }

	ret = xSerialPortBaseInit( ePort, eWantedBaud, eWantedParity, eWantedDataBits, eWantedStopBits );
	vMBPortSerialEnable( FALSE, FALSE );
	
	return ret;
}

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if( xRxEnable )
		USART_ITConfig( MB_UART_Dev, USART_IT_RXNE, ENABLE );
    else
		USART_ITConfig( MB_UART_Dev, USART_IT_RXNE, DISABLE );

    if( xTxEnable )
		USART_ITConfig( MB_UART_Dev, USART_IT_TXE, ENABLE );		
    else
		USART_ITConfig( MB_UART_Dev, USART_IT_TXE, DISABLE );		
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	USART_SendData( MB_UART_Dev, ucByte );
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte = USART_ReceiveData( MB_UART_Dev );
    return TRUE;
}

BOOL
prvMBPortTXIsEnabled(  )
{
	return (BOOL)(MB_UART_Dev->CR1 & (1<<7) );
}

BOOL
prvMBPortRXIsEnabled(  )
{
	return (BOOL)(MB_UART_Dev->CR1 & (1<<5) );
}
#if 1
void
USART1_IRQHandler( void )
{
    static BOOL     xTaskWokenReceive = FALSE;
    static BOOL     xTaskWokenTransmit = FALSE;
	
    portENTER_SWITCHING_ISR(  );

    if( prvMBPortTXIsEnabled(  ) && USART_GetITStatus( MB_UART_Dev, USART_IT_TXE ) )
    {
        xTaskWokenTransmit = pxMBFrameCBTransmitterEmpty(  );
    }
    if( prvMBPortRXIsEnabled(  ) && USART_GetITStatus( MB_UART_Dev, USART_IT_RXNE ) )
    {
        xTaskWokenReceive = pxMBFrameCBByteReceived(  );
    }

    portEXIT_SWITCHING_ISR( ( xTaskWokenReceive
                              || xTaskWokenTransmit ) ? pdTRUE : pdFALSE );
}
#endif
void
vMBPortClose(  )
{
    /* Release resources for the event queue. */
    vMBPortEventClose( xMBSerialEventQueue );
}


