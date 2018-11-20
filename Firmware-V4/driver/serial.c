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

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x.h"

/* Demo application includes. */
#include "serials.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE	( ( xQueueHandle ) 0 )
#define serNO_BLOCK			( ( portTickType ) 0 )
#define serTX_BLOCK_TIME	( 40 / portTICK_RATE_MS )

#define serMAX_PORTS		3

//#define USE_USART1
#define USE_USART2
#define USE_USART3

static xComPort xPorts[serMAX_PORTS];
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
signed portBASE_TYPE xSerialPortBaseInit( eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits)
{
	signed portBASE_TYPE xReturn = 0;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	uint32_t baud,parity,data_bits,stop_bits;

	if ( ePort > serMAX_PORTS )
		return pdFALSE;
		
	switch ( eWantedBaud ){
		case ser1200:	baud = 1200;	break;
		case ser2400:	baud = 2400;	break;
		case ser4800:	baud = 4800;	break;
		case ser9600:	baud = 9600;	break;
		case ser19200:	baud = 19200;	break;
		case ser38400:	baud = 38400;	break;
		case ser57600:	baud = 57600;	break;
		case ser115200:	baud = 115200;	break;
		default :		baud = 115200;	break;
	}
	
	switch ( eWantedParity ){
		case serNO_PARITY:		parity = USART_Parity_No;	break;
		case serODD_PARITY:		parity = USART_Parity_Odd;	break;
		case serEVEN_PARITY:	parity = USART_Parity_Even;	break;
		default :				parity = USART_Parity_No;	break;
	}
	
	switch ( eWantedDataBits ){
		case serBITS_8:		data_bits = USART_WordLength_8b;	break;
		case serBITS_9:		data_bits = USART_WordLength_9b;	break;
		default :			data_bits = USART_WordLength_8b;	break;
	}

	switch ( eWantedStopBits ){
		case serSTOP_1:		stop_bits = USART_StopBits_1;	break;
		case serSTOP_2:		stop_bits = USART_StopBits_2;	break;
		default :			stop_bits = USART_StopBits_2;	break;
	}

	switch ( ePort ){
	case 0:
//		USART_DeInit( USART1 );

		/* Enable USART1 clock */
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE );	

		/* Configure USART1 Rx (PA10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init( GPIOA, &GPIO_InitStructure );
		
		/* Configure USART1 Tx (PA9) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init( GPIOA, &GPIO_InitStructure );

		USART_InitStructure.USART_BaudRate = baud;
		USART_InitStructure.USART_WordLength = data_bits;
		USART_InitStructure.USART_StopBits = stop_bits;
		USART_InitStructure.USART_Parity = parity ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init( USART1, &USART_InitStructure );
		
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );
		
		USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
		USART_Cmd( USART1, ENABLE );
		xReturn = pdTRUE;
		break;
	case 1:
		USART_DeInit( USART2 );

		/* Enable USART1 clock */
		RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );	
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE );	

		GPIO_PinRemapConfig(GPIO_Remap_USART2, DISABLE);

		/* Configure USART1 Rx (PA10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_Init( GPIOA, &GPIO_InitStructure );
		
		/* Configure USART1 Tx (PA9) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init( GPIOA, &GPIO_InitStructure );  

		USART_InitStructure.USART_BaudRate = baud;
		USART_InitStructure.USART_WordLength = data_bits;
		USART_InitStructure.USART_StopBits = stop_bits;
		USART_InitStructure.USART_Parity = parity ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init( USART2, &USART_InitStructure );
		
		USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
		
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );
		
		USART_Cmd( USART2, ENABLE );
		xReturn = pdTRUE;
		break;
	case 2:
		USART_DeInit( USART3 );

		/* Enable USART1 clock */
		RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE );	
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE );	

		/* Configure USART1 Rx (PA10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init( GPIOB, &GPIO_InitStructure );
		
		/* Configure USART1 Tx (PA9) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init( GPIOB, &GPIO_InitStructure );

		USART_InitStructure.USART_BaudRate 		= baud;
		USART_InitStructure.USART_WordLength 	= data_bits;
		USART_InitStructure.USART_StopBits 		= stop_bits;
		USART_InitStructure.USART_Parity 		= parity ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init( USART3, &USART_InitStructure );
		
		USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );
		
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );
		
		USART_Cmd( USART3, ENABLE );
		xReturn = pdTRUE;
		break;
	default:
		xReturn = pdFALSE;
		break;
	}	

	return xReturn;
}
/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInit( eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits, unsigned portBASE_TYPE uxBufferLength )
{
	if ( xSerialPortBaseInit(ePort, eWantedBaud, eWantedParity, eWantedDataBits, eWantedStopBits ) == pdTRUE ){
		/* Create the queues used to hold Rx/Tx characters. */
		xPorts[ePort].xRxedChars = xQueueCreate( uxBufferLength, ( unsigned portBASE_TYPE ) sizeof( unsigned portCHAR ) );
		xPorts[ePort].xCharsForTx = xQueueCreate( uxBufferLength + 1, ( unsigned portBASE_TYPE ) sizeof( unsigned portCHAR ) );
		switch ( ePort ){
		case 0:	xPorts[ePort].xUSART = USART1;	break;
		case 1:	xPorts[ePort].xUSART = USART2;	break;
		case 2:	xPorts[ePort].xUSART = USART3;	break;
		default: return ( xComPortHandle ) 0;
		}	
		return (( xComPortHandle )&xPorts[ePort]);
	} else
		return ( xComPortHandle ) 0;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed portCHAR *pcRxedChar, portTickType xBlockTime )
{
	/* The port handle is not required as this driver only supports one port. */
	//( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( pxPort->xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}

signed portBASE_TYPE xSerialGet( xComPortHandle pxPort, unsigned portCHAR *pcBuf, portBASE_TYPE max_size, portTickType xBlockTime )
{
	portBASE_TYPE i = 0;

	while( i < max_size ){
		if( xQueueReceive( pxPort->xRxedChars, pcBuf+i, xBlockTime ) == pdFALSE )
			break;
		i++;
	}
	return i;
}

signed portBASE_TYPE xSerialIsArrive( xComPortHandle pxPort )
{
	signed portCHAR cRxedChar;

	if( xQueuePeek( pxPort->xRxedChars, &cRxedChar, 0 ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}

/*-----------------------------------------------------------*/
void vSerialPut( xComPortHandle pxPort, const unsigned portCHAR * const pcBuf, unsigned portSHORT usLength )
{
signed portSHORT i;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */
	for(i=0;i<usLength;i++)
	{
		xSerialPutChar( pxPort, pcBuf[i], serNO_BLOCK );
	}
}

void vSerialPutString( xComPortHandle pxPort, const unsigned portCHAR * const pcString, unsigned portSHORT usStringLength )
{
signed portCHAR *pxNext;

	/* A couple of parameters that this port does not use. */
	( void ) usStringLength;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	/* Send each character in the string, one at a time. */
	pxNext = ( signed portCHAR * ) pcString;
	while( *pxNext )
	{
		xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
		pxNext++;
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, unsigned portCHAR cOutChar, portTickType xBlockTime )
{
signed portBASE_TYPE xReturn;

	if( xQueueSend( pxPort->xCharsForTx, &cOutChar, xBlockTime ) == pdPASS )
	{
		xReturn = pdPASS;
		USART_ITConfig( pxPort->xUSART, USART_IT_TXE, ENABLE );
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/
#ifdef USE_USART1
void USART1_IRQHandler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
portCHAR cChar;

	if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xPorts[serCOM1].xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* A character was retrieved from the queue so can be sent to the
			THR now. */
			USART_SendData( USART1, cChar );
		}
		else
		{
			USART_ITConfig( USART1, USART_IT_TXE, DISABLE );		
		}		
	}
	
	if( USART_GetITStatus( USART1, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART1 );
		xQueueSendFromISR( xPorts[serCOM1].xRxedChars, &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
#endif 

#ifdef USE_USART2
void USART2_IRQHandler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
portCHAR cChar;

	if( USART_GetITStatus( USART2, USART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xPorts[serCOM2].xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* A character was retrieved from the queue so can be sent to the
			THR now. */
			USART_SendData( USART2, cChar );
		}
		else
		{
			USART_ITConfig( USART2, USART_IT_TXE, DISABLE );		
		}		
	}
	
	if( USART_GetITStatus( USART2, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART2 );
		xQueueSendFromISR( xPorts[serCOM2].xRxedChars, &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
#endif 


#ifdef USE_USART3
void USART3_IRQHandler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
portCHAR cChar;

	if( USART_GetITStatus( USART3, USART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xPorts[serCOM3].xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* A character was retrieved from the queue so can be sent to the
			THR now. */
			USART_SendData( USART3, cChar );
		}
		else
		{
			USART_ITConfig( USART3, USART_IT_TXE, DISABLE );		
		}		
	}
	
	if( USART_GetITStatus( USART3, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART3 );
		xQueueSendFromISR( xPorts[serCOM3].xRxedChars, &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
#endif 







	
