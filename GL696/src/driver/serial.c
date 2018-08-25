
/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

#include "projdefs.h"
/* Demo application includes. */
#include "serial.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE				( ( xQueueHandle ) 0 )
#define serNO_BLOCK						( ( portTickType ) 0 )
#define serTX_BLOCK_TIME				( 40 / portTICK_RATE_MS )

/*-----------------------------------------------------------*/
#define xQueueHandle uint8_t

#define TBUFSIZE	270
#define RBUFSIZE	270

volatile static uint8_t tbuf[3][TBUFSIZE];	//串口数据发送缓冲
volatile static uint8_t rbuf[3][RBUFSIZE];	//串口数据接收缓冲
volatile static uint16_t tbuflen[3];		//发送缓冲区数据长度计数器
volatile static uint16_t rbuflen[3];

volatile static uint16_t t_in[3];			//发送缓冲区队列指针
volatile static uint16_t t_out[3];

volatile static uint16_t r_in[3];			//接收缓冲区队列指针
volatile static uint16_t r_out[3];

volatile static uint8_t tx_complete[3];
/*-----------------------------------------------------------*/

/* UART interrupt handler. */
//void vUARTInterruptHandler( void );

/*-----------------------------------------------------------*/

void RS485_RE(uint8_t port,uint8_t enable)
{
	switch(port){
	case 0:
		if ( enable == pdTRUE )	GPIO_SetBits	( GPIOA, GPIO_Pin_11 );
		else					GPIO_ResetBits	( GPIOA, GPIO_Pin_11 );
		break;
	default:
		break;
	}
}

void RS485_RE_INIT(uint8_t port)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	switch(port){
	case 0:
		/* Enable GPIOD clock */
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );	
		
		/* Configure USART3 RE (PA11) as output*/
		GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_OD;
		GPIO_Init( GPIOA, &GPIO_InitStructure );
		RS485_RE( port, pdTRUE);
		break;
	default:
		break;
	}
}

/*-----------------------------------------------------------*/
signed portBASE_TYPE xQueueReceive( xQueueHandle pxQueue, signed portCHAR * pvBuffer, portTickType xTicksToWait )
{
	(void) xTicksToWait;

	if ( r_out[pxQueue] != r_in[pxQueue] ){
		*pvBuffer = rbuf[pxQueue] [r_out[pxQueue]++];		

		if ( r_out[pxQueue] >= TBUFSIZE )		    
			r_out[pxQueue] = 0;
		return pdTRUE;
	} else
		return pdFALSE;
}

signed portBASE_TYPE xQueueSendFromISR( xQueueHandle pxQueue, unsigned portCHAR * const pvBuffer, signed portBASE_TYPE *pxTaskWoken )
{
	(void) pxTaskWoken;

	rbuf[pxQueue][r_in[pxQueue]++] = *pvBuffer;
	if ( r_in[pxQueue] >= RBUFSIZE )
		r_in[pxQueue] = 0;
	if ( r_in[pxQueue] == r_out[pxQueue] ){
		r_out[pxQueue] ++;
		if ( r_out[pxQueue] >= RBUFSIZE )
			r_out[pxQueue] = 0;
	}
  
	return pdTRUE;
}

signed portBASE_TYPE xQueueSend( xQueueHandle pxQueue, signed portCHAR * const pvItemToQueue, portTickType xTicksToWait )
{
	(void) xTicksToWait;

	tbuf[pxQueue][t_in[pxQueue]++] = *pvItemToQueue;
	if ( t_in[pxQueue] >= RBUFSIZE )
		t_in[pxQueue] = 0;
	if ( t_in[pxQueue] == t_out[pxQueue] ){
		t_out[pxQueue] ++;
		if ( t_out[pxQueue] >= TBUFSIZE )
			t_out[pxQueue] = 0;
	}
	tx_complete[pxQueue] = 0;

/*	if ( pxQueue == 0 )
	USART_ITConfig( USART1, USART_IT_TXE, ENABLE );		
	else if ( pxQueue == 1 ){
		RS485_RE( pxQueue, pdFALSE );
		USART_ITConfig( USART3, USART_IT_TXE|USART_IT_TC, ENABLE );		
	}*/
	return pdTRUE;
}

signed portBASE_TYPE xQueueReceiveFromISR( xQueueHandle pxQueue, unsigned portCHAR * pvBuffer, signed portBASE_TYPE *pxTaskWoken )
{
	(void) pxTaskWoken;

	if ( t_out[pxQueue] != t_in[pxQueue] ){
		*pvBuffer = tbuf[pxQueue] [t_out[pxQueue]++];		

		if ( t_out[pxQueue] >= TBUFSIZE )
			t_out[pxQueue] = 0;
		return pdTRUE;
	} else
		return pdFALSE;
}


signed portBASE_TYPE xQueueSendIsEmpty( xQueueHandle pxQueue )
{
	if ( t_in[pxQueue] != t_out[pxQueue] ){
		return pdTRUE;
	}
	return pdTRUE;
}
/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal(  eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits )
{
	xComPortHandle xReturn = 0;
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
		default :			data_bits = USART_WordLength_8b;	break;
	}

	switch ( eWantedStopBits ){
		case serSTOP_1:		stop_bits = USART_StopBits_1;	break;
		case serSTOP_2:		stop_bits = USART_StopBits_2;	break;
		default :			stop_bits = USART_StopBits_2;	break;
	}

	switch ( ePort ){
	case 0:
		r_in[ePort] 		= r_out[ePort] = 0;
		t_in[ePort] 		= t_out[ePort] = 0;
		tx_complete[ePort] 	= 1;

		USART_DeInit( USART1 );

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
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );

		USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
		USART_Cmd( USART1, ENABLE );
		xReturn = pdTRUE;
		break;
	case 1:
		r_in[ePort] 		= r_out[ePort] = 0;
		t_in[ePort] 		= t_out[ePort] = 0;
		tx_complete[ePort] 	= 1;

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
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );

		USART_Cmd( USART2, ENABLE );
		xReturn = pdTRUE;
		break;
	case 2:
		r_in[ePort] 		= r_out[ePort] = 0;
		t_in[ePort] 		= t_out[ePort] = 0;
		tx_complete[ePort] 	= 1;

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
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
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

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed portCHAR *pcRxedChar, portTickType xBlockTime )
{
	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( pxPort, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed portCHAR * const pcString, unsigned portSHORT usStringLength )
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

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed portCHAR cOutChar, portTickType xBlockTime )
{
signed portBASE_TYPE xReturn;

	if ( pxPort == 0 ){
		USART_ITConfig( USART1, USART_IT_TXE, DISABLE );		
		USART_ITConfig( USART1, USART_IT_TC, 	DISABLE );		
	} else if ( pxPort == 1 ){
		USART_ITConfig( USART3, USART_IT_TXE, DISABLE );		
		USART_ITConfig( USART3, USART_IT_TC, 	DISABLE );		
	}
	
	if( xQueueSend( pxPort, &cOutChar, xBlockTime ) == pdPASS )
	{
		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}
	if ( pxPort == 0 ){
		RS485_RE( pxPort, pdFALSE );
		USART_ITConfig( USART1, USART_IT_TXE, ENABLE );
		USART_ITConfig( USART1, USART_IT_TC, 	ENABLE );		
	} else if ( pxPort == 1 ){
		RS485_RE( pxPort, pdFALSE );
		USART_ITConfig( USART3, USART_IT_TXE, ENABLE );		
		USART_ITConfig( USART3, USART_IT_TC, 	ENABLE );		
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}

void vSerialWaitTx( xComPortHandle xPort )
{
	while( tx_complete[xPort] == 0 ){
		FEED_DOG();
	}
}
/*-----------------------------------------------------------*/

void USART1_IRQHandler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
unsigned portCHAR cChar;

	if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( 0, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
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
	if( USART_GetITStatus( USART1, USART_IT_TC ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( 0, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* A character was retrieved from the queue so can be sent to the
			THR now. */
			USART_SendData( USART1, cChar );
		}
		else
		{
			RS485_RE( 0, pdTRUE);
			USART_ITConfig( USART1, USART_IT_TC, DISABLE );		
			tx_complete[0] = 1;
		}		
	}
		
	if( USART_GetITStatus( USART1, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( USART1 );
		xQueueSendFromISR( 0, &cChar, &xHigherPriorityTaskWoken );
	}	
}


