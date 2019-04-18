
/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/
#include "stm32f1xx_hal.h"
#include "main.h"
#include "serial.h"


/*-----------------------------------------------------------*/
sSerialQueue serQueue[MAXPORT];
uint8_t rx_buf[MAXPORT][1];


/*-----------------------------------------------------------*/

void RS485_RE(UART_HandleTypeDef *UartHandle,uint8_t enable)
{
	if ( UartHandle->Instance==USART1){
//		if ( enable == pdTRUE )	HAL_GPIO_WritePin (RS485_RE_PORT,RS485_RE_PIN, GPIO_PIN_RESET );
//		else					HAL_GPIO_WritePin (RS485_RE_PORT,RS485_RE_PIN, GPIO_PIN_SET );
	} else if ( UartHandle->Instance==USART2 ){
	}
}

void RS485_RE_INIT(UART_HandleTypeDef *UartHandle)
{
	if ( UartHandle->Instance==USART1){
	}
}

/*-----------------------------------------------------------*/
uint8_t SerialDequeue( psSerialQueue pQueue, uint8_t* dat )
{
	if ( pQueue->rout == pQueue->rin )
		return pdFALSE;

	*dat = pQueue->rbuf[pQueue->rout++];

	if ( pQueue->rout >= TBUFSIZE )
		pQueue->rout = 0;
	return pdTRUE;
}

uint8_t SerialEnqueue( psSerialQueue pQueue, uint8_t* dat )
{
	pQueue->rbuf[pQueue->rin++] = *dat;
	if ( pQueue->rin >= RBUFSIZE )
		pQueue->rin = 0;
	if ( pQueue->rin == pQueue->rout ){
		pQueue->rout ++;
		if ( pQueue->rout >= TBUFSIZE )
			pQueue->rout = 0;
	}

	return pdTRUE;
}

uint8_t xQueueSendIsEmpty( psSerialQueue pQueue )
{
	if ( pQueue->tin != pQueue->tout ){
		return pdTRUE;
	}
	return pdTRUE;
}

uint8_t SerialBufInit( UART_HandleTypeDef *UartHandle )
{
	uint32_t usart;

	if 		( UartHandle->Instance == USART1 )	usart = 0;
	else if ( UartHandle->Instance == USART2 )	usart = 1;
	else if ( UartHandle->Instance == USART3 )	usart = 2;
	else	return pdFALSE;

	serQueue[usart].rin 	= 0;
	serQueue[usart].rout 	= 0;
	serQueue[usart].tin 	= 0;
	serQueue[usart].tout 	= 0;

	if(HAL_UART_Receive_IT(UartHandle, rx_buf[usart], 1) != HAL_OK) Error_Handler();

	return 0;
}

uint8_t SerialGetChar( UART_HandleTypeDef *UartHandle, uint8_t *pcRxedChar )
{
	uint32_t usart;

	if 		( UartHandle->Instance == USART1 )	usart = 0;
	else if ( UartHandle->Instance == USART2 )	usart = 1;
	else if ( UartHandle->Instance == USART3 )	usart = 2;
	else	return pdFALSE;

	return SerialDequeue(&serQueue[usart],pcRxedChar);
}

uint8_t SerialPutBuf(UART_HandleTypeDef *UartHandle,uint8_t* buf,uint16_t len)
{
	while( 1 ){
		if ( UartHandle->gState == HAL_UART_STATE_READY )
			break;
	}

	RS485_RE(UartHandle,0);
	HAL_Delay(2);

	return HAL_UART_Transmit_IT(UartHandle, buf, len);
}

/*-----------------------------------------------------------*/
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Set transmission flag: transfer complete */

	if ( UartHandle->gState == HAL_UART_STATE_READY )
		RS485_RE(UartHandle,1);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	uint8_t usart=0;

	if 		  ( UartHandle->Instance==USART1){	usart = 0;
	} else if ( UartHandle->Instance==USART2){	usart = 1;
	} else if ( UartHandle->Instance==USART3){	usart = 2;
	} else return;

	SerialEnqueue(&serQueue[usart],rx_buf[usart]);
	if(HAL_UART_Receive_IT(UartHandle, rx_buf[usart], 1) != HAL_OK)	Error_Handler();
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	uint8_t usart=0;

	if 		  ( UartHandle->Instance==USART1){	usart = 0;
	} else if ( UartHandle->Instance==USART2){	usart = 1;
	} else if ( UartHandle->Instance==USART3){	usart = 2;
	} else return ;

	HAL_UART_Receive_IT(UartHandle, rx_buf[usart], 1);
}


