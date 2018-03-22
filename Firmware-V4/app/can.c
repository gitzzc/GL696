
/* Scheduler include files. */
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x.h"

#include "serials.h"
#include "can.h"

/* copy from stm32f10x_can.c */
//#define CAN_TSR_TME0     ((u32)0x04000000)    /* Transmit mailbox 0 empty */
//#define CAN_TSR_TME1     ((u32)0x08000000)    /* Transmit mailbox 1 empty */
//#define CAN_TSR_TME2     ((u32)0x10000000)    /* Transmit mailbox 2 empty */
/*************************************/



#define CAN_STACK_SIZE				configMINIMAL_STACK_SIZE
#define comTX_LED_OFFSET			( 0 )
#define comRX_LED_OFFSET			( 1 )
#define comTOTAL_PERMISSIBLE_ERRORS ( 2 )

#define uxCANQueueLength	4


#define comINITIAL_RX_COUNT_VALUE	( 0 )


/* Check variable used to ensure no error have occurred.  The Rx task will
increment this variable after every successfully received sequence.  If at any
time the sequence is incorrect the the variable will stop being incremented. */
static volatile unsigned portBASE_TYPE uxCanRxLoops = comINITIAL_RX_COUNT_VALUE;


PDU_STATE pdu_state;
PSU_STATE psu_state[2];

/* The queue used to hold received characters. */
xQueueHandle xCANTxQueue;
xQueueHandle xCANRxQueue;

/*-----------------------------------------------------------*/

portBASE_TYPE xAreCANTasksStillRunning( void )
{
portBASE_TYPE xReturn;

	/* If the count of successful reception loops has not changed than at
	some time an error occurred (i.e. a character was received out of sequence)
	and we will return false. */
	if( uxCanRxLoops == comINITIAL_RX_COUNT_VALUE )
	{
		xReturn = pdFALSE;
	}
	else
	{
		xReturn = pdTRUE;
	}

	/* Reset the count of successful Rx loops.  When this function is called
	again we expect this to have been incremented. */
	uxCanRxLoops = comINITIAL_RX_COUNT_VALUE;

	return xReturn;
}


void CAN_HardwareConfig(void)
{
	GPIO_InitTypeDef  		GPIO_InitStructure;
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
	NVIC_InitTypeDef  		NVIC_InitStructure;

	/* Create the queues used to hold Rx/Tx CanTxMsg. */
	xCANRxQueue = xQueueCreate( uxCANQueueLength, ( unsigned portBASE_TYPE )sizeof( CanRxMsg ) );
	xCANTxQueue = xQueueCreate( uxCANQueueLength, ( unsigned portBASE_TYPE )sizeof( CanTxMsg ) );

	/* CAN Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/* Configure CAN RX pin */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure CAN TX pin */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIO_PinRemapConfig(GPIO_Remap2_CAN , ENABLE);
	AFIO->MAPR &= ~((u32)0x03<<13);
	AFIO->MAPR |=  ((u32)0x02<<13);
	
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=DISABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
	CAN_InitStructure.CAN_Prescaler=9;//250kbps
	CAN_Init(CAN1,&CAN_InitStructure);
	
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN_TX_IRQChannel;
	//NVIC_Init(&NVIC_InitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
}




signed portBASE_TYPE xCANSetPSU( u8 channle,u16 voltage )
{
	CanTxMsg TxMessage;
	u32 msg_id = (PDC_TO_PSU1_ID&0xFFFF00FF)|((PSU1_ADDR+channle)<<8);
	
	/* 设置电源输出电压 */
	TxMessage.StdId	= msg_id>>18;
	TxMessage.ExtId	= msg_id;
	TxMessage.RTR	= CAN_RTR_DATA;
	TxMessage.IDE	= CAN_ID_EXT;
	TxMessage.DLC	= 8;
	TxMessage.Data[0] = PDC_FUN_SET;
	TxMessage.Data[1] = 0x00;
	TxMessage.Data[2] = 0x00;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = voltage;
	TxMessage.Data[7] = voltage>>8;
	
	return xCANPutMsg( &TxMessage, 500/portTICK_RATE_MS );
}

signed portBASE_TYPE xCANSetPDU( u16 state )
{
	CanTxMsg TxMessage;
	
	/* 设置配电器输出状态 */
	TxMessage.StdId	= PDC_TO_PDU_ID>>18;
	TxMessage.ExtId	= PDC_TO_PDU_ID;
	TxMessage.RTR	= CAN_RTR_DATA;
	TxMessage.IDE	= CAN_ID_EXT;
	TxMessage.DLC	= 8;
	TxMessage.Data[0] = PDC_FUN_SET;
	TxMessage.Data[1] = 0x00;
	TxMessage.Data[2] = 0x00;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = state;
	TxMessage.Data[7] = state>>8;
	
	return xCANPutMsg( &TxMessage, 500/portTICK_RATE_MS );
}

signed portBASE_TYPE xCANCheckMessage( portTickType ms )
{
	CanTxMsg rx;
	portBASE_TYPE ret = FALSE;
	portBASE_TYPE id,time=0;
	u8 	frame_id,channel;
	
	while(1){
		if ( xCANGetMsg( &rx, ms ) == TRUE ) {
			ret	= TRUE;
			id = (rx.StdId << 18) | rx.ExtId;
			switch ( id ){
			case PDU_TO_PDC_ID:
				frame_id = rx.Data[0];
				switch( frame_id ){
				case PDC_FRAME_ID_CH1:
					vPortEnterCritical();
					pdu_state.input_cur[0] = rx.Data[1] | (rx.Data[2]<<8);
					pdu_state.input_vol[0] = rx.Data[3] | (rx.Data[4]<<8);
					pdu_state.output_state[0] = rx.Data[6] | (rx.Data[6]<<8);
					vPortExitCritical();
					break;
				case PDC_FRAME_ID_CH2:
					vPortEnterCritical();
					pdu_state.input_cur[1] = rx.Data[1] | (rx.Data[2]<<8);
					pdu_state.input_vol[1] = rx.Data[3] | (rx.Data[4]<<8);
					vPortExitCritical();
					break;
				default:
					break;				
				}
				break;
			case PSU1_TO_PDC_ID:
			case PSU2_TO_PDC_ID:
				channel = (id&0xFF) - PSU1_ADDR;
				frame_id = rx.Data[0];
				switch( frame_id ){
				case 0:
					vPortEnterCritical();
					psu_state[channel].current = rx.Data[1] | (rx.Data[2]<<8);
					psu_state[channel].voltage = rx.Data[3] | (rx.Data[4]<<8);
					psu_state[channel].ctrl_state= rx.Data[5];
					psu_state[channel].over_cur = rx.Data[6];
					psu_state[channel].over_vol = rx.Data[7];
					vPortExitCritical();
					break;
				default:
					break;
				}
			default:
				break;
			}
		}
		if ( time ++ > ms )
			break;
	}
	return ret;
}


/*-----------------------------------------------------------*/
portTASK_FUNCTION( vCANMonitorTask, pvParameters )
{
	CanTxMsg tx,rx;
	portBASE_TYPE i,j;
	portBASE_TYPE send_id,recv_id;
#define MAX_CHECK	3
//#define REPONSE_TIME_OUT	200
#define REPONSE_TIME_OUT	1000

	for( ;; ){
                                		vTaskDelay( 1000/portTICK_RATE_MS );

/*		// 查询配电器数据 第一路数据
		tx.StdId	= PDC_TO_PDU_ID>>18;
		tx.ExtId	= PDC_TO_PDU_ID;
		tx.RTR		= CAN_RTR_REMOTE;
		tx.IDE		= CAN_ID_EXT;
		tx.DLC		= 8;
		tx.Data[0] 	= PDC_FUN_REQUEST;
		tx.Data[1] 	= PDC_FRAME_ID_CH1;
		for(i=2;i<8;i++)
		tx.Data[i] 	= 0x00;
		xCANPutMsg(	&tx, 10/portTICK_RATE_MS );
		xCANCheckMessage(10);
  
		// 查询配电器数据 第二路数据
		tx.Data[1] = PDC_FRAME_ID_CH2;
		xCANPutMsg( &tx, 10/portTICK_RATE_MS );
		xCANCheckMessage(10);
  
		// 查询电源数据 
		tx.StdId	= PDC_TO_PSU1_ID>>18;
		tx.ExtId	= PDC_TO_PSU1_ID;
		tx.RTR	= CAN_RTR_REMOTE;
		tx.IDE	= CAN_ID_EXT;
		tx.DLC	= 8;
		tx.Data[0] = PDC_FUN_REQUEST;
		for(i=1;i<8;i++)
		tx.Data[i] 	= 0x00;
		xCANPutMsg( &tx, 10/portTICK_RATE_MS );
		xCANCheckMessage(10);
		
		// 查询电源数据 
		tx.StdId	= PDC_TO_PSU2_ID>>18;
		tx.ExtId	= PDC_TO_PSU2_ID;
		xCANPutMsg( &tx, 10/portTICK_RATE_MS );
		xCANCheckMessage(10);
	
*/
		// 查询配电器数据 
		for(j=0;j<2;j++){	
			tx.StdId	= PDC_TO_PDU_ID>>18;
			tx.ExtId	= PDC_TO_PDU_ID;
			tx.RTR		= CAN_RTR_DATA;
			tx.IDE		= CAN_ID_EXT;
			tx.DLC		= 8;
			tx.Data[0] 	= PDC_FUN_REQUEST;
			tx.Data[1] 	= PDC_FRAME_ID_CH1+j;
			for(i=2;i<8;i++) tx.Data[i]	= 0x00;
			
			for(i=0;i<MAX_CHECK;i++){
				xCANPutMsg(	&tx, REPONSE_TIME_OUT/portTICK_RATE_MS );
				if ( xCANGetMsg( &rx, REPONSE_TIME_OUT/portTICK_RATE_MS ) == TRUE ){ //100ms
					if ( PDU_TO_PDC_ID != ((rx.StdId << 18) | rx.ExtId) || j != rx.Data[0] )
						continue;
					
					vPortEnterCritical();
					pdu_state.input_cur[j] = rx.Data[1] | (rx.Data[2]<<8);
					pdu_state.input_vol[j] = rx.Data[3] | (rx.Data[4]<<8);
					if ( j == 0 )
						pdu_state.output_state[0] = rx.Data[6] | (rx.Data[7]<<8);
					vPortExitCritical();
					break;
				}
			}
			if ( i == MAX_CHECK ){
				vPortEnterCritical();
				pdu_state.input_cur[j] = 0;
				pdu_state.input_vol[j] = 0;
				pdu_state.output_state[0] = 0;
				vPortExitCritical();
			}
		}
  
		// 查询电源数据 
		for(j=0;j<MAX_PSU_CHANNEL;j++){
			switch(j){
				case 0:
					send_id = PDC_TO_PSU1_ID;
					recv_id = PSU1_TO_PDC_ID;
					break;
				case 1:
					send_id = PDC_TO_PSU2_ID;
					recv_id = PSU2_TO_PDC_ID;
					break;
				default:
					continue;
			}
			tx.StdId	= send_id>>18;
			tx.ExtId	= send_id;
			tx.RTR		= CAN_RTR_DATA;
			tx.IDE		= CAN_ID_EXT;
			tx.DLC		= 8;
			tx.Data[0] 	= PDC_FUN_REQUEST;
			for(i=1;i<8;i++) tx.Data[i] = 0x00;
			
			for(i=0;i<MAX_CHECK;i++){
				xCANPutMsg(	&tx, REPONSE_TIME_OUT/portTICK_RATE_MS );
				if ( xCANGetMsg( &rx, REPONSE_TIME_OUT/portTICK_RATE_MS ) == TRUE ){ //100ms
					if ( recv_id != ((rx.StdId << 18) | rx.ExtId) )
						continue;
					
					vPortEnterCritical();
					psu_state[j].current 	= rx.Data[1] | (rx.Data[2]<<8);
					psu_state[j].voltage 	= rx.Data[3] | (rx.Data[4]<<8);
					psu_state[j].ctrl_state	= rx.Data[5];
					psu_state[j].over_cur 	= rx.Data[6];
					psu_state[j].over_vol 	= rx.Data[7];
					vPortExitCritical();
					break;
				}
			}
			if ( i == MAX_CHECK ){
				vPortEnterCritical();
				psu_state[j].current 	= 0;
				psu_state[j].voltage 	= 0;
				psu_state[j].ctrl_state	= 0;
				psu_state[j].over_cur 	= 0;
				psu_state[j].over_vol 	= 0;
				vPortExitCritical();
			} 
		}
	}
}

portTASK_FUNCTION( vCANTxTask, pvParameters )
{
	CanTxMsg tx;
	portBASE_TYPE i;
	
	for( ;; ){
		if( xQueueReceive( xCANTxQueue, &tx, portMAX_DELAY ) == pdTRUE )
		{
			for(i=0;i<100;i++){
				if ( CAN_Transmit(CAN1,&tx) != CAN_NO_MB	)
					break;
				vTaskDelay( 10/portTICK_RATE_MS );
			}
		}
	} 
}

void vStartCANTasks( unsigned portBASE_TYPE uxPriority )
{
	CAN_HardwareConfig();

	/* The Tx task is spawned with a lower priority than the Rx task. */
	xTaskCreate( vCANMonitorTask, ( signed char * ) "CANMonitor", CAN_STACK_SIZE, NULL, uxPriority - 1, ( xTaskHandle * ) NULL );
	xTaskCreate( vCANTxTask, ( signed char * ) "CANTx", CAN_STACK_SIZE, NULL, uxPriority - 1, ( xTaskHandle * ) NULL );
}
  
  
signed portBASE_TYPE xCANPutMsg( CanTxMsg *tx, portTickType xBlockTime )
{
signed portBASE_TYPE xReturn;

	if( xQueueSend( xCANTxQueue, tx, xBlockTime ) == pdPASS )
	{
		//CAN_ITConfig(CAN_IT_TME, ENABLE);
		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}


signed portBASE_TYPE xCANGetMsg( CanTxMsg *rx, portTickType xBlockTime )
{
	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xCANRxQueue, rx, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}

/*-----------------------------------------------------------*/

void xCANClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}

/**
  * @brief  This function handles CAN1 Handler.
  * @param  None
  * @retval None
  */
void USB_HP_CAN1_TX_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	portBASE_TYPE i;
	CanTxMsg tx;
	 
	if( CAN_GetITStatus(CAN1, CAN_IT_TME ) == SET )
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		for(i=0;i<3;i++){
			/* Select one empty transmit mailbox */
		  	if ((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0)
				break;
			
			if( xQueueReceiveFromISR( xCANTxQueue, &tx, &xHigherPriorityTaskWoken ) == pdTRUE )
				CAN_Transmit(CAN1,&tx);
			else
				 break;
		}
		if ( i == 0 )
			CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE );
	}
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/**
  * @brief  This function handles CAN1 Handler.
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	CanRxMsg RxMessage;
	
	RxMessage.StdId=0x00;
	RxMessage.ExtId=0x00;
	RxMessage.IDE=0;
	RxMessage.DLC=0;
	RxMessage.FMI=0;
	RxMessage.Data[0]=0x00;
	RxMessage.Data[1]=0x00;
	RxMessage.Data[2]=0x00;
	RxMessage.Data[3]=0x00;
	RxMessage.Data[4]=0x00;
	RxMessage.Data[5]=0x00;
	RxMessage.Data[6]=0x00;
	RxMessage.Data[7]=0x00;
	
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

	// We have not woken a task at the start of the ISR.
	xHigherPriorityTaskWoken = pdFALSE;

	// Post RxMessage.
	xQueueSendFromISR( xCANRxQueue, &RxMessage, &xHigherPriorityTaskWoken);

	// Now the buffer is empty we can switch context if necessary.  Note that the
	// name of the yield function required is port specific.
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}

