/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x.h"

#include "Calibrate.h"
#include "touchscreen.h"
#include "LCD_Message.h"
#include "lcd.h"
#include "window.h"
#include "iic_eeprom.h"
#include "spi.h"
#include "spi_flash.h"


/*-----------------------------------------------------------*/
#define TC_MAX_VALUE		((1<<12)-1)

#define PEN_DOWN_DELAY		10
#define PEN_SAMPLE_DELAY	100

#define AD7843_PORT			GPIOB
#define AD7843_CS			GPIO_Pin_9	//PB.09	
#define AD7843_PENIRQ		GPIO_Pin_8	//PB.08	
//#define AD7843_BUSY		GPIO_Pin_13	//PB.13	

#define AD7843_CS_EN()		GPIO_WriteBit(GPIOB, AD7843_CS, Bit_RESET)
#define AD7843_CS_DIS()		GPIO_WriteBit(GPIOB, AD7843_CS, Bit_SET)

#define PAN_INT_LINE 		EXTI_Line8


#define TC_SPI_SEND(byte) (u8)SPI_Send(SPI2,byte)

/*-----------------------------------------------------------*/

xTaskHandle 	xTCTaskHandle;

extern xQueueHandle xMSGQueue;
static MATRIX 		TC_Matrix;

/*-----------------------------------------------------------*/

void AD7843_Config_SPI(void)
{
	SPI_InitTypeDef    SPI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	/* Enable GPIOB clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	
	/* Configure SPI2 pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI1 disable */
	SPI_Cmd(SPI2, DISABLE);
	
	/* Enable SPI1 clock  */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/* SPI1 Config */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode 		= SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize 	= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 		= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 		= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 		= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit 	= SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	
	/* SPI1 enable */
	SPI_Cmd(SPI2, ENABLE);
}

void AD7843_Config_INT()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure PENIRQ (PB.04) in Input mode */
	GPIO_InitStructure.GPIO_Pin 	= AD7843_PENIRQ;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Connect EXTI Line to GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
	
	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line	= PAN_INT_LINE;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure); 
}

void AD7843_Enable_INT()
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line	= PAN_INT_LINE;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void AD7843_Disable_INT()
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line	= PAN_INT_LINE;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void AD7843_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure CS in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin 	= AD7843_CS;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(AD7843_PORT, &GPIO_InitStructure);

	AD7843_CS_DIS();
	AD7843_Config_INT();
	AD7843_Config_SPI();
}

u16 AD7843_Read(u8 cmd)
{
	vu32 i;
	u16 ret;
	
	SPI_Take( SPI2, portMAX_DELAY );	
	
	AD7843_Config_SPI();
	
	AD7843_CS_EN();
	for(i=0;i<10;i++);
	
	TC_SPI_SEND (cmd);
	for(i=0;i<100;i++);
	//TC_SPI_SEND (0x00);
	ret = TC_SPI_SEND (0x00);
	
	//for(i=0;i<10;i++);
	//TC_SPI_SEND (0x00);	
	ret = TC_SPI_SEND (0x00) | (ret << 8);
	
	AD7843_CS_DIS();

	SPI_Give( SPI2 );	

	return ret;
}


void TCGetXY(POINT* tc)
{
	u16 tmp,max_x=0,min_x=0,max_y=0,min_y=0;
	u32 x=0,y=0,i;
	
	for(i=0;i<16;i++){
		//NO power-down		
		tmp = (AD7843_Read(0x83|(0x05<<4))>>3>>2) & 0x3FF;
		if ( i > 6 ){
			if ( tmp > max_x )
				max_x = tmp;
			else if ( tmp < min_x )
				min_x = tmp;
			x += tmp;
		}
	}		
	
	for(i=0;i<26;i++){
		tmp = (AD7843_Read(0x83|(0x01<<4))>>3>>2) & 0x3FF;
		if ( i > 16 ){
			if ( tmp > max_y )
				max_y = tmp;
			else if ( tmp < min_y )
				min_y = tmp;
			y += tmp;
		}
	}
	//enter power-down
	AD7843_Read(0x80|(0x01<<4)) ;
	
	tc->x = (x-max_x-min_x)>>3;
	tc->y = (y-max_y-min_y)>>3;
}


portBASE_TYPE TouchScreen_Calibrate(portBASE_TYPE cmd)
{
	portBASE_TYPE i;
	POINT Display[3] = {LCD_SCR_WIDTH*15/100,LCD_SCR_HIGH*15/100,
						LCD_SCR_WIDTH*50/100,LCD_SCR_HIGH*85/100,
						LCD_SCR_WIDTH*85/100,LCD_SCR_HIGH*50/100};
	POINT TC[3];
	HIDMessage msg;
	portCHAR ret=pdTRUE;
	uint8_t str[32];

	if ( cmd != 1 ) {
		SPI_FLASH_BufferRead(str, TC_CFG_FLAG_ADDR, 2);
		if ( (str[0] | (str[1]<<8)) == TC_CFG_FLAG ){
			SPI_FLASH_BufferRead((uint8_t*)&TC_Matrix, TC_CFG_FLAG_ADDR+2, sizeof(TC_Matrix));
			return pdFAIL;
		}
	}

	vPortEnterCritical();
	memset((uint8_t*)&TC_Matrix,0,sizeof(TC_Matrix));
	vPortExitCritical();
	
	LCD_Clear();
	LCD_SetFont_EN(&EN_Font16x32);
	LCD_SetFont_CH(&CH_Font32x32);
	
	while(1){
		if ( xQueueReceive( xMSGQueue, &msg, 0 ) == pdFAIL )
			break;
	}

	while ( 1 ){
		LCD_SetCursor(0,LCD_SCR_HIGH/2-32);
		LCD_DisplayString("         请点击屏幕十字线的中心        \r\n");
		LCD_SetCursor(0,LCD_SCR_HIGH/2);
		LCD_DisplayString("            对触摸屏进行校准           \r\n");
	
		for(i=0;i<3;){
			LCD_SetCursor( Display[i].x-16/2, Display[i].y-32/2 );
			LCD_DisplayString("+");	
			if ( xQueueReceive( xMSGQueue, &msg, 60*configTICK_RATE_HZ ) == pdFAIL ){
				ret = pdFAIL;
				break;
			} else {
				if ( msg.type == HID_TOUCHSCREEN &&  msg.id == HID_TC_DOWN || msg.id == HID_TC_FLEETING ){
					LCD_SetCursor( 0,0 );
					sprintf(str,"tc x:%4d,y:%4d,id:%2d\r\n",msg.raw_x,msg.raw_y,msg.id);
					LCD_DisplayString(str);
				} else if ( msg.id == HID_TC_UP ) {
					TC[i].x = msg.raw_x;
					TC[i].y = msg.raw_y;
					LCD_SetCursor( Display[i].x-16/2, Display[i].y-32/2 );
					LCD_DisplayString(" ");	
					i++;
				}
			}
		}
		if ( ret == pdFAIL )
			break;
		
		if ( setCalibrationMatrix( &Display[0], &TC[0], &TC_Matrix ) == OK ){
			ret = pdTRUE;
			//保存参数
			str[0] = TC_CFG_FLAG&0xFF;
			str[1] = TC_CFG_FLAG>>8;
			SPI_FLASH_SectorErase(0);
			SPI_FLASH_BufferWrite((uint8_t*)str, TC_CFG_FLAG_ADDR, 2);
			SPI_FLASH_BufferWrite((uint8_t*)&TC_Matrix, TC_CFG_FLAG_ADDR+2, sizeof(TC_Matrix));
			break;
		}
		LCD_SetCursor(0,LCD_SCR_HIGH/2+32);
		LCD_DisplayString("             请再校准一次");
	}
	return ret;
}

void vTouchTask( void *pvParameters )
{
	portBASE_TYPE tc_delay = PEN_DOWN_DELAY;
	portBASE_TYPE pen_down_time = 0;
	HIDMessage msg;
	POINT	tc,disp;
	
	AD7843_Init();
	AD7843_Read(0x01);//setup PD0-1 
	
	for( ;; )
	{
		vTaskDelay( tc_delay / portTICK_RATE_MS );
		if ( !(GPIOB->IDR & AD7843_PENIRQ) ){
			vTaskDelay( portTICK_RATE_MS );
			if ( GPIOB->IDR & AD7843_PENIRQ )
				continue;
				
			AD7843_Init();
			TCGetXY(&tc);
			vPortEnterCritical();
			getDisplayPoint(&disp,&tc,&TC_Matrix);
			vPortExitCritical();
			if ( disp.x > LCD_SCR_WIDTH )
				disp.x = LCD_SCR_WIDTH;
			else if ( disp.x < 0 )
				disp.x = 0;
			if ( disp.y > LCD_SCR_HIGH )
				disp.y = LCD_SCR_HIGH;
			else if ( disp.y < 0 )
				disp.y = 0;
				
			msg.type= HID_TOUCHSCREEN;
			if ( msg.id == HID_TC_DOWN )
				msg.id 	= HID_TC_FLEETING;
			else if ( msg.id == HID_TC_UP )
				msg.id = HID_TC_DOWN;
			msg.x 		= disp.x;
			msg.y 		= disp.y;
			msg.raw_x 	= tc.x;
			msg.raw_y 	= tc.y;
					
			if ( pen_down_time ++ > (configTICK_RATE_HZ*20) / PEN_SAMPLE_DELAY ){
				msg.id = HID_TC_CALIBRATE;
			}
			Win_PutMsg(&msg);
			
			tc_delay = PEN_SAMPLE_DELAY;
		} else if ( pen_down_time ) {
			vTaskDelay( portTICK_RATE_MS );
			if ( !(GPIOB->IDR & AD7843_PENIRQ) )
				continue;
				
			pen_down_time 	= 0;
			msg.type	 	= HID_TOUCHSCREEN;
			msg.id 			= HID_TC_UP;
			Win_PutMsg(&msg);

            AD7843_Enable_INT();
			tc_delay = PEN_DOWN_DELAY;
			vTaskSuspend(NULL);
		}
	}
}


/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
 	if (EXTI_GetITStatus(PAN_INT_LINE ) != RESET)
	{
		AD7843_Disable_INT();
		
		xTaskResumeFromISR(xTCTaskHandle);
	}
    EXTI_ClearITPendingBit(PAN_INT_LINE);
}


