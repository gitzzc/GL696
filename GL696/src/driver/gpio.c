/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "projdefs.h"
#include "gpio.h"
#include "timer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//-------------------------------------------------------------------------
static const sDIO_PIN dio_pins[] = 
{
	{RELAY0,	DIO_OUT,GPIO_Pin_0	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY1,	DIO_OUT,GPIO_Pin_1	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY2,	DIO_OUT,GPIO_Pin_2	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY3,	DIO_OUT,GPIO_Pin_3	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY4,	DIO_OUT,GPIO_Pin_4	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY5,	DIO_OUT,GPIO_Pin_5	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY6,	DIO_OUT,GPIO_Pin_6	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY7,	DIO_OUT,GPIO_Pin_7	,GPIOD,GPIO_Mode_Out_OD,1},
	
	{RELAY8,	DIO_OUT,GPIO_Pin_12	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY9,	DIO_OUT,GPIO_Pin_13	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY10,	DIO_OUT,GPIO_Pin_14	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY11,	DIO_OUT,GPIO_Pin_15	,GPIOD,GPIO_Mode_Out_OD,1},
	{RELAY12,	DIO_OUT,GPIO_Pin_6	,GPIOB,GPIO_Mode_Out_OD,1},
	{RELAY13,	DIO_OUT,GPIO_Pin_7	,GPIOB,GPIO_Mode_Out_OD,1},
	{RELAY14,	DIO_OUT,GPIO_Pin_12	,GPIOC,GPIO_Mode_Out_OD,1},
	{RELAY15,	DIO_OUT,GPIO_Pin_13	,GPIOC,GPIO_Mode_Out_OD,1},

	{PWR_0,		DIO_OUT,GPIO_Pin_4	,GPIOA,GPIO_Mode_Out_OD,1},
	{PWR_1,		DIO_OUT,GPIO_Pin_5	,GPIOA,GPIO_Mode_Out_OD,1},
	{PWR_2,		DIO_OUT,GPIO_Pin_6	,GPIOA,GPIO_Mode_Out_OD,1},
	{PWR_3,		DIO_OUT,GPIO_Pin_7	,GPIOA,GPIO_Mode_Out_OD,1},
}; 

sBIT_FILTER bitFilter[] = 
{
};

//-------------------------------------------------------------------------
void DIO_SetMode(ePIN_NAME pin, DIO_MODE_TYPE type)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin 	= dio_pins[pin].pin;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= type;
	GPIO_Init(dio_pins[pin].reg, &GPIO_InitStructure);
}

//-------------------------------------------------------------------------
void DIO_Write(ePIN_NAME pin, uint8_t hi_low)
{
	if ( (uint32_t)pin > sizeof(dio_pins)/sizeof(dio_pins[0]) )
		return;
	
	if ( dio_pins[pin].type == DIO_IN )
		return;
		
	if ( hi_low == pdLOW )
		GPIO_ResetBits(dio_pins[pin].reg,dio_pins[pin].pin);
	else
		GPIO_SetBits	(dio_pins[pin].reg,dio_pins[pin].pin);
	DIO_SetMode(pin,dio_pins[pin].mode);
}

//-------------------------------------------------------------------------
uint8_t DIO_Read(ePIN_NAME pin)
{
	if ( (uint32_t)pin > sizeof(dio_pins)/sizeof(dio_pins[0]) )
		return 0;

	if ( dio_pins[pin].type == DIO_IN )
		return GPIO_ReadInputDataBit(dio_pins[pin].reg,dio_pins[pin].pin);
	else
		return GPIO_ReadOutputDataBit(dio_pins[pin].reg,dio_pins[pin].pin);
}

//-------------------------------------------------------------------------
void DIO_Init()
{
	uint32_t i;

	/* Enable GPIOx clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);

	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	for(i=0;i<sizeof(dio_pins)/sizeof(sDIO_PIN);i++){
		if ( dio_pins[i].type == DIO_OUT ){
			DIO_Write((ePIN_NAME)i,dio_pins[i].level);
		}
		DIO_SetMode((ePIN_NAME)i,dio_pins[i].mode);
	}
	DO_DelayInit();
}

uint32_t DIO_MonitorTask(psBIT_FILTER filter,uint8_t* bit_buf,uint32_t bit_num,uint32_t* alarm_num)
{
	uint8_t i,bit;
	uint32_t new = 0;

	for(i=0;i<bit_num;i++){
		bit = (bit_buf[i/8] >> (i%8)) & 0x01;
		if ( filter[i].level == bit ){
			if ( filter[i].count++ >= filter[i].filter ){
				filter[i].bit 		= bit;
				if ( filter[i].bit == filter[i].alarm_type ){
					if ( filter[i].alarm == 0 )
						new ++;
					filter[i].alarm = 1;
					alarm_num ++;
				} else
					filter[i].alarm = 0;
			}
		} else {
			filter[i].bit 	|= DIO_CHANGING;
			filter[i].count	 = 0;
		}
		filter[i].level = bit;
	}
	return new;
}


//------------------------------------------------------------
#define DO_COUNT 32
static uint8_t	DO_Status[DO_COUNT/8];
static uint32_t	DO_DelayCount[DO_COUNT];

void DO_WriteDelay(ePIN_NAME pin, uint8_t hi_low, uint32_t delay_invert)
{
	DIO_Write(pin, hi_low);

	if ( hi_low == pdHIGH ){
		DO_Status[pin/8] |=   (1<<(pin%8)); 
	} else {
		DO_Status[pin/8] &= (~(1<<(pin%8))); 
	}
	DO_DelayCount[pin] = delay_invert;
}

void DO_DelayInit(void)
{
	memset(DO_Status,0xFF,sizeof(DO_Status));
	memset(DO_DelayCount,0,sizeof(DO_DelayCount)*sizeof(uint32_t));
}

void DO_DelayTask( uint32_t tick )
{
	uint32_t i;
	
	for(i=0;i<DO_COUNT;i++){
		if ( DO_DelayCount[i] ){
			if ( DO_DelayCount[i] <= tick ){
				DO_WriteDelay((ePIN_NAME)i,!((DO_Status[i/8] >> (i%8))&0x01),0);
				DO_DelayCount[i] = 0;
			} else
				DO_DelayCount[i] -= tick;
		}
	}
}

