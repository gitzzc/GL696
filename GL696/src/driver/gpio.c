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
	{DO0		,DIO_OUT,GPIO_Pin_3 ,GPIOB,GPIO_Mode_Out_OD,1},
	{DO1 		,DIO_OUT,GPIO_Pin_4 ,GPIOB,GPIO_Mode_Out_OD,1},
	{DO2 		,DIO_OUT,GPIO_Pin_5 ,GPIOB,GPIO_Mode_Out_OD,1},
	{DO3	 	,DIO_OUT,GPIO_Pin_6 ,GPIOB,GPIO_Mode_Out_OD,1},
	{DO4		,DIO_OUT,GPIO_Pin_7	,GPIOB,GPIO_Mode_Out_OD,1},
	{DO5		,DIO_OUT,GPIO_Pin_0	,GPIOE,GPIO_Mode_Out_OD,1},
	{DO6		,DIO_OUT,GPIO_Pin_1	,GPIOE,GPIO_Mode_Out_OD,1},
	{DO7		,DIO_OUT,GPIO_Pin_2	,GPIOE,GPIO_Mode_Out_OD,1},
	
	{DO8		,DIO_OUT,GPIO_Pin_3 ,GPIOE,GPIO_Mode_Out_OD,1},
	{DO9		,DIO_OUT,GPIO_Pin_4 ,GPIOE,GPIO_Mode_Out_OD,1},
	{DO10 		,DIO_OUT,GPIO_Pin_5 ,GPIOE,GPIO_Mode_Out_OD,1},
	{DO11		,DIO_OUT,GPIO_Pin_6 ,GPIOE,GPIO_Mode_Out_OD,1},
	{DO12		,DIO_OUT,GPIO_Pin_7	,GPIOE,GPIO_Mode_Out_OD,1},
	{DO13		,DIO_OUT,GPIO_Pin_8	,GPIOE,GPIO_Mode_Out_OD,1},
	{DO14		,DIO_OUT,GPIO_Pin_9	,GPIOE,GPIO_Mode_Out_OD,1},
	{DO15		,DIO_OUT,GPIO_Pin_10,GPIOE,GPIO_Mode_Out_OD,1},

	{DO16		,DIO_OUT,GPIO_Pin_11,GPIOE,GPIO_Mode_Out_OD,1},
	{DO17		,DIO_OUT,GPIO_Pin_12,GPIOE,GPIO_Mode_Out_OD,1},
	{DO18		,DIO_OUT,GPIO_Pin_13,GPIOE,GPIO_Mode_Out_OD,1},
	{DO19		,DIO_OUT,GPIO_Pin_14,GPIOE,GPIO_Mode_Out_OD,1},
	{DO20		,DIO_OUT,GPIO_Pin_15,GPIOE,GPIO_Mode_Out_OD,1},

	{DO_ADSEL0	,DIO_OUT,GPIO_Pin_12,GPIOD,GPIO_Mode_Out_PP,0},
	{DO_ADSEL1	,DIO_OUT,GPIO_Pin_13,GPIOD,GPIO_Mode_Out_PP,0},
	{DO_ADSEL2	,DIO_OUT,GPIO_Pin_14,GPIOD,GPIO_Mode_Out_PP,0},
	{DO_DISEL0	,DIO_OUT,GPIO_Pin_10,GPIOC,GPIO_Mode_Out_PP,0},
	{DO_DISEL1	,DIO_OUT,GPIO_Pin_11,GPIOC,GPIO_Mode_Out_PP,0},
	{DO_DISEL2	,DIO_OUT,GPIO_Pin_12,GPIOC,GPIO_Mode_Out_PP,0},

	{DI0		,DIO_IN	,GPIO_Pin_12,GPIOA,GPIO_Mode_IPU	,1},
	{DI1		,DIO_IN	,GPIO_Pin_15,GPIOA,GPIO_Mode_IPU	,1},
	{DI2		,DIO_IN	,GPIO_Pin_8 ,GPIOB,GPIO_Mode_IPU	,1},
	{DI3		,DIO_IN	,GPIO_Pin_9 ,GPIOB,GPIO_Mode_IPU	,1},

	{DO_LED		,DIO_OUT,GPIO_Pin_2	,GPIOB,GPIO_Mode_Out_PP,1},
}; 

sBIT_FILTER bitFilter[] = 
{
	{FT_KEY1,	0,HZ_TICK/10,DIO_ALARM_HIGH},	{FT_VOUT9, 0,HZ_TICK/500,DIO_ALARM_HIGH},		{FT_VOUT1,	0,HZ_TICK/10, DIO_ALARM_HIGH},	{FT_OC1,	0,HZ_TICK/10,DIO_ALARM_HIGH},
	{FT_KEY2,	0,HZ_TICK/10,DIO_ALARM_HIGH},	{FT_VOUT10,0,HZ_TICK/500,DIO_ALARM_HIGH},		{FT_VOUT2,	0,HZ_TICK/10, DIO_ALARM_HIGH},	{FT_OC2,	0,HZ_TICK/10,DIO_ALARM_HIGH},
	{FT_KEY3,	0,HZ_TICK/10,DIO_ALARM_HIGH},	{FT_VOUT11,0,HZ_TICK/500,DIO_ALARM_HIGH},		{FT_VOUT3,	0,HZ_TICK/10, DIO_ALARM_HIGH},	{FT_OC3,	0,HZ_TICK/10,DIO_ALARM_HIGH},
	{FT_KEY4,	0,HZ_TICK/10,DIO_ALARM_HIGH},	{FT_VOUT12,0,HZ_TICK/500,DIO_ALARM_HIGH},		{FT_VOUT4,	0,HZ_TICK/10, DIO_ALARM_HIGH},	{FT_OC4,	0,HZ_TICK/10,DIO_ALARM_HIGH},
	{FT_KEY8,	0,HZ_TICK/10,DIO_ALARM_HIGH},	{FT_STATE1,0,HZ_TICK/10, DIO_ALARM_HIGH},		{FT_VOUT8,	0,HZ_TICK/500,DIO_ALARM_HIGH},	{FT_OC8,	0,HZ_TICK/10,DIO_ALARM_HIGH},
	{FT_KEY7,	0,HZ_TICK/10,DIO_ALARM_HIGH},	{FT_VOUT15,0,HZ_TICK/10, DIO_ALARM_HIGH},		{FT_VOUT7,	0,HZ_TICK/500,DIO_ALARM_HIGH},	{FT_OC7,	0,HZ_TICK/10,DIO_ALARM_HIGH},
	{FT_KEY6,	0,HZ_TICK/10,DIO_ALARM_HIGH},	{FT_VOUT14,0,HZ_TICK/10, DIO_ALARM_HIGH},		{FT_VOUT6,	0,HZ_TICK/10, DIO_ALARM_HIGH},	{FT_OC6,	0,HZ_TICK/10,DIO_ALARM_HIGH},
	{FT_KEY5,	0,HZ_TICK/10,DIO_ALARM_HIGH},	{FT_VOUT13,0,HZ_TICK/10, DIO_ALARM_HIGH},		{FT_VOUT5,	0,HZ_TICK/10, DIO_ALARM_HIGH},	{FT_OC5,	0,HZ_TICK/10,DIO_ALARM_HIGH},
};

//-------------------------------------------------------------------------
void DIO_SetMode(ePIN_NAME pin, DIO_MODE_TYPE type)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin 	= dio_pins[pin].pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
uint8_t DI_ReadAll(uint8_t* bitBuf,uint8_t bufLen)
{
	uint16_t i;
	
	if ( bufLen * 8 < 32 )
		return 0;

	DIO_SetMode(DO_DISEL0,dio_pins[DO_DISEL0].mode);
	DIO_SetMode(DO_DISEL1,dio_pins[DO_DISEL1].mode);
	DIO_SetMode(DO_DISEL2,dio_pins[DO_DISEL2].mode);

	for(i=0;i<4;i++){
		if ( ((i*2) >> 0)&0x01 ) DIO_Write(DO_DISEL0,1);	else DIO_Write(DO_DISEL0,0);
		if ( ((i*2) >> 1)&0x01 ) DIO_Write(DO_DISEL1,1);	else DIO_Write(DO_DISEL1,0);
		if ( ((i*2) >> 2)&0x01 ) DIO_Write(DO_DISEL2,1);	else DIO_Write(DO_DISEL2,0);
		DelayUs(10);
		bitBuf[i] = (DIO_Read(DI0)<<0) | (DIO_Read(DI1)<<1) | (DIO_Read(DI2)<<2) | (DIO_Read(DI3)<<3);

		if ( ((i*2+1) >> 0)&0x01 ) DIO_Write(DO_DISEL0,1);	else DIO_Write(DO_DISEL0,0);
		if ( ((i*2+1) >> 1)&0x01 ) DIO_Write(DO_DISEL1,1);	else DIO_Write(DO_DISEL1,0);
		if ( ((i*2+1) >> 2)&0x01 ) DIO_Write(DO_DISEL2,1);	else DIO_Write(DO_DISEL2,0);
		DelayUs(10);
		bitBuf[i] = (DIO_Read(DI0)<<4) | (DIO_Read(DI1)<<5) | (DIO_Read(DI2)<<6) | (DIO_Read(DI3)<<7);
	}

	return 1;
}


//-------------------------------------------------------------------------
void DIO_Init()
{
	uint32_t i;

	/* Enable GPIOx clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
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

