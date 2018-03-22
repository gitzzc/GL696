/* Includes ------------------------------------------------------------------*/

#include "stdlib.h"
#include "stdio.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "gpio.h"

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

sBIT_FILTER bitFilter[sizeof(dio_pins)/sizeof(sDIO_PIN)];
uint8_t	alarm_new;
uint8_t	alarm_num;
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
	if ( pin > sizeof(dio_pins)/sizeof(dio_pins[0]) )
		return;
		
	if ( hi_low == pdLOW )
		GPIO_ResetBits(dio_pins[pin].reg,dio_pins[pin].pin);
	else
		GPIO_SetBits(dio_pins[pin].reg,dio_pins[pin].pin);
}

//-------------------------------------------------------------------------
uint8_t DIO_Read(ePIN_NAME pin)
{
	if ( pin > sizeof(dio_pins)/sizeof(dio_pins[0]) )
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

	for(i=0;i<sizeof(dio_pins)/sizeof(sDIO_PIN);i++){
		if ( dio_pins[i].type == DIO_OUT ){
			DIO_Write((ePIN_NAME)i,dio_pins[i].level);
		}
		DIO_SetMode((ePIN_NAME)i,dio_pins[i].mode);
	}
}

//-------------------------------------------------------------------------


volatile uint16_t RelayBits=0;

#define RELAY_OC 	GPIO_Pin_12
#define RELAY_CS0 	GPIO_Pin_13
#define RELAY_CS1 	GPIO_Pin_14
#define RELAY_PORT 	GPIOD

void Relay_INIT()
{
	GPIO_InitTypeDef 	GPIO_InitStructure;

	/* Enable the Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
									  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7	| \
									  RELAY_OC	 | RELAY_CS0  | RELAY_CS1;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_Init(RELAY_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 	= RELAY_OC;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;
	GPIO_Init(RELAY_PORT, &GPIO_InitStructure);

	GPIO_WriteBit(RELAY_PORT, RELAY_OC,  Bit_SET);
	GPIO_WriteBit(RELAY_PORT, RELAY_CS0, Bit_RESET);
	GPIO_WriteBit(RELAY_PORT, RELAY_CS1, Bit_RESET);
}

void Relay_Set(uint16_t relay)
{
	volatile int i;
	
	GPIO_WriteBit(RELAY_PORT, RELAY_CS0, Bit_RESET);
	for(i=0;i<100;i++);

	GPIO_Write(RELAY_PORT,((~relay>>8)&0xFF) | GPIO_ReadOutputData(RELAY_PORT)&~0xFF);
	GPIO_WriteBit(RELAY_PORT, RELAY_CS1, Bit_SET);
	for(i=0;i<100;i++);
	GPIO_WriteBit(RELAY_PORT, RELAY_CS1, Bit_RESET);
	for(i=0;i<100;i++);

	GPIO_Write(RELAY_PORT,((~relay)&0xFF) | GPIO_ReadOutputData(RELAY_PORT)&~0xFF);
	GPIO_WriteBit(RELAY_PORT, RELAY_CS0, Bit_SET);
	for(i=0;i<100;i++);
	//GPIO_WriteBit(RELAY_PORT, RELAY_CS0, Bit_RESET);
	//for(i=0;i<100;i++);

	GPIO_WriteBit(RELAY_PORT, RELAY_OC, Bit_RESET);
	
	RelayBits = relay;
}

void Relay_SetBit(uint16_t relay_bit,uint8_t st)
{
	if ( relay_bit > 15 )
		return ;
		
	if ( st == 1 )
		Relay_Set(RelayBits | (1<<relay_bit));
	else if ( st == 0 )
		Relay_Set(RelayBits & (~(1<<relay_bit)));
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
				filter[i].bit_state = DIO_CONFIRM;
				if ( filter[i].bit == filter[i].alarm_type ){
					if ( filter[i].alarm == 0 )
						new ++;
					filter[i].alarm = 1;
					alarm_num ++;
				} else
					filter[i].alarm = 0;
			}
		} else {
			filter[i].bit_state = DIO_CHANGING;
			filter[i].count	 	= 0;
		}
		filter[i].level = bit;
	}
	return new;
}
