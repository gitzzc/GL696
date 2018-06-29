#ifndef __DIO_H__
#define __DIO_H__


#define DIO_MODE_TYPE	GPIOMode_TypeDef

//-------------------------------------------------------------------------
#define DIO_IN			0
#define DIO_OUT			1
#define DIO_IN_ADC		2

#define DIO_ALARM_LOW	0
#define DIO_ALARM_HIGH	1
#define DIO_ALARM_NO 	2

#define DIO_CONFIRM		(0<<1)
#define DIO_CHANGING 	(1<<1)

#define BF_BIT_MASK		0x01


//-------------------------------------------------------------------------

typedef enum   
{	
	DO0=0,
	DO1,
	DO2,
	DO3,
	DO4,
	DO5,
	DO6,
	DO7,
	
	DO8,
	DO9,
	DO10,
	DO11,
	DO12,
	DO13,
	DO14,
	DO15,
	
	DO16,
	DO17,
	DO18,
	DO19,
	DO20,

	DO_ADSEL0,
	DO_ADSEL1,
	DO_ADSEL2,
	DO_DISEL0,
	DO_DISEL1,
	DO_DISEL2,

	DI0,
	DI1,
	DI2,
	DI3,

	DO_LED
} ePIN_NAME;

//-------------------------------------------------------------------------
typedef enum   
{
	FT_KEY1=0,	FT_VOUT9,	FT_VOUT1,FT_OC1,
	FT_KEY2,		FT_VOUT10,FT_VOUT2,FT_OC2,
	FT_KEY3,		FT_VOUT11,FT_VOUT3,FT_OC3,
	FT_KEY4,		FT_VOUT12,FT_VOUT4,FT_OC4,
	FT_KEY8,		FT_STATE1,FT_VOUT8,FT_OC8,
	FT_KEY7,		FT_VOUT15,FT_VOUT7,FT_OC7,	
	FT_KEY6,		FT_VOUT14,FT_VOUT6,FT_OC6,	
	FT_KEY5,		FT_VOUT13,FT_VOUT5,FT_OC5,	
} eFILTER_NAME;
//-------------------------------------------------------------------------
typedef struct
{
	ePIN_NAME 		name;
	uint8_t 			type;
	uint16_t 			pin;
	GPIO_TypeDef* 		reg;
	GPIOMode_TypeDef 	mode;
	uint8_t 			level;
} sDIO_PIN,*psDIO_PIN;

typedef struct
{
	eFILTER_NAME name;
	uint16_t 	count;
	uint16_t 	filter;
	uint16_t 	alarm_type	:2;	//DIO_ALARM _NO, _LOW, _HIGH
	uint16_t 	alarm		:2;	
	uint16_t 	bit			:2;
	uint16_t 	level		:1;
	uint16_t 	res			:9;
} sBIT_FILTER,*psBIT_FILTER;

//-------------------------------------------------------------------------
extern sBIT_FILTER bitFilter[];
void DIO_Init(void);
void DIO_Write(ePIN_NAME pin, uint8_t hi_low);
uint8_t DIO_Read(ePIN_NAME pin);
void DIO_SetMode(ePIN_NAME pin, DIO_MODE_TYPE type);
uint32_t DIO_MonitorTask(psBIT_FILTER filter,uint8_t* bit_buf,uint32_t bit_num,uint32_t* alarm_num);
uint8_t DI_ReadAll(uint8_t* bitBuf,uint8_t bufLen);
void DO_DelayInit(void);
void DO_WriteDelay(ePIN_NAME pin, uint8_t hi_low, uint32_t delay_invert);
void DO_DelayTask( uint32_t tick );


//-------------------------------------------------------------------------


#endif
