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

#define DIO_CONFIRM		0
#define DIO_CHANGING 	1

//-------------------------------------------------------------------------
typedef enum   
{
	RELAY0=0,
	RELAY1,	
	RELAY2,	
	RELAY3,	
	RELAY4,	
	RELAY5,	
	RELAY6,	
	RELAY7,	

	RELAY8,	
	RELAY9,	
	RELAY10,	
	RELAY11,	
	RELAY12,	
	RELAY13,	
	RELAY14,	
	RELAY15,	

	PWR_0,	
	PWR_1,	
	PWR_2,	
	PWR_3,	
} ePIN_NAME;

typedef enum   
{
	FLT_PC6 = 0,

} eFILTER_NAME;
//-------------------------------------------------------------------------
typedef struct
{
	ePIN_NAME 			name;
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
	uint16_t 	bit_state	:1;	//DIO_CHANGING,DIO_CONFIRM
	uint16_t 	bit			:2;
	uint16_t 	level		:1;
	uint16_t 	res			:8;
} sBIT_FILTER,*psBIT_FILTER;

//-------------------------------------------------------------------------
extern sBIT_FILTER bitFilter[];
extern volatile uint16_t RelayBits;

#define DIO_GET_FILET

void DIO_Init(void);
void DIO_Write(ePIN_NAME pin, uint8_t hi_low);
uint8_t DIO_Read(ePIN_NAME pin);
void DIO_SetMode(ePIN_NAME pin, DIO_MODE_TYPE type);
uint32_t DIO_MonitorTask(psBIT_FILTER filter,uint8_t* bit_buf,uint32_t bit_num,uint32_t* alarm_num);

void Relay_INIT(void);
void Relay_Set(uint16_t relay);
void Relay_SetBit(uint16_t relay_bit,uint8_t st);
//-------------------------------------------------------------------------


#endif
