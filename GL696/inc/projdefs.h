#ifndef PROJDEFS_H
#define PROJDEFS_H

#include "stdint.h"
#include "stdlib.h"

#include "stm32f10x.h"

#include "../src/driver/adc.h"

/* Defines the prototype to which task functions must conform. */
/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE	double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	unsigned portLONG
#define portBASE_TYPE	long


typedef void (*pdTASK_CODE)( void * );
typedef unsigned portLONG portTickType;


#define pdTRUE		( 1 )
#define pdFALSE		( 0 )

#define pdHIGH		( 1 )
#define pdLOW		( 0 )

#define pdON		( 1 )
#define pdOFF		( 0 )

#define pdPASS									( 1 )
#define pdFAIL									( 0 )
#define errQUEUE_EMPTY						( 0 )
#define errQUEUE_FULL							( 0 )

/* Error definitions. */
#define errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY	( -1 )
#define errNO_TASK_TO_RUN						( -2 )
#define errQUEUE_BLOCKED						( -4 )
#define errQUEUE_YIELD							( -5 )

//---------------------------------------------------------------
#define ADC_CUR1	0
#define ADC_CUR2	1
#define ADC_CUR3	2
#define ADC_CUR4	3
#define ADC_CUR5	4
#define ADC_CUR6	5

#define ADC_VOL1	8
#define ADC_VOL2	9
#define ADC_MUX		10

#define ADC_VOL3	5	//cd4051
#define ADC_VOL4	7	//cd4051
#define ADC_LD1		3	//cd4051
#define ADC_LD3		0	//cd4051
#define ADC_LD4		1 //cd4051

#define FEED_DOG	IWDG_ReloadCounter

#define	PMU_MAX			8
//----------PMU_CTRL----------
#define PMU_POWERON	(1<<0)
#define PMU_RESET		(1<<1)
//----------PMU_STATUS--------
#define PMU_OV		(1<<0)
#define PMU_OC		(1<<1)
#define PMU_OT		(1<<2)
#define PMU_ERR		(1<<3)
#define PMU_TO		(1<<4)

#define PMU_UPDATE_VOL	(1<<15)
#define PMU_UPDATE_CTRL	(1<<14)

#define	DIO_OFF	1
#define DIO_ON	0

typedef struct _PMU_STATUS_
{
	uint16_t 	vol;
	uint16_t	cur;
	uint16_t	status;
	uint16_t	vol_set;
	uint16_t	ctrl;
} sPMU_STATUS,*psPMU_STATUS;

typedef struct _SYS_CFG_ 
{
	//------0-7----------
	uint16_t 	year;
	uint16_t 	date;
	uint16_t 	hour;
	uint16_t 	time;
	uint16_t 	hwid;
	uint16_t 	apid;
	uint16_t 	fwver;
	uint16_t 	key;
	
	//------8-15---------
	uint16_t 	serial[2];
	uint16_t	update;	
	uint16_t	reserve1[5];

	//------16-63---------
	uint16_t 	mac[3];
	uint16_t	ip_mode;
	uint16_t	ip_addr[4];
	uint16_t	ip_mask[4];
	uint16_t	gateway[4];
	uint16_t	dns1[4];	
	uint16_t	dns2[4];	
	uint16_t	tcp_host[4];
	uint16_t	tcp_host_port;
	uint16_t	tcp_client_port;
	uint16_t 	udp_host[4];
	uint16_t 	udp_host_port;
	uint16_t	udp_client_port;
	uint16_t	reserve2[12];

	//------64-111---------
	sADDA_CFG	sADC_cfg[16/*ADC_CHANNEL*/];
	uint16_t	reserve3[16];

	//------128-167---------
	uint16_t 	vol[14];
	uint16_t	cur[6];
	uint16_t	pout;
	uint16_t	oc;
	uint16_t	reset;
	int16_t		cur_ld[4];
	uint16_t	pau_st;
	uint16_t	reserve4[12];
	
	//------168-217---------
	sPMU_STATUS spmu[PMU_MAX];
	uint16_t	reserve5[10];
	
	//------218-255---------
	uint16_t	dout[2];
	uint16_t	din[2];
	uint16_t	adc[16];
	uint16_t	reserve6[17];
	
} sSYS_CFG,*psSYS_CFG;

typedef struct __FRAME_HEAD__
{
	uint16_t 	head;		//帧头
	uint16_t	length;	//长度
	uint16_t 	mctrl;	//主控
	uint8_t		type;		//类型
	uint8_t		ret;		//结果
	uint8_t		para_len;//参数个数
	uint8_t		time[14];//时码
}sFRAME_HEAD,*psFRAME_HEAD;

extern sSYS_CFG 			sSys_cfg;
extern const sSYS_CFG sSys_def_cfg;
extern uint8_t LastTimeCode[];

void assert_failed(uint8_t* file, uint32_t line);

#endif /* PROJDEFS_H */

