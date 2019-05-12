#ifndef __GL696H_H__
#define __GL696H_H__

#include "stdint.h"

#define HW_VER		0x0200
#define MAX_HV_NO 	4

typedef struct _GL_CONFIG_
{
	//------0-7----------
	uint16_t 	usYear;
	uint16_t 	usDate;
	uint16_t 	usHour;
	uint16_t 	usTime;
	uint16_t 	usHWId;
	uint16_t 	usHWVer;
	uint16_t 	usFWId;
	uint16_t 	usFWVer;

	//------8-15---------
	uint16_t 	usKey;
	uint16_t 	usSerial[4];
	uint16_t	usReserve1[3];

	//------16-31--------
	uint16_t	usReserve2[16];

	//------32-39---------
	uint16_t	usAddrss;
	uint16_t	usReserve3[7];

	//------40-47---------
	uint16_t 	usVolMax;			//最大电压值,V
	uint16_t 	usVolScale;			//
	uint16_t 	usVolErrorRate;		//输出误差比率,0.1%
	uint16_t 	usVolStep;			//电压步进
	uint16_t 	usVolStepInterval;	//电压步进间隔,mS
	uint16_t 	usVolStepTimeout;	//电压步进输出超时,mS
	uint16_t 	usVolLevel1;		//高压上升到此值后，开始加电流，V
	uint16_t 	usCurMax;			//最大功率值,mW
	uint16_t 	usCurScale;			//
	uint16_t 	usCurErrorRate;		//输出误差比率,0.1%
	uint16_t 	usCurStep;			//电流步进控制电压mv
	uint16_t 	usCurStepInterval;	//电流步进输出间隔,mS
	uint16_t 	usCurStepTimeout;	//电流步进输出超时,mS
	uint16_t 	usCurCtrlStart;		//电流控制起始值
  
	uint16_t	usMotorOnTime;
	uint16_t	usMotorOffTime;
	uint16_t	BaffleOnTime;
	uint16_t	BaffleOffTime;
} __attribute__ ((aligned(4))) sGL_CONFIG,*psGL_CONFIG;

typedef struct _HV_STATUS_
{
    union _USSTATUS
    {
        uint16_t word;
        struct{
        	uint16_t POWERON	:1;
        	uint16_t ERROR		:1;
        	uint16_t Start		:1;
        	uint16_t Stop		:1;
        }bit;
    } usStatus;
	uint16_t 	usVolSet;	//电压设定值,V
	uint16_t 	usVolCtrl;	//控制电压输出值,V
	uint16_t 	usVolFb;	//电压实际采样值,V
	uint16_t 	usCurSet;	//电流设定值,0.1mA
	uint16_t 	usCurCtrl;	//
	uint16_t 	usCurFb;	//当前电流采样值,0.1mA
	uint16_t	usReserve[1];
} __attribute__ ((aligned(4))) sHV_STATUS,*psHV_STATUS;

typedef struct _MPUMP_
{
	uint16_t 	status;
	uint16_t 	freq;
	uint16_t 	voltage;
	uint16_t 	current;
	uint16_t 	alarm;
	uint16_t 	temperature;
	uint16_t	usReserve[2];
} __attribute__ ((aligned(4))) sMPUMP,*psMPUP;

typedef struct _sGL_STATUS_
{
	uint16_t 	usRelay[2];
	uint16_t	usADC[16];
	uint16_t	usDAC[16];

	sHV_STATUS	shv[MAX_HV_NO];
	sMPUMP 		smpump;
	float 		vmeter;
} __attribute__ ((aligned(4))) sGL_STATUS,*psGL_STATUS;


extern sGL_STATUS sGL_status;
extern sGL_CONFIG sGL_config;
extern sGL_CONFIG const sGL_default_config;

void AL1000_Init(void);
void AL1000_Task(void);


#endif
