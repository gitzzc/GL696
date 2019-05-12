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
	uint16_t 	usVolMax;			//����ѹֵ,V
	uint16_t 	usVolScale;			//
	uint16_t 	usVolErrorRate;		//���������,0.1%
	uint16_t 	usVolStep;			//��ѹ����
	uint16_t 	usVolStepInterval;	//��ѹ�������,mS
	uint16_t 	usVolStepTimeout;	//��ѹ���������ʱ,mS
	uint16_t 	usVolLevel1;		//��ѹ��������ֵ�󣬿�ʼ�ӵ�����V
	uint16_t 	usCurMax;			//�����ֵ,mW
	uint16_t 	usCurScale;			//
	uint16_t 	usCurErrorRate;		//���������,0.1%
	uint16_t 	usCurStep;			//�����������Ƶ�ѹmv
	uint16_t 	usCurStepInterval;	//��������������,mS
	uint16_t 	usCurStepTimeout;	//�������������ʱ,mS
	uint16_t 	usCurCtrlStart;		//����������ʼֵ
  
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
	uint16_t 	usVolSet;	//��ѹ�趨ֵ,V
	uint16_t 	usVolCtrl;	//���Ƶ�ѹ���ֵ,V
	uint16_t 	usVolFb;	//��ѹʵ�ʲ���ֵ,V
	uint16_t 	usCurSet;	//�����趨ֵ,0.1mA
	uint16_t 	usCurCtrl;	//
	uint16_t 	usCurFb;	//��ǰ��������ֵ,0.1mA
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
