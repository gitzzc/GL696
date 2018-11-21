#ifndef __GL696H_H__
#define __GL696H_H__

#include "timerout.h"
#include "stdint.h"

typedef struct 
{
  uint16_t * queue;
  unsigned char size;
  unsigned char front;
  unsigned char rear;
} QUEUE,*pQUEUE;



typedef struct 
{
	uint16_t id;
	unsigned char vol_adc_ch;
	unsigned char vol_dac_ch;
	unsigned char cur_adc_ch;
	unsigned char cur_dac_ch;
	unsigned char power_ch;
	
	uint16_t st;

	uint16_t vol_max;				//����ѹֵ,V
	uint16_t vol_scale;			//
	uint16_t vol_err_rate;		//���������,1��
	uint16_t ctl_scale;			//
	uint16_t vol_step;			//��ѹ����,1��
	uint16_t vol_step_interval;	//��ѹ�������,mS
	uint16_t vol_step_timeout;	//��ѹ���������ʱ,mS
	uint16_t vol_level1;			//��ѹ��������ֵ�󣬿�ʼ�ӵ�����V
  
	uint16_t cur_max;				//�����ֵ,mW
	uint16_t cur_err_rate;		//���������,1��
	uint16_t cur_scale;			//
	uint16_t cur_step;			//�����������Ƶ�ѹmv
	uint16_t cur_step_interval;	//��������������,mS
	uint16_t cur_step_timeout;	//�������������ʱ,mS
	uint16_t cur_ctl_start;		//����������ʼֵ
  
	uint16_t pre_vol_set;
	uint16_t vol_set;			//��ѹ�趨ֵ,V
	uint16_t vol_fb;			//��ѹʵ�ʲ���ֵ,V
	uint16_t vol_ctl;			//���Ƶ�ѹ���ֵ,V

	uint16_t pre_cur_set;
	uint16_t cur_set;	//�����趨ֵ,0.1mA
	uint16_t cur_fb;	//��ǰ��������ֵ,0.1mA
	uint16_t cur_ctl;	//

	unsigned long  task;
	unsigned long  pre_tick;
	
	QUEUE* vol_queue;
	QUEUE* cur_queue;
	
	psTIMEOUT vol_set_to;
	psTIMEOUT vol_check_to;
	psTIMEOUT cur_set_to;
	psTIMEOUT cur_check_to;
}HVS;


typedef struct _POWER_PUMP_
{
	uint16_t usPowerIO;
	uint16_t usPowerON;
	uint16_t usReserve[6];
} sVacuumGauge,*psVacuumGauge;

typedef struct _VACUUM_GAUGE_
{
	uint16_t usPowerIO;
	uint16_t usPowerON;
	float 	 fVacuum;
	uint16_t usStartDelay;
	uint16_t usStopDelay;
	uint16_t usReserve[2];
} sVacuumGauge,*psVacuumGauge;

typedef struct _MOLECULAR_PUMP_
{
	uint16_t usPowerIO;
	uint16_t usPowerON;
	uint16_t usStatus;
	uint16_t usFreq;
	uint16_t usVoltage;
	uint16_t usCurrent;
	uint16_t usAlarm;
	uint16_t usTemperature;
	uint16_t usReserve[8];
} sMolecularPump,*psMolecularPump;

extern HVS hvsr;
extern HVS hvsl;
extern float vmeter;


void vGL696H_Task( void *pvParameters );


#endif
