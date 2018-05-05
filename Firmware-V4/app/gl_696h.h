#ifndef __GL696H_H__
#define __GL696H_H__

#include "timerout.h"
#include "stdint.h"

typedef struct 
{
  unsigned short * queue;
  unsigned char size;
  unsigned char front;
  unsigned char rear;
} QUEUE,*pQUEUE;



typedef struct 
{
	unsigned short id;
	unsigned char vol_adc_ch;
	unsigned char vol_dac_ch;
	unsigned char cur_adc_ch;
	unsigned char cur_dac_ch;
	unsigned char power_ch;
	
	unsigned short st;  		

	unsigned short vol_max;				//����ѹֵ,V
	unsigned short vol_scale;			//
	unsigned short vol_err_rate;		//���������,1��
	unsigned short vol_step;			//��ѹ����,1��
	unsigned short vol_step_interval;	//��ѹ�������,mS
	unsigned short vol_step_timeout;	//��ѹ���������ʱ,mS
	unsigned short vol_level1;			//��ѹ��������ֵ�󣬿�ʼ�ӵ�����V
  
	unsigned short cur_max;				//�����ֵ,mW
	unsigned short cur_err_rate;		//���������,1��	
	unsigned short cur_scale;			//
	unsigned short cur_step;			//�����������Ƶ�ѹmv
	unsigned short cur_step_interval;	//��������������,mS
	unsigned short cur_step_timeout;	//�������������ʱ,mS
	unsigned short cur_ctl_start;		//����������ʼֵ
  
	unsigned short pre_vol_set;  		
	unsigned short vol_set;			//��ѹ�趨ֵ,V
	unsigned short vol_fb;			//��ѹʵ�ʲ���ֵ,V
	unsigned short vol_ctl;			//���Ƶ�ѹ���ֵ,V

	unsigned short pre_cur_set;  		
	unsigned short cur_set;	//�����趨ֵ,0.1mA
	unsigned short cur_fb;	//��ǰ��������ֵ,0.1mA
	unsigned short cur_ctl;	//

	unsigned long  task;
	unsigned long  pre_tick;
	
	QUEUE* vol_queue;
	QUEUE* cur_queue;
	
	psTIMEOUT vol_set_to;
	psTIMEOUT vol_check_to;
	psTIMEOUT cur_set_to;
	psTIMEOUT cur_check_to;
}HVS;

typedef struct 
{
	float vmeter;
	unsigned long run_time;
	unsigned short mpump_freq;
} SYSCTL;

extern HVS hvsr;
extern HVS hvsl;
extern float vmeter;


void vGL696H_Task( void *pvParameters );


#endif
