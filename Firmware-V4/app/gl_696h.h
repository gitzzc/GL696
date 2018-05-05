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

	unsigned short vol_max;				//最大电压值,V
	unsigned short vol_scale;			//
	unsigned short vol_err_rate;		//输出误差比率,1‰
	unsigned short vol_step;			//电压步进,1‰
	unsigned short vol_step_interval;	//电压步进间隔,mS
	unsigned short vol_step_timeout;	//电压步进输出超时,mS
	unsigned short vol_level1;			//高压上升到此值后，开始加电流，V
  
	unsigned short cur_max;				//最大功率值,mW
	unsigned short cur_err_rate;		//输出误差比率,1‰	
	unsigned short cur_scale;			//
	unsigned short cur_step;			//电流步进控制电压mv
	unsigned short cur_step_interval;	//电流步进输出间隔,mS
	unsigned short cur_step_timeout;	//电流步进输出超时,mS
	unsigned short cur_ctl_start;		//电流控制起始值
  
	unsigned short pre_vol_set;  		
	unsigned short vol_set;			//电压设定值,V
	unsigned short vol_fb;			//电压实际采样值,V
	unsigned short vol_ctl;			//控制电压输出值,V

	unsigned short pre_cur_set;  		
	unsigned short cur_set;	//电流设定值,0.1mA
	unsigned short cur_fb;	//当前电流采样值,0.1mA
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
