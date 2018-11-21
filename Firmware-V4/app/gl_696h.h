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

	uint16_t vol_max;				//最大电压值,V
	uint16_t vol_scale;			//
	uint16_t vol_err_rate;		//输出误差比率,1‰
	uint16_t ctl_scale;			//
	uint16_t vol_step;			//电压步进,1‰
	uint16_t vol_step_interval;	//电压步进间隔,mS
	uint16_t vol_step_timeout;	//电压步进输出超时,mS
	uint16_t vol_level1;			//高压上升到此值后，开始加电流，V
  
	uint16_t cur_max;				//最大功率值,mW
	uint16_t cur_err_rate;		//输出误差比率,1‰
	uint16_t cur_scale;			//
	uint16_t cur_step;			//电流步进控制电压mv
	uint16_t cur_step_interval;	//电流步进输出间隔,mS
	uint16_t cur_step_timeout;	//电流步进输出超时,mS
	uint16_t cur_ctl_start;		//电流控制起始值
  
	uint16_t pre_vol_set;
	uint16_t vol_set;			//电压设定值,V
	uint16_t vol_fb;			//电压实际采样值,V
	uint16_t vol_ctl;			//控制电压输出值,V

	uint16_t pre_cur_set;
	uint16_t cur_set;	//电流设定值,0.1mA
	uint16_t cur_fb;	//当前电流采样值,0.1mA
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
