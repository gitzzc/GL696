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



typedef struct _HV_CFG_
{
	uint16_t usVolAdcIO;
	uint16_t usVolDacIO;
	uint16_t usCurAdcIO;
	uint16_t usCurDacIO;
	uint16_t usPowerIO;
	
	uint16_t usVolMax;			//最大电压值,KV
	uint16_t usVolAdcScale;		//电压采样系数，
	uint16_t usVolDacScale;		//电压控制系数，
	uint16_t usVolError;		//输出误差比率,0.1%
	uint16_t usVolStep;			//电压步进,1‰
	uint16_t usVolStepInterval;	//电压步进间隔,mS
	uint16_t usVolStepTimeout;	//电压步进输出超时,mS
	uint16_t usVolStart;		//高压上升到此值后，开始加电流，V
  
	uint16_t usCurMax;			//最大功率值,uA
	uint16_t usCurAdcScale;		//电流采样系数，
	uint16_t usCurDacScale;		//电流控制系数，
	uint16_t usCurError;		//输出误差比率,0.1%
	uint16_t usCurScale;		//
	uint16_t usCurStep;			//电流步进控制电压mv
	uint16_t usCurStepInterval;	//电流步进输出间隔,mS
	uint16_t usCurStepTimeout;	//电流步进输出超时,mS
	uint16_t usCurStart;		//电流控制起始值
  
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
}sHVConfig,*psHVConfig;

typedef struct _HV_STATUS_
{
	uint16_t	usStatus;
	uint16_t	usVolSet;
	uint16_t	usVolFb;
	uint16_t	usVolCtrl;
	uint16_t	usVolCtrlDac;
	uint16_t	usCurSet;
	uint16_t	usCurFb;
	uint16_t	usCurCtrl;
	uint16_t	usCurCtrlDac;
	uint16_t 	usReserve[7];
}sHVStatus,*psHVStatus;

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

typedef struct _SYS_CONFIG_
{
	uint16_t usPowerPumpType;
	uint16_t usPowerPumpPowerIO;
	uint16_t usPowerPumpComPort;
	uint16_t usMolecularPumpType;
	uint16_t usMolecularPumpPowerIO;
	uint16_t usMolecularPumpComPort;
	uint16_t usVacuumGaugeType;
	uint16_t usVacuumGaugePowerIO;
	uint16_t usVacuumGaugeComPort;
	uint16_t usVacuumGaugeAdcIO;

};


typedef struct _SYS_STATUS_
{
	uint16_t	usPowerPump;
	uint16_t	usVacuumGauge;
	float		usVacuum;
	uint16_t	usMolcularPump;
	uint16_t	usMP_Freq;
	uint16_t	usMP_Voltage;
	uint16_t	usMP_Current;
	uint16_t	usMP_Alarm;
	uint16_t	usMP_Temperature;




};


extern HVS hvsr;
extern HVS hvsl;
extern float vmeter;


void vGL696H_Task( void *pvParameters );


#endif
