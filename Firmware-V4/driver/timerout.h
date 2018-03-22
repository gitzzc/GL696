/*
 *	文件名称：timer.h
 *	摘    要：提供系统基本的时间基准，和定时功能。
 *           
 *          
 *	当前版本：0.11
 *	完成日期：
 *	编译环境：
 *
 */

#ifndef __TIMER_H__
#define __TIMER_H__

//--------------------------------------------------
#define HZ_TICK		1000L

//--------------------------------------------------
extern volatile uint32_t system_tick;

#define IncrementTick()	(system_tick++)
#define GET_SYS_TICK()	(system_tick)

//--------------------------------------------------
#define TO_NOT_INIT	(1<<0)
#define TO_INIT		(1<<1)
#define TO_RUNING	(1<<2)
#define TO_TIMEOUT	(1<<3)
#define TO_STOP		(1<<4)

//--------------------------------------------------
typedef struct
{
	uint32_t 	timeout;
	uint32_t 	origin;
	uint32_t 	backup;
	uint8_t 	status;
} sTIMEOUT,*psTIMEOUT;

typedef struct 
{
	uint8_t 	status;
	uint32_t 	on_time;
	uint32_t 	off_time;
	sTIMEOUT 	*timer;
	void 		(*func)(uint8_t);
} sTIMER_CTL,*psTIMER_CTL;

//--------------------------------------------------
void Delay(uint32_t tick);
void DelayUntil( uint32_t* PreviousWakeTime, uint32_t xTimeIncrement );
psTIMEOUT creat_timeout(psTIMEOUT psto);
void start_timeout(psTIMEOUT psto,uint32_t timeout);
void stop_timeout(psTIMEOUT psto);
void restart_timeout(psTIMEOUT psto);
uint8_t get_timeout(psTIMEOUT psto);

sTIMER_CTL* creat_timer_ctl(uint32_t on, uint32_t off ,void (*func)(uint8_t));
void del_timer_ctl(sTIMER_CTL* tc);
void timer_ctl_task(sTIMER_CTL* tc);

//--------------------------------------------------

#endif 
