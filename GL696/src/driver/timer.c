/*
 *	文件名称：timer.c
 *	摘    要：提供系统基本的时间基准，和定时功能。
 *           
 *          
 *	当前版本：0.11
 *	完成日期：
 *
 */

#include "stdint.h"
#include "stdlib.h"

#include "projdefs.h"
#include "timer.h"

volatile uint32_t system_tick = 0;

#if 0
//-----------------------------------------------------------------------
/*
 * function:        creat_timer
 * argument:        none
 * return value:    pTIMEOUT:指向TIMEOUT的结构指针
 * description:     创建一个TIMEOUT结构体，并返回指向这个结构体的指针
 *
 */
psTIMEOUT creat_timeout(psTIMEOUT psto)
{
	if ( psto == NULL ) {
 		if ( (psto = malloc(sizeof(sTIMEOUT))) == NULL )
		assert_failed(__FILE__,__LINE__);
	}

	psto->status = TO_INIT;
	return psto;
}

/*
 * function:        del_timeout
 * argument:        pTIMEOUT:指向TIMEOUT的结构指针
 * return value:    none
 * description:     删除一个TIMEOUT结构体
 *
 */
void del_timeout(psTIMEOUT psto) 
{
	free(psto);
}
#endif

/*
 * function		: start_timeout
 * argument		: pTIMEOUT:指向TIMEOUT的结构指针
 *				  		time	设定的定时时间，
 * return value	: none
 * description	: 设定定时器定时时间
 *
 */
void start_timeout(sTIMEOUT* psto,uint32_t time)
{      
	psto->status 	= TO_RUNING;
	psto->timeout 	= time;	
	psto->origin 	= GET_SYS_TICK();
}

/*
 * function		: restart_timeout
 * argument		: pTIMEOUT:指向TIMEOUT的结构指针
 * return value	: none
 * description	: 重启定时器
 *
 */
void restart_timeout(sTIMEOUT* psto)
{      
	psto->origin 	= GET_SYS_TICK();
	psto->status 	= TO_RUNING;
}

/*
 * function		: stop_timeout
 * argument		: pTIMEOUT:指向TIMEOUT的结构指针
 * return value	: none
 * description	: 设定定时器定时时间
 *
 */
void stop_timeout(sTIMEOUT* psto)
{      
	psto->status 	= TO_STOP;
}

/*
 * function		: get_timeout
 * argument		: pTIMEOUT:指向TIMEOUT的结构指针
 * return value	: 定时器状态
 *				TO_RUNING 定时器正在运行中
 *				TO_STOP	  定时已经结束，并己停止运行
 *				TO_TIMEOUT 定时器定时结束，此状态只在定时结束后第一次读取时返回
 *
 * description	: 读取当前定时器状态
 *
 */
uint8_t get_timeout(sTIMEOUT* psto)
{
	uint32_t ltemp;
	uint8_t ret;

	ret = psto->status;
	if (ret != TO_RUNING)
		return (ret);

	//cli();
	ltemp = GET_SYS_TICK();
	//sei();
	if ( ltemp >= psto->origin )
		ltemp -= psto->origin;
	else 
		ltemp += UINT32_MAX - psto->origin;

	if ( ltemp >= psto->timeout ){
		//psto->timeout = 0;
	  	//psto->origin 	= ltemp;
		psto->status 	= TO_STOP;
		ret 			= TO_TIMEOUT;
    	return ret;
	} else
    	return ret;
}

//--------------------------------------------------------------------------
#if 0
sTIMER_CTL* creat_timer_ctl(uint32_t on, uint32_t off ,void (*func)(uint8_t))
{
	sTIMER_CTL * psTimer_ctl;
	sTIMEOUT * to;

	psTimer_ctl = malloc(sizeof(sTIMER_CTL));
	if ( psTimer_ctl == NULL )
		assert_failed(__FILE__,__LINE__);
		
	to = creat_timeout(NULL);
	if ( to == NULL )
		assert_failed(__FILE__,__LINE__);
		
	psTimer_ctl->status 	= pdOFF;
	psTimer_ctl->on_time 	= on;
	psTimer_ctl->off_time 	= off;
	psTimer_ctl->timer 		= to;
	psTimer_ctl->func 		= func;
	start_timeout(to,on);
	return psTimer_ctl;
}

void del_timer_ctl( sTIMER_CTL* tc)
{
	if ( tc ){
		if ( tc->timer )
			del_timeout(tc->timer);
		free(tc);
	}
}

void timer_ctl_task(sTIMER_CTL* tc)
{ 
	uint8_t ret;
	if ( (ret = get_timeout(tc->timer)) == TO_TIMEOUT ) {
		if ( tc->status == pdON ){
			tc->status = pdOFF;
			if ( tc->func )
				(tc->func)(tc->status);
			if ( tc->off_time != 0 )
				start_timeout ( tc->timer, tc->off_time );
		} else {
			tc->status = pdON;
			if ( tc->func )
				(tc->func)(tc->status);
			if ( tc->on_time != 0 )
				start_timeout ( tc->timer, tc->on_time );
		}
	} else if ( ret == TO_NOT_INIT ) {
		start_timeout(tc->timer, tc->on_time);
	}
}
#endif

//----------------------------------------------------------------------
void Delay(uint32_t tick)
{
	uint32_t prev_tick = GET_SYS_TICK();
	uint32_t t;

	while (1) {
		t = GET_SYS_TICK();
		if ( t < prev_tick ){
			if ( UINT32_MAX - prev_tick + t >= tick )
				break;
		} else if ( t - prev_tick >= tick )
			break;
		FEED_DOG();
	}			
}

void DelayUntil( uint32_t* PreviousWakeTime, uint32_t xTimeIncrement )
{
	uint32_t t;

	while (1) {
		t = GET_SYS_TICK();
		if ( t < *PreviousWakeTime ){
			if ( UINT32_MAX - *PreviousWakeTime + t >= xTimeIncrement )
				break;
		} else if ( t - *PreviousWakeTime >= xTimeIncrement )
			break;
		FEED_DOG();
	}
	*PreviousWakeTime = t;
}

void DelayUs(uint32_t us)
{
	uint32_t __IO i;
	
	for(i=0;i<us;i++);
}
