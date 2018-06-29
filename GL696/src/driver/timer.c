/*
 *	�ļ����ƣ�timer.c
 *	ժ    Ҫ���ṩϵͳ������ʱ���׼���Ͷ�ʱ���ܡ�
 *           
 *          
 *	��ǰ�汾��0.11
 *	������ڣ�
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
 * return value:    pTIMEOUT:ָ��TIMEOUT�Ľṹָ��
 * description:     ����һ��TIMEOUT�ṹ�壬������ָ������ṹ���ָ��
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
 * argument:        pTIMEOUT:ָ��TIMEOUT�Ľṹָ��
 * return value:    none
 * description:     ɾ��һ��TIMEOUT�ṹ��
 *
 */
void del_timeout(psTIMEOUT psto) 
{
	free(psto);
}
#endif

/*
 * function		: start_timeout
 * argument		: pTIMEOUT:ָ��TIMEOUT�Ľṹָ��
 *				  		time	�趨�Ķ�ʱʱ�䣬
 * return value	: none
 * description	: �趨��ʱ����ʱʱ��
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
 * argument		: pTIMEOUT:ָ��TIMEOUT�Ľṹָ��
 * return value	: none
 * description	: ������ʱ��
 *
 */
void restart_timeout(sTIMEOUT* psto)
{      
	psto->origin 	= GET_SYS_TICK();
	psto->status 	= TO_RUNING;
}

/*
 * function		: stop_timeout
 * argument		: pTIMEOUT:ָ��TIMEOUT�Ľṹָ��
 * return value	: none
 * description	: �趨��ʱ����ʱʱ��
 *
 */
void stop_timeout(sTIMEOUT* psto)
{      
	psto->status 	= TO_STOP;
}

/*
 * function		: get_timeout
 * argument		: pTIMEOUT:ָ��TIMEOUT�Ľṹָ��
 * return value	: ��ʱ��״̬
 *				TO_RUNING ��ʱ������������
 *				TO_STOP	  ��ʱ�Ѿ�����������ֹͣ����
 *				TO_TIMEOUT ��ʱ����ʱ��������״ֻ̬�ڶ�ʱ�������һ�ζ�ȡʱ����
 *
 * description	: ��ȡ��ǰ��ʱ��״̬
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
