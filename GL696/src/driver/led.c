#include "projdefs.h"
#include "led.h"

void LED_SetStartup(psLED_CONFIG cfg,uint32_t on_time,uint32_t off_time,uint32_t count)
{
	cfg->on_time 	= on_time;
	cfg->off_time 	= off_time;
	cfg->count		= count;
}

void LED_SetDefault(psLED_CONFIG cfg,uint32_t on_time,uint32_t off_time)
{
	cfg->default_on_time 	= on_time;
	cfg->default_off_time 	= off_time;
}

void LED_Task(psLED_CONFIG cfg)
{
	uint8_t ret;
	
	if ( (ret=get_timeout(&(cfg->stimer))) == TO_TIMEOUT ){
		if ( cfg->led ){
			SYSTEM_LED_ON();
			cfg->led = 0;
			if ( cfg->count ){
				start_timeout(&(cfg->stimer),cfg->off_time);
			} else {
				start_timeout(&(cfg->stimer),cfg->default_off_time);
			}
		} else {
			SYSTEM_LED_OFF();
			cfg->led = 1;
			if ( cfg->count ){
				cfg->count --;
				start_timeout(&(cfg->stimer),cfg->on_time);
			} else {
				start_timeout(&(cfg->stimer),cfg->default_on_time);
			}
		}
	} else if ( ret != TO_RUNING ){
		start_timeout(&(cfg->stimer),cfg->default_on_time);
	}
}
