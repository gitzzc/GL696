/*
 * FreeModbus Libary: 
 * Copyright (C) 
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c
 */

/* ----------------------- System includes ----------------------------------*/

/* ----------------------- FreeRTOS includes --------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* ----------------------- STRM32 includes ----------------------------------*/
#include "stm32f10x.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_TIMER_DEV            ( TIM2 )
#define MB_TIMER_PRESCALER      ( 255UL )
#define MB_IRQ_PRIORITY         ( configLIBRARY_KERNEL_INTERRUPT_PRIORITY )

/* Timer ticks are counted in multiples of 50us. Therefore 20000 ticks are
 * one second.
 */
#define MB_TIMER_TICKS          ( 20000UL )


/* ----------------------- Static variables ---------------------------------*/
static USHORT   usTimerDeltaOCRA;


/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{

//	unsigned long ulFrequency;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef 		NVIC_InitStructure;
//	TIM_OCInitTypeDef  		TIM_OCInitStructure;

    /* Calculate output compare value for timer1. */
    usTimerDeltaOCRA =
        ( ( configCPU_CLOCK_HZ / ( MB_TIMER_PRESCALER + 1 ) ) *
          usTim1Timerout50us ) / ( MB_TIMER_TICKS );

	/* Enable timer clocks */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

	/* Initialise data. */
	TIM_DeInit( MB_TIMER_DEV );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

	/* Time base configuration for timer 2. */
	TIM_TimeBaseStructure.TIM_Period = usTimerDeltaOCRA;
	TIM_TimeBaseStructure.TIM_Prescaler = MB_TIMER_PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( MB_TIMER_DEV, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig( MB_TIMER_DEV, DISABLE );
	
	/* PWM1 Mode configuration: Channel1 */
/*	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = usTimerDeltaOCRA;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(MB_TIMER_DEV, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(MB_TIMER_DEV, TIM_OCPreload_Enable);
*/		
	/* Enable TIM2 IT. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MB_IRQ_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );	
	//TIM_ITConfig( MB_TIMER_DEV, TIM_IT_Update, ENABLE );

	/* Finally, enable timers. */
	//TIM_Cmd( MB_TIMER_DEV, ENABLE );
	
    vMBPortTimersDisable(  );

    return TRUE;
}

void
TIM2_IRQHandler( void )
{
    static portBASE_TYPE xTaskSwitch = pdFALSE;

    portENTER_SWITCHING_ISR(  );

    if( ( usTimerDeltaOCRA > 0 ) && ( TIM_GetITStatus( MB_TIMER_DEV, TIM_IT_Update ) ) )
    {
        xTaskSwitch = pxMBPortCBTimerExpired(  );
    }

   	TIM_ClearFlag( MB_TIMER_DEV, TIM_IT_Update );
    portEXIT_SWITCHING_ISR( xTaskSwitch );
}

void
vMBPortTimersEnable(  )
{
	if( usTimerDeltaOCRA > 0 )
    {
		TIM_SetCounter( MB_TIMER_DEV, 0 );
		TIM_ClearFlag( MB_TIMER_DEV, TIM_FLAG_Update );
		TIM_ITConfig ( MB_TIMER_DEV, TIM_IT_Update, ENABLE );
		TIM_Cmd( MB_TIMER_DEV, ENABLE );
    }
}

void
vMBPortTimersDisable(  )
{
    /* We can always clear both flags. This improves performance. */
    TIM_ClearFlag( MB_TIMER_DEV, TIM_FLAG_Update );
    TIM_ITConfig ( MB_TIMER_DEV, TIM_IT_Update, DISABLE );
	TIM_Cmd( MB_TIMER_DEV, DISABLE );
}
