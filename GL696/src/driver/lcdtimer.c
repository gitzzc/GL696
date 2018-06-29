/* Library includes. */
#include "stm32f10x.h"
#include "projdefs.h"
#include "lcd.h"

/* The set frequency of the interrupt.  Deviations from this are measured as
the jitter. */
#define timerINTERRUPT_FREQUENCY		( ( unsigned portSHORT ) 1000 )

/* The expected time between each of the timer interrupts - if the jitter was
zero. */
#define timerEXPECTED_DIFFERENCE_VALUE	( SystemCoreClock / timerINTERRUPT_FREQUENCY )

/* The highest available interrupt priority. */
#define timerHIGHEST_PRIORITY			( 0x0F )

/*-----------------------------------------------------------*/
void LcdTimerInit( void )
{
    unsigned long ulFrequency;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable timer clocks */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

	/* Initialise data. */
	TIM_DeInit( TIM2 );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

	/* Time base configuration for timer 2 - which generates the interrupts. */
	ulFrequency = SystemCoreClock / timerINTERRUPT_FREQUENCY;	
	TIM_TimeBaseStructure.TIM_Period 		= ( unsigned portSHORT ) ( ulFrequency & 0xffffUL );
	TIM_TimeBaseStructure.TIM_Prescaler 	= 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig( TIM2, ENABLE );
	
	/* Enable TIM2 IT.  TIM3 does not generate an interrupt. */
	NVIC_InitStructure.NVIC_IRQChannel 				= TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 	= 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = timerHIGHEST_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Init( &NVIC_InitStructure );	
	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE );

	/* Finally, enable timer. */
	TIM_Cmd( TIM2, ENABLE );
}

/*-----------------------------------------------------------*/
void TIM2_IRQHandler( void )
{
	LCD_pixel_refresh();

    TIM_ClearITPendingBit( TIM2, TIM_IT_Update );
}


