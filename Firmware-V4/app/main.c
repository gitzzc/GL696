/*
    FreeRTOS V6.0.5 - Copyright (C) 2010 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS eBook                                  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the LCD by the 'Check' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.
 *
 * "Check" task -  This only executes every five seconds but has the highest
 * priority so is guaranteed to get processor time.  Its main function is to
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write an error to the LCD (via the LCD task).  If all the demo tasks are
 * executing with their expected behaviour then the check task writes PASS
 * along with the max jitter time to the LCD (again via the LCD task), as
 * described above.
 *
 *
 * "uIP" task -  This is the task that handles the uIP stack.  All TCP/IP
 * processing is performed in this task.
 *

 */

/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x.h"

/* Demo app includes. */
#include "lcd.h"
#include "LCD_Message.h"
#include "BlockQ.h"
#include "death.h"
#include "integer.h"
#include "blocktim.h"
#include "partest.h"
#include "semtest.h"
#include "PollQ.h"
#include "flash.h"
#include "comtest2.h"
#include "modbus.h"
#include "touchscreen.h"
#include "serials.h"
#include "pwm_dac.h"

#include "timerout.h"
#include "gl_696h.h"
#include "adc.h"
#include "window.h"
#include "spi.h"
#include "spi_flash.h"

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainUIP_TASK_PRIORITY				( tskIDLE_PRIORITY + 5 )
#define mainGL696H_TASK_PRIORITY			( tskIDLE_PRIORITY + 5 )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY           ( tskIDLE_PRIORITY + 3 )
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainCOM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainCAN_PRIORITY					( tskIDLE_PRIORITY + 5 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainMB_TASK_PRIORITY    			( tskIDLE_PRIORITY + 5 )
#define mainTC_TASK_PRIORITY    			( tskIDLE_PRIORITY + 4 )

/* The WEB server has a larger stack as it utilises stack hungry string
handling library calls. */
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 3 )

#define mainBASIC_LCD_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 3 )

#define mainBASIC_GL696H_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 4 )

/* The check task uses the sprintf function so requires a little more stack. */
#define mainCHECK_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

/* The maximum number of message that can be waiting for display at any one
time. */
#define mainLCD_QUEUE_SIZE					( 3 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The time between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( portTickType ) 5000 / portTICK_RATE_MS )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE		( 115200 )

/* The LED used by the comtest tasks. See the comtest.c file for more
information. */
#define mainCOM_TEST_LED			( 0 )

/*-----------------------------------------------------------*/
/*
 * The LCD is written two by more than one task so is controlled by a
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the LCD directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */
void vLCDTask( void *pvParameters );

/*
 * Configures the timers and interrupts for the fast interrupt test as
 * described at the top of this file.
 */
extern void vSetupTimerTest( void );

/*
 * The task that handles the uIP stack.  All TCP/IP processing is performed in
 * this task.


 */
extern void vuIP_Task( void *pvParameters );
extern void vMBTCPSlaveTask( void *pvParameters );
extern void vMBRTUSlaveTask( void *pvParameters );
extern void vMassFlow_Task( void *pvParameters );
extern void vGL696H_Test_Task( void *pvParameters );

/*-----------------------------------------------------------*/

/* Local includes ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define countof(a) (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
sADC_CONFIG sADC_cfg[ADC_CHANNEL];

/*-----------------------------------------------------------*/
void IWDG_init(void)
{
	/* IWDG timeout equal to 100 ms (the timeout may varies due to LSI frequency
	dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz */
	IWDG_SetPrescaler(IWDG_Prescaler_128);
	
	/* Set counter reload value to 625 */
	IWDG_SetReload(625);
	
	/* Reload IWDG counter */
	IWDG_ReloadCounter();
	
	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}


static void prvSetupHardware( void )
{
	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

	/* Set the Vector Table base address at 0x08003000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0000 );
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	vParTestInitialise();
	
	ADC_GetConfig(sADC_cfg);
	ADC_SetConfig(sADC_cfg);
	ADC_InitChannel();

	PWM_DAC_INIT();
	Relay_INIT();
	SPI_Bus_init();
	
	IWDG_init();
	
	/* Configure the timers used by the fast interrupt timer test. */
	//vSetupTimerTest();
}
/*-----------------------------------------------------------*/

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc( int ch, FILE *f )
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	(void)f;
	
	xSerialPutChar( 0, ch, 0 );
	return ch;
}


/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  printf("%s %d\r\n",file,line);
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif


int main( void )
{
	int i;
#ifdef DEBUG
  debug();
#endif

	prvSetupHardware();

/*	for(i=0;i<REG_INPUT_NREGS;i++)
		usRegInputBuf[i] = 30000+i;
	for(i=0;i<REG_HOLDING_NREGS;i++)
		usRegHoldingBuf[i] = 40000 + i;
*/	
	/* Create the queue used by the LCD task.  Messages for display on the LCD
	are received via this queue. */
	//xLCDQueue = xQueueCreate( mainLCD_QUEUE_SIZE, sizeof( xLCDMessage ) );

	/* Start the standard demo tasks. */
	//vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
    //vCreateBlockTimeTasks();
    //vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
    //vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
    //vStartIntegerMathTasks( mainINTEGER_TASK_PRIORITY );
	vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
	
	/* Create the uIP task.  The WEB server runs in this task. */
//	xTaskCreate( vuIP_Task, ( signed char * ) "uIP", mainBASIC_WEB_STACK_SIZE, ( void * ) NULL, mainUIP_TASK_PRIORITY, NULL );
//	xTaskCreate( vMBTCPSlaveTask, ( signed char * ) "FMB-TCP", configMINIMAL_STACK_SIZE*3, ( void * ) NULL, mainMB_TASK_PRIORITY, NULL );
	xTaskCreate( vMBRTUSlaveTask, ( signed char * ) "FMB-RTU", configMINIMAL_STACK_SIZE*2, ( void * ) NULL, mainMB_TASK_PRIORITY, NULL );
	
//	xTaskCreate( vMassFlow_Task, ( signed portCHAR * ) "GL696H", mainBASIC_GL696H_STACK_SIZE, NULL, mainGL696H_TASK_PRIORITY, NULL );
	xTaskCreate( vGL696H_Task, ( signed portCHAR * ) "GL696H", mainBASIC_GL696H_STACK_SIZE, NULL, mainGL696H_TASK_PRIORITY, NULL );
//	xTaskCreate( vGL696H_Test_Task, ( signed portCHAR * ) "GL696H", mainBASIC_GL696H_STACK_SIZE, NULL, mainGL696H_TASK_PRIORITY, NULL );
	
//	Win_Init();

	/* The suicide tasks must be created last as they need to know how many
	tasks were running prior to their creation in order to ascertain whether
	or not the correct/expected number of tasks are running at any given time. */
    vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );
	
	/* Start the scheduler. */
	vTaskStartScheduler();
	
	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return 0;
}


