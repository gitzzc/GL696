/* Includes ------------------------------------------------------------------*/

#include "stdlib.h"
#include "stdio.h"
#include "string.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "keyboard.h"
#include "lcd.h"
#include "touchscreen.h"
#include "window.h"
#include "win_main.h"
#include "iic_eeprom.h"
#include "spi_flash.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
    
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


#define WIN_MSG_QSIZE 8

#define TC_TASK_PRIORITY    				( tskIDLE_PRIORITY + 3 )
#define WINDOW_TASK_PRIORITY    			( tskIDLE_PRIORITY + 2 )

xQueueHandle xMSGQueue;
static psWindow pCurWin;

portBASE_TYPE Win_GetMsg(HIDMessage *msg, portTickType xBlockTime )
{
	if( xQueueReceive( xMSGQueue, msg, xBlockTime ) )
		return pdTRUE;
	else
		return pdFALSE;
}

portBASE_TYPE Win_PutMsg(HIDMessage *msg)
{
	if( xQueueSend( xMSGQueue, msg, 0 ) != pdPASS )
		return pdFALSE;
	else
		return pdTRUE;	
}

portBASE_TYPE Win_InitMsg(void)
{
	/* Create the queues */
	xMSGQueue = xQueueCreate( WIN_MSG_QSIZE, ( portBASE_TYPE ) sizeof( HIDMessage ) );
	if ( xMSGQueue == 0 ){
		return pdFALSE;
	}
	return pdTRUE;	
}

portBASE_TYPE Win_PutWinMsg(uint16_t winid)
{
	HIDMessage msg;
		
	msg.type = HID_WINDOW;
	msg.id = winid;
	return (Win_PutMsg(&msg));	
}


portBASE_TYPE Win_Init(void)
{
	//prvConfigureLCD();

	//LCD_DrawMonoPict( ( unsigned portLONG * ) pcBitmap );
	/* Set the Back Color */
	//LCD_SetBackColor( Black );
	/* Set the Text Color */
	//LCD_SetTextColor( 0x421F );

	/* Create the queues */
	if ( Win_InitMsg() == pdFALSE )
		return pdFALSE;

	pCurWin = &win_main;
	Win_PutWinMsg(MSG_WIN_DRAW);

	xTaskCreate( vTouchTask	, ( signed char * ) "TouchScreen", configMINIMAL_STACK_SIZE*2, ( void * ) NULL, TC_TASK_PRIORITY, &xTCTaskHandle );
	xTaskCreate( WinTask	, ( signed char * ) "Windows"	 , configMINIMAL_STACK_SIZE*4, ( void * ) NULL, WINDOW_TASK_PRIORITY, NULL );

	return pdTRUE;
}

void Win_SetFront(psWindow pswin)
{
   pCurWin = pswin;
   Win_PutWinMsg(MSG_WIN_DRAW);
}

void Win_MenuDraw(uint8_t index)
{
	LCD_SetCursor(pCurWin->psMenu[index].XPos,pCurWin->psMenu[index].YPos);
	LCD_DisplayString(pCurWin->psMenu[index].Text);
}

void Win_MenuTask(uint16_t msg)
{
	portBASE_TYPE i;

	if ( msg == 0 || pCurWin->psMenu == NULL )
		return;

	switch(msg){
	case MSG_WIN_DRAW:
		for(i=0;pCurWin->psMenu[i].user_msg != MSG_WIN_NULL;i++){
			Win_MenuDraw(i);
		}
		break;
/*	case MSG_WIN_UP:
		if ( pCurWin->curMenu ){
			Win_MenuDraw(pCurWin->curMenu--);
			LCD_SetTextColor(0);
			Win_MenuDraw(pCurWin->curMenu);
			LCD_SetTextColor(1);
		}
		break;
	case MSG_WIN_DOWN:
		if ( pCurWin->psMenu[pCurWin->curMenu+1].user_msg != MSG_WIN_NULL ){
			Win_MenuDraw(pCurWin->curMenu++);
			LCD_SetTextColor(0);
			Win_MenuDraw(pCurWin->curMenu);
			LCD_SetTextColor(1);
		}
		break;
*/	case MSG_WIN_ENTER:
		Win_PutWinMsg(pCurWin->psMenu[pCurWin->curMenu].user_msg);
		break;
	case MSG_WIN_CANCEL:
		break;
	default:
		for(i=0;pCurWin->psMenu[i].key_msg != MSG_WIN_NULL;i++){
			if ( pCurWin->psMenu[i].key_msg == msg ){
				pCurWin->curMenu = i;
				LCD_SetTextColor(0);
				Win_MenuDraw(i);
				LCD_SetTextColor(1);
				Win_PutWinMsg(pCurWin->psMenu[i].user_msg);
				break;
			}
		}
		if ( msg & MSG_KEY_BREAK ){
			LCD_SetTextColor(1);
			Win_MenuDraw(pCurWin->curMenu);
		}
		break;
	}

	return;
}

void LineMenu_repaint(psMenu menu,uint8_t index)
{
	if ( menu->curt == index )
		LCD_SetTextColor(0);
	if ( menu->item[index].paint ){
		LCD_SetCursor(menu->startX + strlen(menu->item[index].Text),menu->startY);
		menu->item[index].paint();
	}
	if ( menu->curt == index )
		LCD_SetTextColor(1);
	
}

void LineMenu_Draw(psLineMenuItem item )
{
	LCD_DisplayString(item->Text);
	if ( item->paint != NULL )
		item->paint();
}

void LineMenu_Task( psMenu menu ,uint16_t msg)
{
	portBASE_TYPE i;
	portBASE_TYPE first,curt;

	if ( msg == 0 )
		return;
		
	first = menu->first;
	curt = menu->curt;

	switch(msg){
	case MSG_WIN_DRAW:
		LCD_SetCursor(menu->startX,menu->startY);
		for(i=first;i<first+menu->lines && menu->item[i].Text != NULL;i++){
			if ( curt == i ){
				LCD_SetTextColor(0);
				LineMenu_Draw(&menu->item[i]);
				LCD_SetTextColor(1);
			} else {
				LineMenu_Draw(&menu->item[i]);
			}
			LCD_SetCursor(menu->startX,LCD_GetCursorY()+16);
		}
		break;
	case MSG_WIN_REFRESH:
		LCD_SetCursor(menu->startX,menu->startY+(curt-first)*16);
		LCD_SetTextColor(0);
		LineMenu_Draw(&menu->item[curt]);
		LCD_SetTextColor(1);
		break;	
	case MSG_WIN_UP:
		if ( curt == 0 )
			break;
		LCD_SetCursor(menu->startX,menu->startY+(curt-first)*16);
		if ( curt > first && curt < first + menu->lines ){
			LineMenu_Draw(&menu->item[curt--]);
			LCD_SetTextColor(0);
			LCD_SetCursor(menu->startX,LCD_GetCursorY()-16);
			LineMenu_Draw(&menu->item[curt]);
			LCD_SetTextColor(1);
		} else if ( curt == first ) {
			curt --;
			LCD_SetCursor(menu->startX,menu->startY);
			LCD_SetTextColor(0);
			LineMenu_Draw(&menu->item[curt]);
			LCD_SetTextColor(1);
			for(i=1;i<menu->lines;i++){
				LCD_SetCursor(menu->startX,LCD_GetCursorY()+16);
				LineMenu_Draw(&menu->item[i]);
			}
			first--;
		} else {
			curt = 0;
			first = 0;
		}
		break;
	case MSG_WIN_DOWN:
		if ( curt >= first && curt < first + menu->lines - 1 ){
			LCD_SetCursor(menu->startX,menu->startY+(curt-first)*16);
			LineMenu_Draw(&menu->item[curt++]);
			LCD_SetTextColor(0);
			LCD_SetCursor(menu->startX,LCD_GetCursorY()+16);
			LineMenu_Draw(&menu->item[curt]);
			LCD_SetTextColor(1);
		} else if ( curt == first + menu->lines - 1 ) {
			if ( menu->item[curt+1].Text != NULL ){
				curt++;
				LCD_SetCursor(menu->startX,menu->startY);
				for(i=0;i<menu->lines-1;i++){
					LineMenu_Draw(&menu->item[curt-menu->lines+1+i]);
					LCD_SetCursor(menu->startX,LCD_GetCursorY()+16);
				}
				LCD_SetTextColor(0);
				LineMenu_Draw(&menu->item[curt]);
				LCD_SetCursor(menu->startX,LCD_GetCursorY()+16);
				LCD_SetTextColor(1);
				first ++;
			}
		} else {
			curt = 0;
			first = 0;
		}		
		break;
	case MSG_WIN_ENTER:
		//Win_PutMsg(menu[curt].user_msg);
		break;
	case MSG_WIN_CANCEL:
		break;
	default:
		for(i=0;menu->item[i].Text != NULL;i++){
			if ( menu->item[i].shortcut == msg && menu->item[i].msg ){
				Win_PutWinMsg( menu->item[i].msg );
				break;
			}
		}
		break;
	}

	menu->first = first;
	menu->curt = curt;

}

portBASE_TYPE Button_Check(sButton* btn,uint16_t num,HIDMessage* msg)
{
	portBASE_TYPE i,ret = 0;
	
	if ( msg->type != HID_TOUCHSCREEN )
		return FALSE;
		
	for(i=0;i<num;i++){
		if ((msg->x >= btn[i].x) && (msg->x <= btn[i].x + btn[i].width) && \
			(msg->y >= btn[i].y) && (msg->y <= btn[i].y + btn[i].height) ) {
			return btn[i].msg_id;
		}
	}
	return 0;
}


portTASK_FUNCTION( WinTask, pvParameters )
{
	HIDMessage msg;
 	portTickType xLastWakeTime,refresh_time;
	unsigned portBASE_TYPE sec = 0;
	char str[32];

	( void ) pvParameters;
	
	vTaskDelay( configTICK_RATE_HZ*2 );
	prvConfigureLCD();

	//if ( Win_Init() == pdFALSE )
	//	return ;
	
	TouchScreen_Calibrate(0);
	
	Win_PutWinMsg(MSG_WIN_DRAW);
		
	xLastWakeTime   = xTaskGetTickCount ();
	refresh_time	= xLastWakeTime;
	
	while( 1 ){
		if ( Win_GetMsg( &msg, configTICK_RATE_HZ/100 ) == pdPASS ){
 			if ( pCurWin->Dispatch ) {
				pCurWin->Dispatch( &msg );
			}
		} else if ( xLastWakeTime - refresh_time > WIN_REFRESH_TIME ){
			refresh_time = xLastWakeTime;
			Win_PutWinMsg(MSG_WIN_REFRESH);
		}

		vTaskDelayUntil( &xLastWakeTime, WIN_IDLE_TIME );
	}
}



