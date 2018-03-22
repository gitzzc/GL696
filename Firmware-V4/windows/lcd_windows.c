#include "../common.h"
#include "../timer.h"
#include "../key.h"
#include "../sed1335.h"


#include "windows.h"

#include "win_main.h"
#include "win_alarm.h"
#include "win_channel0.h"
#include "win_channel1.h"
#include "win_channel2.h"
#include "win_channel3.h"
#include "win_config.h"



#define WIN_MSG_QSIZE 8
static xQueueHandle xWinMsg_Queue;
static PSWINDOW pCurWin;

unsigned portBASE_TYPE Win_GetMsg(uint16_t *msg, portTickType xBlockTime )
{
	if( xQueueReceive( xWinMsg_Queue, msg, xBlockTime ) )
		return pdTRUE;
	else
		return pdFALSE;
}


unsigned portBASE_TYPE Win_PutMsg(uint16_t msg)
{
	if( xQueueSend( xWinMsg_Queue, &msg, 0 ) != pdPASS )
		return pdFALSE;
	else
		return pdTRUE;	
}

unsigned portBASE_TYPE Win_Init(void)
{
	/* Create the queues */
	xWinMsg_Queue = xQueueCreate( WIN_MSG_QSIZE, ( portBASE_TYPE ) sizeof( uint16_t ) );
	if ( xWinMsg_Queue == 0 ){
		return pdFALSE;
	}
	return pdTRUE;

	pCurWin = &win_main;
	Win_put_msg(MSG_DRAW);
}


void Win_MenuDraw(uint8_t index)
{
	LCD_SetCursor(pCurWin->psMenu[index].XPos,pCurWin->psMenu[index].YPos);
	LCD_Printf(pCurWin->psMenu[index].Text);
}

void Win_MenuTask(uint16_t msg)
{
	if ( msg == 0 || pCurWin->psMenu == NULL )
		return;
		
	if (msg != MSG_KEY_UP 	&& msg != MSG_KEY_DOWN 	&&
		msg != MSG_KEY_ENTER&& msg != MSG_KEY_CANCEL&&
		msg != MSG_DRAW )
		return;
	
	switch(msg){
	case MSG_DRAW:
		for(i=0;pCurWin->psMenu[i].msg != MSG_NULL;i++){
			Win_MenuDraw(i);
		}
		break;
	case MSG_MENU_UP:
		if ( pCurWin->curMenu ){
			Win_MenuDraw(pCurWin->curMenu);
			LCD_SetColor(1);
			Win_MenuDraw(pCurWin->curMenu-1);
			LCD_SetColor(0);
		}
		break;
	case MSG_MENU_DOWN:
		if ( pCurWin->curMenu[pCurWin->curMenu+1].msg != MSG_NULL ){
			Win_MenuDraw(pCurWin->curMenu);
			LCD_SetColor(1);
			Win_MenuDraw(pCurWin->curMenu+1);
			LCD_SetColor(0);
		}
		break;
	case MSG_MENU_ENTER:
		lcd_put_msg(pCurWin->psMenu[pCurWin->curMenu].msg)
		break;
	case MSG_MENU_CANCEL:
		break;
	default:
		if ( msg & MENU_SHORTCUT_MASK ){
			for(i=0;pCurWin->psMenu[i].msg != MSG_NULL;i++){
				if ( pCurWin->psMenu[i].msg == msg ){
					LCD_SetColor(1);
					Win_MenuDraw(i);
					LCD_SetColor(0);
					lcd_put_msg(pCurWin->psMenu[pCurWin->curMenu].msg)
				}
			}
		}
		break;
	}

	return;
}

static portTASK_FUNCTION( WinTask, pvParameters )
{
	uint16_t msg;
	unsigned char temp;
	//static unsigned char Win_sleep = 0;

	( void ) pvParameters;
	
	if ( Win_Init() == pdFALSE )
		return pdFALSE;
		
	while( 1 ){
		if ( KB_Get(&msg,10) == pdTRUE ){
			//beep_set(ON);
			//delayms(10);
			//beep_set(OFF);
			//beep_mute(MUTE);

			if ( pCurWin->Dispatch ) {
				pCurWin->Dispatch( msg );
				Win_MenuTask( msg );
			}
		} else {
			if ( pCurWin->Dispatch ) {
				pCurWin->Dispatch( MSG_REFRESH );
				Win_MenuTask( msg );
			}
		}
	}
	return pdFALSE;
}



