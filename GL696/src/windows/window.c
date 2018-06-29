/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include "projdefs.h"
#include "common.h"
#include "window.h"
#include "win_main.h"

#include "keyboard.h"
#include "lcd.h"
#include "timer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
    
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//---------------------------------------------------------------------
#define LCD_MSG_SIZE 8
uint16_t lcd_msg_buf[LCD_MSG_SIZE];
QUEUE lcd_msg_queue;

psWindow pCurWin;

//---------------------------------------------------------------------

unsigned portBASE_TYPE Win_GetMsg(uint16_t *msg, portTickType xBlockTime )
{
	if( dequeue(&lcd_msg_queue,msg) < 0 )
		return pdFALSE;
	else
		return pdTRUE;
}

unsigned portBASE_TYPE Win_PutMsg(uint16_t msg)
{
	if( enqueue(&lcd_msg_queue,msg) < 0 )
		return pdFALSE;
	else
		return pdTRUE;	
}

unsigned portBASE_TYPE Win_Init(void)
{
	/* Create the queues */
	init_queue(&lcd_msg_queue,lcd_msg_buf,LCD_MSG_SIZE);

	pCurWin = &win_main;
	Win_PutMsg(MSG_WIN_DRAW);
	return pdTRUE;
}

void Win_SetFront(psWindow pswin)
{
   pCurWin = pswin;
   Win_PutMsg(MSG_WIN_DRAW);
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
		Win_PutMsg(pCurWin->psMenu[pCurWin->curMenu].user_msg);
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
				Win_PutMsg(pCurWin->psMenu[i].user_msg);
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
				Win_PutMsg( menu->item[i].msg );
				break;
			}
		}
		break;
	}

	menu->first = first;
	menu->curt = curt;
}

void Win_Task(void)
{
	uint16_t msg;
	static uint32_t sec = 0;
	uint32_t ret;
	static sTIMEOUT win_to;

	if ( (ret=get_timeout(&win_to)) == TO_TIMEOUT ){
		start_timeout(&win_to,WIN_REFRESH_TIME);

		while( Win_GetMsg( &msg, 0) ){
 			if ( pCurWin->Dispatch ) {
				pCurWin->Dispatch( msg );
			}
		}
		
		if ( KB_Get(&msg,0) == pdTRUE ){
			if ( pCurWin->Dispatch ) {
				pCurWin->Dispatch( msg | MSG_USER_MASK);
			}
		} 
		
		if ( (sec++ % (WINDOW_SEC)) == 0 ){
			Win_PutMsg(MSG_WIN_REFRESH);
		}
	} else if ( ret != TO_RUNING ){
		start_timeout(&win_to,HZ_TICK/WINDOW_SEC);
	}
}




