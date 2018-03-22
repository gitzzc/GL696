/* Includes ------------------------------------------------------------------*/

#include "stdlib.h"
#include "stdio.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "lcd.h"
#include "adc.h"
#include "keyboard.h"
#include "window.h"
#include "win_main.h"
#include "win_dccs.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void win_DC_test_refresh(void);
void win_DC_test_msg(uint16_t msg);

sWindow win_DC_test = {
	win_DC_test_msg,0,0
};


void win_DC_test_draw(void)
{
	LCD_Clear();
	win_DC_test_refresh();
}

void DC_main(void)
{
	char str[64];
	uint16_t i = 0,x,y;
	static uint16_t test=0;

//	LCD_DrawRect(0,0,320,240);

	LCD_SetCursor(2,3+22*i++); 
	LCD_SetFont_EN(&EN_Font8x16);
	LCD_SetFont_CH(&CH_Font16x16);
	
	LCD_DisplayString("£±.æ¯‘µµÁ◊Ë≤‚ ‘:               ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("£≤.µÁµ„ªÕ∑≤‚ ‘:           ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("£≥.ºÏ¬©µÁ◊Ë≤‚ ‘:               ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("£¥.º”Œ¬µÁ◊Ë≤‚ ‘:           ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("£µ.µÁ≥ÿº”Œ¬ ±º‰≤‚ ‘:           ");
	LCD_SetCursor(2,3+22*i++); 
}


void win_DC_test_msg(uint16_t msg)
{
	static uint8_t enter = 0,para = 0;

	switch (msg){
	case MSG_WIN_NULL:
		return;
	case MSG_WIN_DRAW:
		win_DC_test_draw();
		break;
	case MSG_WIN_REDRAW:
		win_DC_test_draw();
		break;
	case MSG_WIN_REFRESH:
		win_DC_test_refresh();
		break;
	case MSG_KEY_F3:
		Win_SetFront(&win_main);
		break;
	default:
		break;
	}
}

