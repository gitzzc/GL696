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
#include "ups.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void win_BXJG_test_refresh(void);
void win_BXJG_test_msg(uint16_t msg);


uint32_t ups_lcd_bright=0xFFFF;

sWindow win_BXJG_test = {
	win_BXJG_test_msg,0,0
};


void win_BXJG_test_draw(void)
{
	char str[64];
	uint16_t x=0,y=0;

	x = 0; y = 0;
	LCD_Clear();
	LCD_SetCursor(x,y);

	win_BXJG_test_refresh();
}

void win_BXJG_test_refresh(void)
{
	char str[64];
	uint16_t buf[16];
	uint16_t i = 0,x,y;
	static uint16_t test=0;

//	LCD_DrawRect(0,0,320,240);

	LCD_SetFont_EN(&EN_Font8x16);
	LCD_SetFont_CH(&CH_Font20x20);
	
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("                        　Ⅰ组   Ⅱ组");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("１.绝缘电阻测试:               ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("２.独立电路间绝缘性:           ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("３.保险功能测试:               ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("４.电磁绕圈电阻测试:           ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("５.二极管良好性测试:           ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("６.解保功能测试:               ");
	LCD_SetCursor(2,3+22*i++); 
	LCD_DisplayString("７.火工品电阻测试:             ");
	
}


void win_BXJG_test_msg(uint16_t msg)
{
	static uint8_t enter = 0,para = 0;
	static uint8_t* buf;
	static uint32_t bufsize;

	switch (msg){
	case MSG_WIN_NULL:
		return;
	case MSG_WIN_DRAW:
		win_BXJG_test_draw();
		break;
	case MSG_WIN_REDRAW:
		win_BXJG_test_draw();
		break;
	case MSG_WIN_REFRESH:
		win_BXJG_test_refresh();
		break;
	case MSG_KEY_F3:
		//Win_SetFront(&win_main);
		break;
	default:
		break;
	}

}

