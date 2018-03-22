/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "window.h"
#include "win_main.h"

#include "lcd.h"
#include "fonts.h"
#include "adc.h"
#include "keyboard.h"
#include "gl_696h.h"
#include "modbus.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


static sButton btns[] = {
	{16*10,32*4+2,16*6,32,NULL,NULL,BTN_HVL_SETV},//160,130
	{16*10,32*5+2,16*6,32,NULL,NULL,BTN_HVL_SETC},//160,162
	{16*25,32*4+2,16*6,32,NULL,NULL,BTN_HVR_SETV},//
	{16*25,32*5+2,16*6,32,NULL,NULL,BTN_HVR_SETC},//400,162 - 496,194
	{16*23,32*9+2,16*7,32*2," 增 加 ",NULL,BTN_UP},
	{16*30,32*9+2,16*7,32*2," 减 小 ",NULL,BTN_DOWN},
	{16*5 ,32*9+2,16*7,32*2,NULL,NULL,BTN_START_STOP},
};




static portBASE_TYPE page=0;    
uint32_t hv_set_flag=0;
uint32_t hv_ctrl_flag=0;
uint32_t hv_start_stop = 0;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void main_win_refresh(void);
void main_win_msg(HIDMessage* msg);


sWindow win_main = {
	main_win_msg,0,0
};


void main_win_draw(void)
{
//	char str[64];
	uint16_t x,y,i;

//	LCD_Clear();
	LCD_SetTextColor( 0x421F );
	LCD_SetFont_EN(&EN_Font16x32);
	LCD_SetFont_CH(&CH_Font32x32);

//	for(i=0;i<2;i++)
	{
		//if ( i )
			LCD_SET_PAGE(LCD_WRITE_PAGE0 | LCD_DISPLAY_PAGE1);
		//else
		//	LCD_SET_PAGE(LCD_WRITE_PAGE1 | LCD_DISPLAY_PAGE0);
	
		LCD_SetCursor(x=2,y=2);
		LCD_DisplayString("         AL-901 真 空 清 洁 仪         \r\n\r\n");
		
		LCD_DisplayString("            左     枪      右     枪   \r\n");
		LCD_DisplayString("          设定值 实际值  设定值 实际值 \r\n");
		LCD_DisplayString(" 电压(KV): 0.0    0.0     0.0    0.0   \r\n");
		LCD_DisplayString(" 电流(mA): 0.00   0.00    0.00   0.00  \r\n");
		LCD_DisplayString(" 真空度: 1.00E+00  分子泵频率:   0     \r\n");
		LCD_DisplayString(" 本次已经减薄时间: 00:00:00            \r\n");
		LCD_DisplayString(" 自动停机时间:     01:00:00            \r\n");
		
		LCD_SetCursor(0,LCD_GetCursorY()+16);
		LCD_DisplayString("      启 动             增 加  减 小   \r\n\r\n");
	
	
		LCD_DisplayString("   北京艾博智业离子技术有限责任公司    \r\n");
		LCD_DisplayString("  客服电话: 010-12345678 13812345678   \r\n");
	}
	LCD_SET_PAGE(LCD_WRITE_PAGE0 | LCD_DISPLAY_PAGE0);
	
	//LCD_DrawRect(0,0,LCD_SCR_WIDTH,LCD_SCR_HIGH);
}

void main_win_refresh(void)
{
	char str[32];
	uint16_t sample;
	uint16_t x,y,i;
	static uint32_t page = 0;

	for(i=1;i<5;i++){
		portENTER_CRITICAL();
		if ( i == BTN_HVL_SETV )
			sprintf(str," %2.1f ",(float)hvsl.vol_set/1000);
		else if ( i == BTN_HVL_SETC )
			sprintf(str," %1.2f ",(float)hvsl.cur_set/10000);
		else if ( i == BTN_HVR_SETV )
			sprintf(str," %2.1f ",(float)hvsr.vol_set/1000);
		else if ( i == BTN_HVR_SETC )
			sprintf(str," %1.2f ",(float)hvsr.cur_set/10000);
		portEXIT_CRITICAL();
		if ( i == hv_set_flag )
			LCD_SetTextColor(Red);
		LCD_SetCursor(btns[i-1].x,btns[i-1].y);
		LCD_DisplayString(str);
		LCD_SetTextColor( 0x421F );
	}
	
	portENTER_CRITICAL();
	sprintf(str," %2.1f ",(float)hvsl.vol_fb/1000);
	portEXIT_CRITICAL();
	LCD_SetCursor(16*17,32*4+2);
	LCD_DisplayString(str);

	portENTER_CRITICAL();
	sprintf(str," %1.2f ",(float)hvsl.cur_fb/10000);
	portEXIT_CRITICAL();
	LCD_SetCursor(16*17,32*5+2);
	LCD_DisplayString(str);

	portENTER_CRITICAL();
	sprintf(str," %2.1f ",(float)hvsr.vol_fb/1000);
	portEXIT_CRITICAL();
	LCD_SetCursor(16*32,32*4+2);
	LCD_DisplayString(str);

	portENTER_CRITICAL();
	sprintf(str," %1.2f ",(float)hvsr.cur_fb/10000);
	portEXIT_CRITICAL();
	LCD_SetCursor(16*32,32*5+2);
	LCD_DisplayString(str);
	
	portENTER_CRITICAL();
	sprintf(str," %.2E  ",(float)vmeter/1000);
	portEXIT_CRITICAL();
	LCD_SetCursor(16*8,32*6+2);
	LCD_DisplayString(str);

	portENTER_CRITICAL();
	sprintf(str," %3d ",usRegInputBuf[MB_MPUMP_FREQ]);
	portEXIT_CRITICAL();
	LCD_SetCursor(16*30,32*6+2);
	LCD_DisplayString(str);
}


void main_win_msg(HIDMessage* msg)
{
	uint8_t dev_code;
	char str[32];
	uint32_t i;

	if ( msg->type == HID_WINDOW ){
		switch (msg->id){
		case MSG_WIN_NULL:
			return;
		case MSG_WIN_DRAW:
			main_win_draw();
			break;
		case MSG_WIN_REDRAW:
			main_win_draw();
			break;
		case MSG_WIN_REFRESH:
			main_win_refresh();
			break;
		default:
			break;
		}
	} else if ( msg->type == HID_KEYBOARD ) { 
	} else if ( msg->type == HID_TOUCHSCREEN ) { 
		uint16_t btn;

		if ( msg->id == HID_TC_CALIBRATE ){
			TouchScreen_Calibrate(1);
			main_win_draw();
		}

		btn = Button_Check(btns,sizeof(btns)/sizeof(btns[0]),msg);
		if ( btn >= BTN_HVL_SETV && btn <= BTN_HVR_SETC ){
			if ( msg->id == HID_TC_DOWN ){
				hv_set_flag = btn;
				Win_PutWinMsg(MSG_WIN_REFRESH);
			}
		} else if ( btn >= BTN_UP && btn <= BTN_DOWN ){
			if ( msg->id == HID_TC_DOWN || msg->id == HID_TC_FLEETING ){
				hv_ctrl_flag = btn;
				
				if ( hv_set_flag ) {
					portENTER_CRITICAL();
					switch( hv_set_flag ){
					case BTN_HVL_SETV:	
						if ( btn == BTN_UP )
							hvsl.vol_set = hvsl.vol_set+100 > hvsl.vol_max ? hvsl.vol_max : hvsl.vol_set+100;
						else
							hvsl.vol_set = hvsl.vol_set > 100 ? hvsl.vol_set - 100 : 0;
						sprintf(str," %2.1f ",(float)hvsl.vol_set/1000);
						break;
					case BTN_HVL_SETC:	
						if ( btn == BTN_UP )
							hvsl.cur_set = hvsl.cur_set+100 > hvsl.cur_max ? hvsl.cur_max : hvsl.cur_set+100;
						else
							hvsl.cur_set = hvsl.cur_set > 100 ? hvsl.cur_set - 100 : 0;
						sprintf(str," %1.2f ",(float)hvsl.cur_set/10000);
						break;
					case BTN_HVR_SETV:	
						if ( btn == BTN_UP )
							hvsr.vol_set = hvsr.vol_set+100 > hvsr.vol_max ? hvsr.vol_max : hvsr.vol_set+100;
						else
							hvsr.vol_set = hvsr.vol_set > 100 ? hvsr.vol_set - 100 : 0;
						sprintf(str," %2.1f ",(float)hvsr.vol_set/1000);
						break;
					case BTN_HVR_SETC:
						if ( btn == BTN_UP )
							hvsr.cur_set = hvsr.cur_set+100 > hvsr.cur_max ? hvsr.cur_max : hvsr.cur_set+100;
						else
							hvsr.cur_set = hvsr.cur_set > 100 ? hvsr.cur_set - 100 : 0;
						sprintf(str," %1.2f ",(float)hvsr.cur_set/10000);
						break;
					default :	break;
					}
					portEXIT_CRITICAL();

					LCD_SetTextColor(Red);
					LCD_SetCursor(btns[hv_set_flag-1].x,btns[hv_set_flag-1].y);
					LCD_DisplayString(str);
				}					
				LCD_SetTextColor(Red);
				LCD_SetCursor(btns[btn-1].x,btns[btn-1].y+16);
				LCD_DisplayString(btns[btn-1].text);
				LCD_SetTextColor( 0x421F );
			} else if ( msg->id == HID_TC_UP ){
				LCD_SetCursor(btns[btn-1].x,btns[btn-1].y+16);
				LCD_DisplayString(btns[btn-1].text);
			}	
		} else if ( btn == BTN_START_STOP ){
			if ( msg->id == HID_TC_DOWN ){
				LCD_SetTextColor(Red);
				if ( hv_start_stop ){
					hv_start_stop = 0;
					LCD_SetCursor(btns[btn-1].x,btns[btn-1].y+16);
					LCD_DisplayString(" 启 动 ");
					portENTER_CRITICAL();
					usRegHoldingBuf[MB_SYS_AUTOCTL] = SYS_AUTO_EN | SYS_AUTO_OFF;
					portEXIT_CRITICAL();
				} else {
					hv_start_stop = 1;
					LCD_SetCursor(btns[btn-1].x,btns[btn-1].y+16);
					LCD_DisplayString(" 停 止 ");
					portENTER_CRITICAL();
					usRegHoldingBuf[MB_SYS_AUTOCTL] = SYS_AUTO_EN | SYS_AUTO_ON;
					portEXIT_CRITICAL();
				}
				LCD_SetTextColor( 0x421F );
				Win_PutWinMsg(MSG_WIN_REFRESH);
			} else if ( msg->id == HID_TC_UP ){
				if ( hv_start_stop ){
					LCD_SetCursor(btns[BTN_START_STOP-1].x,btns[BTN_START_STOP-1].y+16);
					LCD_DisplayString(" 停 止 ");
				} else {
					LCD_SetCursor(btns[BTN_START_STOP-1].x,btns[BTN_START_STOP-1].y+16);
					LCD_DisplayString(" 启 动 ");
				}
			}	
		} else {
			if ( msg->id == HID_TC_UP ){
				if ( hv_set_flag > 0 ){
					portENTER_CRITICAL();
					if ( hv_set_flag == BTN_HVL_SETV )
						sprintf(str," %2.1f ",(float)hvsl.vol_set/1000);
					else if ( hv_set_flag == BTN_HVL_SETC )
						sprintf(str," %1.2f ",(float)hvsl.cur_set/10000);
					else if ( hv_set_flag == BTN_HVR_SETV )
						sprintf(str," %2.1f ",(float)hvsr.vol_set/1000);
					else if ( hv_set_flag == BTN_HVR_SETC )
						sprintf(str," %1.2f ",(float)hvsr.cur_set/10000);
					portEXIT_CRITICAL();
					
					LCD_SetCursor(btns[hv_set_flag-1].x,btns[hv_set_flag-1].y);
					LCD_DisplayString(str);
					hv_set_flag = 0;
				}
				
				LCD_SetCursor(btns[BTN_UP-1].x,btns[BTN_UP-1].y+16);
				LCD_DisplayString(btns[BTN_UP-1].text);
				LCD_SetCursor(btns[BTN_DOWN-1].x,btns[BTN_DOWN-1].y+16);
				LCD_DisplayString(btns[BTN_DOWN-1].text);

				if ( hv_start_stop ){
					LCD_SetCursor(btns[BTN_START_STOP-1].x,btns[BTN_START_STOP-1].y+16);
					LCD_DisplayString(" 停 止 ");
				} else {
					LCD_SetCursor(btns[BTN_START_STOP-1].x,btns[BTN_START_STOP-1].y+16);
					LCD_DisplayString(" 启 动 ");
				}
				
				/*LCD_SetCursor(btns[BTN_UP-1].x,btns[BTN_UP-1].y);
				LCD_DisplayString("       ");
				LCD_SetCursor(btns[BTN_DOWN-1].x,btns[BTN_DOWN-1].y);
				LCD_DisplayString("       ");
				*/
			}
		}
	}
}


