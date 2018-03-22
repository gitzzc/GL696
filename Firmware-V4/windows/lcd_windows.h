/*
 *	Copyright (c) 2006,北京意科通信技术有限责任公司
 *	All ringts reserved.
 *
 *	当前版本：0.11
 *	作    者：ZZC@SINA.COM
 *	完成日期：
 *	编译环境：avr-gcc-3.4.5,avr-lib-1.4.1
 *	文件名称：adc.h
 *	摘    要：交流检测板ADC电路相关定义
 *
 *
 *	ver 0.11
 *	增加校准参数CRC值，并增加一校准参数备份，如果第一个参数块损坏，读取第二块。
 *          
 *
 */


#ifndef __lcd_windows_h__
#define __lcd_windows_h__

#include "../key.h"
#include "../sed1335.h"
#include "../graphic.h"
#include "../dio.h"




#define LCD_REFRESH_TIME 	(1*HZ)
#define LCD_IDLE_TIME			(60*HZ)//10sec


#define MSG_NULL			0
#define MSG_KEY_UP		KEY_UP
#define MSG_KEY_DOWN	KEY_DOWN
#define MSG_KEY_LEFT	KEY_LEFT
#define MSG_KEY_RIGHT	KEY_RIGHT
#define MSG_KEY_ENTER	KEY_ENTER
#define MSG_KEY_CANCEL	KEY_CANCEL

#define MSG_REFRESH	0xF0
#define MSG_DRAW		0xF1
#define MSG_REDRAW	0xF2
#define TIMER_1S		0xF3



typedef struct _WINDOW_{
	void (*Draw)(void);
	void (*Dispatch)(unsigned short);
	PSMenu psMenu;
	uint8_t curMenu;
} WINDOW,*PWINDOW;


typedef struct _MENU_ITEM_
{
	uint16_t msg;
	uint8_t* Text;
	uint16_t XPos;
	uint16_t YPos;
	uint16_t shortcut;
} SMENUITEM,*PSMENUITE;


extern PWINDOW* pCurWin;

void Win_Init(void);

#endif
