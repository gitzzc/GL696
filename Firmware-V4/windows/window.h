/*
    
 *
 */


#ifndef __window_h__
#define __window_h__

#include "FreeRTOS.h"

#define WINDOW_SEC 			100
#define WIN_REFRESH_TIME 	(configTICK_RATE_HZ/2)
#define WIN_IDLE_TIME		(configTICK_RATE_HZ/100)//10msec

#define MSG_WIN_MASK	(1<<14)
#define MSG_WIN_NULL	(0)
#define MSG_WIN_UP		(MSG_WIN_MASK+1)
#define MSG_WIN_DOWN	(MSG_WIN_MASK+2)
#define MSG_WIN_LEFT	(MSG_WIN_MASK+3)
#define MSG_WIN_RIGHT	(MSG_WIN_MASK+4)
#define MSG_WIN_ENTER	(MSG_WIN_MASK+5)
#define MSG_WIN_CANCEL	(MSG_WIN_MASK+6)

#define MSG_WIN_REFRESH	(MSG_WIN_MASK+0xF0)
#define MSG_WIN_DRAW	(MSG_WIN_MASK+0xF1)
#define MSG_WIN_REDRAW	(MSG_WIN_MASK+0xF2)

//----user-defined message---------------------
#define MSG_USER_MASK			(1<<13)
#define MSG_USER_HVL_SETV		(MSG_USER_MASK+1)
#define MSG_USER_HVL_SETC		(MSG_USER_MASK+2)
#define MSG_USER_MAIN_SETUP		(MSG_USER_MASK+3)
#define MSG_USER_SETUP_UP		(MSG_USER_MASK+4)
#define MSG_USER_SETUP_DOWN		(MSG_USER_MASK+5)
#define MSG_USER_SETUP_CANCEL	(MSG_USER_MASK+6)
#define MSG_USER_SETUP_ENTER	(MSG_USER_MASK+7)
#define MSG_USER_PARA_UP		(MSG_USER_MASK+8)
#define MSG_USER_PARA_DOWN		(MSG_USER_MASK+9)
#define MSG_USER_PARA_CANCEL	(MSG_USER_MASK+10)
#define MSG_USER_PARA_SAVE		(MSG_USER_MASK+11)

//----menu message---------------------
#define MSG_MENU_MASK			(1<<12)

//-----------------------------------------

#define HID_WINDOW			1
#define HID_KEYBOARD		2
#define HID_TOUCHSCREEN		3
#define HID_TC_CALIBRATE	4
#define HID_MOUSE			5

#define HID_TC_UP				1
#define HID_TC_DOWN 			2
#define HID_TC_FLEETING 		3

typedef struct
{
	unsigned char	type;
	unsigned short 	id;
	unsigned short 	raw_x;
	unsigned short 	raw_y;
	unsigned short 	x;
	unsigned short 	y;
	unsigned short 	key;
	unsigned short 	fn;
} HIDMessage;

typedef struct S_BUTTON
{
	uint16_t x,y;
	uint16_t width,height;
	const char* text;
	const char* image;
	uint16_t msg_id;
} sButton;


typedef struct
{
	uint16_t user_msg;
	char* Text;
	uint16_t XPos;
	uint16_t YPos;
	uint16_t key_msg;
} sMenuItem,*psMenuItem;

typedef struct 
{
	uint16_t msg;
	char* Text;
	uint16_t shortcut;
	void (*paint)(void);
} sLineMenuItem,*psLineMenuItem;


typedef struct 
{
	uint8_t attr;
	uint8_t curt;
	uint8_t first;
	uint8_t lines;
	uint16_t startX;
	uint16_t startY;
	psLineMenuItem item;
} sMenu,*psMenu;

typedef struct _SWINDOW_{
	void (*Dispatch)(HIDMessage* msg);
	psMenuItem psMenu;
	uint8_t curMenu;
} sWindow,*psWindow;


extern xQueueHandle xMSGQueue;
extern psWindow pCurWin;

portBASE_TYPE Win_Init(void);
portBASE_TYPE Win_PutMsg(HIDMessage *msg);
portBASE_TYPE Win_GetMsg(HIDMessage *msg, portTickType xBlockTime );
void Win_SetFront(psWindow pswin);
void LineMenu_Task( psMenu menu ,uint16_t msg);
portBASE_TYPE Button_Check(sButton* btn,uint16_t num,HIDMessage* msg);

portTASK_FUNCTION( WinTask, pvParameters );

#endif
