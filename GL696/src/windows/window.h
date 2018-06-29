/*
    
 *
 */


#ifndef __window_h__
#define __window_h__

#define WINDOW_SEC	 		(5)
#define WIN_REFRESH_TIME 	(HZ_TICK/WINDOW_SEC)
#define WIN_IDLE_TIME		(60*HZ)//10sec

#define MSG_WIN_MASK	(1<<14)
#define MSG_WIN_NULL	(0)
#define MSG_WIN_UP		(MSG_WIN_MASK+1)
#define MSG_WIN_DOWN	(MSG_WIN_MASK+2)
#define MSG_WIN_LEFT	(MSG_WIN_MASK+3)
#define MSG_WIN_RIGHT	(MSG_WIN_MASK+4)
#define MSG_WIN_ENTER	(MSG_WIN_MASK+5)
#define MSG_WIN_CANCEL	(MSG_WIN_MASK+6)

#define MSG_WIN_REFRESH	(MSG_WIN_MASK+0xF0)
#define MSG_WIN_DRAW		(MSG_WIN_MASK+0xF1)
#define MSG_WIN_REDRAW	(MSG_WIN_MASK+0xF2)

//----menu message---------------------
#define MSG_MENU_MASK			(1<<12)



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
	void (*Dispatch)(unsigned short);
	psMenuItem psMenu;
	uint8_t curMenu;
} sWindow,*psWindow;



extern psWindow pCurWin;

unsigned portBASE_TYPE Win_Init(void);
void Win_SetFront(psWindow pswin);
void LineMenu_Task( psMenu menu ,uint16_t msg);
unsigned portBASE_TYPE Win_GetMsg(uint16_t *msg, portTickType xBlockTime );
unsigned portBASE_TYPE Win_PutMsg(uint16_t msg);

//portTASK_FUNCTION( WinTask, pvParameters );
void Win_Task(void);


#endif
