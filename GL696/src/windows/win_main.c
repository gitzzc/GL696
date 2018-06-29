/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "projdefs.h"

#include "lcd.h"
#include "adc.h"
#include "keyboard.h"
#include "window.h"
#include "win_main.h"
#include "modbus.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int8_t page=0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void main_win_refresh(void);
void main_win_msg(uint16_t msg);


sWindow win_main = {
	main_win_msg,0,0
};


void main_win_draw(void)
{
	LCD_Clear();
	main_win_refresh();
}

void main_win_refresh(void)
{
	char str[64];
	char* str2="";
	float positive ,negative;
	//uint16_t i;
	
	LCD_SetCursor(0,0);
	
	switch(page){
	case 0:
		LCD_SetCursor(0,LCD_GetCursorY()+8+4);
		sprintf(str,"   CPU ״̬:%s\n",((sSys_cfg.pau_st>>2)&0x01)?" �쳣 ":" ���� ");LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"   �Լ�״̬:%s\n",((sSys_cfg.pau_st>>0)&0x01)?" �쳣 ":" ���� ");LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"   ���Ʒ�ʽ:%s\n",((sSys_cfg.pau_st>>1)&0x01)?" ���� ":" �̿� ");LCD_DisplayString(str);
		break;
	case 1:
		sprintf(str,"ͨ· ��ѹ  ���� ״̬\n");	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.pout>> 0)&0x01)?" ͨ":" ��";
		sprintf(str,"1:%5.1fV, %4.1fA %s\n",(float)sSys_cfg.vol[0]/100,(float)sSys_cfg.cur[0]/100,str2);	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.pout>> 1)&0x01)?" ͨ":" ��";
		sprintf(str,"2:%5.1fV, %4.1fA %s\n",(float)sSys_cfg.vol[1]/100,(float)sSys_cfg.cur[1]/100,str2);	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.pout>> 2)&0x01)?" ͨ":" ��";
		sprintf(str,"3:%5.1fV, %4.1fA %s\n",(float)sSys_cfg.vol[2]/100,(float)sSys_cfg.cur[2]/100,str2);	LCD_DisplayString(str);
		break;
	case 2:
		sprintf(str,"ͨ· ��ѹ  ���� ״̬\n");	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.pout>> 3)&0x01)?" ͨ":" ��";
		sprintf(str,"4:%5.1fV, %4.1fA %s\n",(float)sSys_cfg.vol[3]/100,(float)sSys_cfg.cur[3]/100,str2);	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.pout>> 4)&0x01)?" ͨ":" ��";
		sprintf(str,"5:%5.1fV, %4.1fA %s\n",(float)sSys_cfg.vol[4]/100,(float)sSys_cfg.cur[4]/100,str2);	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.pout>> 5)&0x01)?" ͨ":" ��";
		sprintf(str,"6:%5.1fV, %4.1fA %s\n",(float)sSys_cfg.vol[5]/100,(float)sSys_cfg.cur[5]/100,str2);	LCD_DisplayString(str);
		break;
	case 3:
		sprintf(str,"    ͨ·���״̬\n");	LCD_DisplayString(str);
		str2 = ((sSys_cfg.pout>> 6)&0x01)?"���1 :ͨ, ":"���1 :��, ";LCD_DisplayString(str2);
		str2 = ((sSys_cfg.pout>> 7)&0x01)?"���2 :ͨ\n":"���2 :��\n";	LCD_DisplayString(str2);
		str2 = ((sSys_cfg.pout>> 8)&0x01)?"�ⱬ  :ͨ, ":"�ⱬ  :��, ";LCD_DisplayString(str2);
		str2 = ((sSys_cfg.pout>> 9)&0x01)?"ͨ·10:ͨ\n":"ͨ·10:��\n";	LCD_DisplayString(str2);
		str2 = ((sSys_cfg.pout>>10)&0x01)?"����  :ͨ, ":"����  :��, ";LCD_DisplayString(str2);
		str2 = ((sSys_cfg.pout>>11)&0x01)?"ͨ·12:ͨ\n":"ͨ·12:��\n";	LCD_DisplayString(str2);
		str2 = ((sSys_cfg.pout>>12)&0x01)?"ͨ·13:ͨ, ":"ͨ·13:��, ";LCD_DisplayString(str2);
		str2 = ((sSys_cfg.pout>>13)&0x01)?"ͨ·14:ͨ\n":"ͨ·14:��\n";	LCD_DisplayString(str2);
		break;
	case 4:
		sprintf(str,"    ͨ·����״̬\n");	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.oc>>0)&0x01)?" 1·:����,"		:" 1·:����,";		LCD_DisplayString(str2);
		str2 = ((sSys_cfg.oc>>1)&0x01)?" 2·:���� \n"	:" 2·:���� \n";	LCD_DisplayString(str2);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.oc>>2)&0x01)?" 3·:����,"		:" 3·:����,";		LCD_DisplayString(str2);
		str2 = ((sSys_cfg.oc>>3)&0x01)?" 4·:���� \n"	:" 4·:���� \n";	LCD_DisplayString(str2);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		str2 = ((sSys_cfg.oc>>4)&0x01)?" 5·:����,"		:" 5·:����,";		LCD_DisplayString(str2);
		str2 = ((sSys_cfg.oc>>5)&0x01)?" 6·:���� \n"	:" 6·:���� \n";	LCD_DisplayString(str2);
		break;
	case 5:
		sprintf(str,"      ©��״̬\n");	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		if ( sSys_cfg.cur_ld[0] >= 0 )	{
			positive = -sSys_cfg.cur_ld[0]; negative = 0;
		}	else {
			negative = -sSys_cfg.cur_ld[0];  positive= 0;
		}
		sprintf(str,"�� ��:��%4.1fV,��%3.1fV\n",positive/100,negative/100);	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		if ( sSys_cfg.cur_ld[2] >= 0 )	{
			positive = -sSys_cfg.cur_ld[2]; negative = 0;
		}	else {
			negative = -sSys_cfg.cur_ld[2]; positive = 0;
		}
		sprintf(str,"��3��:��%4.1fV,��%3.1fV\n",positive/100,negative/100);	LCD_DisplayString(str);
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		if ( sSys_cfg.cur_ld[3] >= 0 )	{
			positive = -sSys_cfg.cur_ld[3]; negative = 0;
		} else {
			negative = -sSys_cfg.cur_ld[3]; positive = 0;
		}
		sprintf(str,"��4��:��%4.1fV,��%3.1fV\n",positive/100,negative/100);	LCD_DisplayString(str);
		break;
	case 6:
		sprintf(str,"  ����ϵͳ��Դģ��\n");	LCD_DisplayString(str);
				 if ( sSys_cfg.spmu[0].status & PMU_OV )	str2 = "��ѹ";
		else if ( sSys_cfg.spmu[0].status & PMU_OV )	str2 = "����";
		else if ( sSys_cfg.spmu[0].status & PMU_OT )	str2 = "����";
		else if ( sSys_cfg.spmu[0].status & PMU_ERR)	str2 = "�쳣";
		else if ( sSys_cfg.spmu[0].status & PMU_TO )	str2 = "�쳣";
		else str2 = "����";
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"1:%5.1fV,%5.1fA %s\n",(float)sSys_cfg.spmu[0].vol/100,(float)sSys_cfg.spmu[0].cur/100,str2);	LCD_DisplayString(str);
	
				 if ( sSys_cfg.spmu[1].status & PMU_OV )	str2 = "��ѹ";
		else if ( sSys_cfg.spmu[1].status & PMU_OV )	str2 = "����";
		else if ( sSys_cfg.spmu[1].status & PMU_OT )	str2 = "����";
		else if ( sSys_cfg.spmu[1].status & PMU_ERR)	str2 = "�쳣";
		else if ( sSys_cfg.spmu[1].status & PMU_TO )	str2 = "�쳣";
		else str2 = "����";
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"2:%5.1fV,%5.1fA %s\n",(float)sSys_cfg.spmu[1].vol/100,(float)sSys_cfg.spmu[1].cur/100,str2);	LCD_DisplayString(str);
		break;
	case 7:
		sprintf(str,"  �¿�ϵͳ��Դģ��\n");	LCD_DisplayString(str);
				 if ( sSys_cfg.spmu[2].status & PMU_OV )	str2 = "��ѹ";
		else if ( sSys_cfg.spmu[2].status & PMU_OV )	str2 = "����";
		else if ( sSys_cfg.spmu[2].status & PMU_OT )	str2 = "����";
		else if ( sSys_cfg.spmu[2].status & PMU_ERR)	str2 = "�쳣";
		else if ( sSys_cfg.spmu[2].status & PMU_TO )	str2 = "�쳣";
		else str2 = "����";
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"1:%5.1fV,%5.1fA %s\n",(float)sSys_cfg.spmu[2].vol/100,(float)sSys_cfg.spmu[2].cur/100,str2);	LCD_DisplayString(str);

				 if ( sSys_cfg.spmu[3].status & PMU_OV )	str2 = "��ѹ";
		else if ( sSys_cfg.spmu[3].status & PMU_OV )	str2 = "����";
		else if ( sSys_cfg.spmu[3].status & PMU_OT )	str2 = "����";
		else if ( sSys_cfg.spmu[3].status & PMU_ERR)	str2 = "�쳣";
		else if ( sSys_cfg.spmu[3].status & PMU_TO )	str2 = "�쳣";
		else str2 = "����";
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"2:%5.1fV,%5.1fA %s\n",(float)sSys_cfg.spmu[3].vol/100,(float)sSys_cfg.spmu[3].cur/100,str2);	LCD_DisplayString(str);
		break;
	case 8:
		sprintf(str,"    �������Դģ��\n");	LCD_DisplayString(str);
				 if ( sSys_cfg.spmu[4].status & PMU_OV )	str2 = "��ѹ";
		else if ( sSys_cfg.spmu[4].status & PMU_OV )	str2 = "����";
		else if ( sSys_cfg.spmu[4].status & PMU_OT )	str2 = "����";
		else if ( sSys_cfg.spmu[4].status & PMU_ERR)	str2 = "�쳣";
		else if ( sSys_cfg.spmu[4].status & PMU_TO )	str2 = "�쳣";
		else str2 = "����";
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"1:%5.1fV,%5.1fA %s\n",(float)sSys_cfg.spmu[4].vol/100,(float)sSys_cfg.spmu[4].cur/100,str2);	LCD_DisplayString(str);

				 if ( sSys_cfg.spmu[5].status & PMU_OV )	str2 = "��ѹ";
		else if ( sSys_cfg.spmu[5].status & PMU_OV )	str2 = "����";
		else if ( sSys_cfg.spmu[5].status & PMU_OT )	str2 = "����";
		else if ( sSys_cfg.spmu[5].status & PMU_ERR)	str2 = "�쳣";
		else if ( sSys_cfg.spmu[5].status & PMU_TO )	str2 = "�쳣";
		else str2 = "����";
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"2:%5.1fV,%5.1fA %s\n",(float)sSys_cfg.spmu[5].vol/100,(float)sSys_cfg.spmu[5].cur/100,str2);	LCD_DisplayString(str);
		break;
	case 9:
		sprintf(str,"    �������Դģ��\n");	LCD_DisplayString(str);
				 if ( sSys_cfg.spmu[6].status & PMU_OV )	str2 = "��ѹ";
		else if ( sSys_cfg.spmu[6].status & PMU_OV )	str2 = "����";
		else if ( sSys_cfg.spmu[6].status & PMU_OT )	str2 = "����";
		else if ( sSys_cfg.spmu[6].status & PMU_ERR)	str2 = "�쳣";
		else if ( sSys_cfg.spmu[6].status & PMU_TO )	str2 = "�쳣";
		else str2 = "����";
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"1:%5.1fV,%5.1fA %s\n",(float)sSys_cfg.spmu[6].vol/100,(float)sSys_cfg.spmu[6].cur/100,str2);	LCD_DisplayString(str);

				 if ( sSys_cfg.spmu[7].status & PMU_OV )	str2 = "��ѹ";
		else if ( sSys_cfg.spmu[7].status & PMU_OV )	str2 = "����";
		else if ( sSys_cfg.spmu[7].status & PMU_OT )	str2 = "����";
		else if ( sSys_cfg.spmu[7].status & PMU_ERR)	str2 = "�쳣";
		else if ( sSys_cfg.spmu[7].status & PMU_TO )	str2 = "�쳣";
		else str2 = "����";
		LCD_SetCursor(0,LCD_GetCursorY()+4);
		sprintf(str,"2:%5.1fV,%5.1fA %s\n",(float)sSys_cfg.spmu[7].vol/100,(float)sSys_cfg.spmu[7].cur/100,str2);	LCD_DisplayString(str);
		break;
	default:
		page = 0;
		break;
	}
}

void main_win_msg(uint16_t msg)
{
	switch (msg){
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
	case MSG_USER_MAIN_UP:
		page --;
		if ( page < 0 )
			page = 9;
		Win_PutMsg(MSG_WIN_REDRAW);
		break;
	case MSG_USER_MAIN_DOWN:
		page ++;
		if ( page > 9 )
			page = 0;
		Win_PutMsg(MSG_WIN_REDRAW);
		break;
	default:
		break;
	}
}


