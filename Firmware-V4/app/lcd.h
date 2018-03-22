/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : lcd.h
* Author             : MCD Application Team
* Date First Issued  : mm/dd/yyyy
* Description        : This file contains all the functions prototypes for the
*                      lcd software driver.
********************************************************************************
* History:
* mm/dd/yyyy
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "fonts.h"

/* Exported types ------------------------------------------------------------*/


#define LCD_SCR_WIDTH   640
#define LCD_SCR_HIGH	480


//extern uint16_t LCD_CursorX,LCD_CursorY;

/* The queue used to send messages to the LCD task. */
extern xQueueHandle xLCDQueue;

/* Exported constants --------------------------------------------------------*/
/* LCD Control pins */
#define CtrlPin_NCS    GPIO_Pin_2   /* PB.02 */
#define CtrlPin_RS     GPIO_Pin_7   /* PD.07 */
#define CtrlPin_NWR    GPIO_Pin_15  /* PD.15 */

/* LCD color */
#define White          0xFFFF
#define Black          0x0000
#define Blue           0x001F
#define Orange         0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

#define Line0          0
#define Line1          24
#define Line2          48
#define Line3          72
#define Line4          96
#define Line5          120
#define Line6          144
#define Line7          168
#define Line8          192
#define Line9          216

#define Horizontal     0x00
#define Vertical       0x01

#define LCD_WRITE_PAGE0		(0)
#define LCD_WRITE_PAGE1		(1<<3)
#define LCD_DISPLAY_PAGE0	(0)
#define LCD_DISPLAY_PAGE1	(1<<4)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void LCD_Init(void);
void LCD_SetTextColor(uint16_t Color);
uint16_t LCD_GetTextColor( void );
void LCD_SetBackColor(uint16_t Color);
uint16_t LCD_GetBackColor( void );
void LCD_ClearLine(uint8_t Line);
void LCD_Clear(void);
void LCD_SET_PAGE(uint8_t page);

void LCD_SetFont_EN(sFONT *fonts);
sFONT *LCD_GetFont_EN(void);
void LCD_SetFont_CH(sFONT *fonts);
sFONT *LCD_GetFont_CH(void);

void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
u16 LCD_GetCursorX(void);
u16 LCD_GetCursorY(void);

void LCD_DisplayAscii(int8_t Ascii);
void LCD_DisplayString(const char* str);
void LCD_ScrollText(uint8_t Line, uint8_t *ptr);
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_DrawLine(int16_t X1, int16_t Y1,int16_t X2,int16_t Y2);
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t b, uint16_t a);
void LCD_DrawCircle(uint16_t cx, uint16_t cy ,uint16_t radius);
void LCD_DrawMonoPict(uc32 *Pict);
void LCD_DrawBMP(u32 BmpAddress);

/*----- Medium layer function -----*/
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);

/*----- Low layer function -----*/
void LCD_CtrlLinesConfig(void);
void LCD_CtrlLinesWrite(GPIO_TypeDef* GPIOx, uint16_t CtrlPins, BitAction BitVal);
void LCD_SPIConfig(void);

#endif /* __LCD_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
