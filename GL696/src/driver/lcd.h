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
#include "fonts.h"

/* Exported types ------------------------------------------------------------*/


#define LCD_SCR_WIDTH   160
#define LCD_SCR_HIGH	80

/* Exported constants --------------------------------------------------------*/

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

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void LCD_SetTextColor(u16 Color);
u16 LCD_GetTextColor( void );
void LCD_SetBackColor(u16 Color);
u16 LCD_GetBackColor( void );
void LCD_SetFont_EN(sFONT *fonts);
void LCD_SetFont_CH(sFONT *fonts);
sFONT *LCD_GetFont_EN(void);
sFONT *LCD_GetFont_CH(void);
void LCD_ClearLine(u8 Line);
void LCD_Clear(void);
void LCD_SetCursor(u16 Xpos, u16 Ypos);
u16 LCD_GetCursorX(void);
u16 LCD_GetCursorY(void);

uint16_t LCD_GetWindowSize(uint16_t width,uint16_t height);
void LCD_GetWindow(uint8_t* buf,uint16_t tx,uint16_t ty,uint16_t width,uint16_t height);
void LCD_SetWindow(const uint8_t *buf, uint16_t tx, uint16_t ty, uint16_t width, uint16_t height);
void LCD_ClearWindow(uint16_t tx, uint16_t ty, uint16_t width, uint16_t height);

void LCD_DisplayAscii(u8 Ascii);
void LCD_DisplayString(const char* str);
void LCD_ScrollText(u8 Line, u8 *ptr);
void LCD_SetDisplayWindow(u8 Xpos, u16 Ypos, u8 Height, u16 Width);
void LCD_DrawLine(short X1, short Y1,short X2,short Y2);
void LCD_DrawRect(unsigned short x, unsigned short y, unsigned short b, unsigned short a);
void LCD_DrawCircle(unsigned short cx, unsigned short cy ,unsigned short radius);
void LCD_DrawMonoPict(uc32 *Pict);
void LCD_DrawBMP(u32 BmpAddress);

/*----- Medium layer function -----*/
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_SetBright(uint32_t bright);
uint32_t LCD_GetBright(void);

/*----- Low layer function -----*/
void LCD_Init(void);
void LCD_PortInit(void);
void LCD_pixel_refresh( void );

#endif /* __LCD_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
