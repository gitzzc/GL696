/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : lcd.c
* Author             : MCD Application Team
* Date First Issued  : mm/dd/yyyy
* Description        : This file includes the LCD driver for AM-240320LTNQW00H
*                      liquid Crystal Display Module of STM32F10x-EVAL.
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

/* Includes ------------------------------------------------------------------*/
/* Standard includes. */
#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"
//#include "spi_flash.h"

#include "fonts.h"
#include "lcd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "LCD_Message.h"
#include "touchscreen.h"
#include "Calibrate.h"
#include "can.h"
#include "iic_eeprom.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


#define countof(a) (sizeof(a) / sizeof(*(a)))
#define BUFFER_SIZE1             (countof(Tx1_Buffer)-1)
#define BUFFER_SIZE2             (countof(Tx2_Buffer)-1)
	  
	  
/* Private macro -------------------------------------------------------------*/

#define LCD_LED_ON()	GPIO_WriteBit(GPIOE, LCD_LED_PIN, (BitAction)1)
#define LCD_LED_OFF()	GPIO_WriteBit(GPIOE, LCD_LED_PIN, (BitAction)0)
	
#define LCD_ADDR_DAT 	(0<<8)
#define LCD_ADDR_ROW 	(1<<8)
#define LCD_ADDR_COL 	(2<<8)
#define LCD_ADDR_PAGE	(3<<8)

#define LCD_PAGE_0		0x08
#define LCD_PAGE_1		0x10

#define LCD_LED_PIN		GPIO_Pin_6
#define LCD_CS_PIN		GPIO_Pin_7

#define LCD_A0_PIN		GPIO_Pin_8
#define LCD_A1_PIN		GPIO_Pin_9
#define LCD_RD_PIN		GPIO_Pin_10
#define LCD_WR_PIN		GPIO_Pin_11

//#define LCD_SET_PAGE(x)		LCD_page = x

/*#define LCD_WriteRAM(RGB_Code) \
		do {	\
			LCD_WriteReg(LCD_ADDR_DAT,RGB_Code>>8);	\
			LCD_WriteReg(LCD_ADDR_DAT,RGB_Code);	\
		} while(0)
*/
/* Private variables ---------------------------------------------------------*/
  /* Global variables to set the written text color */
static uint16_t LCD_TextColor = White;
static uint16_t LCD_BackColor = Black;

volatile uint16_t LCD_CursorX;
volatile uint16_t LCD_CursorY;

volatile uint8_t LCD_page;

static sFONT *LCD_Currentfonts_EN;
static sFONT *LCD_Currentfonts_CH;

/* The queue used to send messages to the LCD task. */
xQueueHandle xLCDQueue;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*-----------------------------------------------------------*/


/*******************************************************************************
* Function Name  : LCD_CtrlLinesConfig
* Description    : Configures LCD control lines in Output Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure DATA (PE.08-15) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6  | GPIO_Pin_7  | \
								  GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11 |\
								  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Configure CS,RD,WR,A0,A1 (PD.03-05, PD.07, PD.11-12) in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_8  | GPIO_Pin_9 | GPIO_Pin_10  | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	LCD_LED_OFF();
}

/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Write the selected LCD Register.
* Input          : None
* Output         : None
* Return         : LCD Register Value.
*******************************************************************************/
void LCD_WriteReg(uint16_t LCD_Reg,uint8_t LCD_RegValue)
{ 
	vu8 i;
	
	vPortEnterCritical();
	GPIOE->ODR = (GPIOE->ODR&0x007F) | ((uint16_t)LCD_RegValue<<8); //for(i=0;i<5;i++);
	GPIOD->ODR = (GPIOD->ODR&0xF0FF) | LCD_RD_PIN | LCD_Reg;	//for(i=0;i<5;i++);
	GPIOD->ODR |= LCD_WR_PIN;	//for(i=0;i<5;i++);
	GPIOE->ODR |= LCD_CS_PIN;
	vPortExitCritical();
}

/*******************************************************************************
* Function Name  : LCD_ReadReg
* Description    : Reads the selected LCD Register.
* Input          : None
* Output         : None
* Return         : LCD Register Value.
*******************************************************************************/
uint8_t LCD_ReadReg(uint8_t LCD_Reg)
{	
	uint8_t tmp;
	
//	LCD_WR_ADDR_EN(LCD_Reg);
//	tmp = LCD_READ(LCD_RegValue);
//	LCD_WR_ADDR_DIS();
	return tmp;
}


uint16_t  LCD_WriteRAM(uint16_t RGB_Code)
{
	LCD_WriteReg(LCD_ADDR_DAT,RGB_Code>>8);	
	LCD_WriteReg(LCD_ADDR_DAT,RGB_Code);
	return 0;	
}
/*******************************************************************************
* Function Name  : LCD_ReadRAM
* Description    : Reads the LCD RAM.
* Input          : None
* Output         : None
* Return         : LCD RAM Value.
*******************************************************************************/
uint16_t  LCD_ReadRAM(void)
{
  uint16_t tmp = 0;

	//tmp  = LCD_ReadReg(LCD_ADDR_DAT);
	//tmp |= LCD_ReadReg(LCD_ADDR_DAT)<<8;

  return tmp;
}

void LCD_SET_PAGE(uint8_t page)
{
	LCD_page = page;
	LCD_SetCursor(LCD_CursorX, LCD_CursorY);
}

/*******************************************************************************
* Function Name  : LCD_PowerOn
* Description    :
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_PowerOn(void)
{
}

/*******************************************************************************
* Function Name  : LCD_DisplayOn
* Description    : Enables the Display.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayOn(void)
{
	LCD_LED_ON();
}

/*******************************************************************************
* Function Name  : LCD_DisplayOff
* Description    : Disables the Display.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayOff(void)
{
	LCD_LED_OFF();
}

void prvConfigureLCD( void )
{
	/* Initialize the LCD */
	LCD_Init();
	
	/* Set the Back Color */
	LCD_SetBackColor( Black );

	/* Set the Text Color */
	LCD_SetTextColor( 0x051F );

	LCD_SET_PAGE(LCD_WRITE_PAGE0 | LCD_DISPLAY_PAGE0);
	LCD_Clear();

	LCD_SET_PAGE(LCD_WRITE_PAGE1 | LCD_DISPLAY_PAGE0);
	LCD_Clear();	

	LCD_SET_PAGE(LCD_WRITE_PAGE0 | LCD_DISPLAY_PAGE0);
	
	LCD_DisplayOn();
}

/*******************************************************************************
* Function Name  : LCD_Init
* Description    : Initializes LCD.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_Init(void)
{
/* Configure the LCD Control pins --------------------------------------------*/
  LCD_CtrlLinesConfig();
}
/*******************************************************************************
* Function Name  : LCD_SetTextColor
* Description    : Sets the Text color.
* Input          : - Color: specifies the Text color code RGB(5-6-5).
* Output         : - TextColor: Text color global variable used by LCD_DrawChar
*                  and LCD_DrawPicture functions.
* Return         : None
*******************************************************************************/
void LCD_SetTextColor(uint16_t Color)
{
  LCD_TextColor = Color;
}

/*******************************************************************************
* Function Name  : LCD_SetTextColor
* Description    : Sets the Text color.
* Input          : - Color: specifies the Text color code RGB(5-6-5).
* Output         : - TextColor: Text color global variable used by LCD_DrawChar
*                  and LCD_DrawPicture functions.
* Return         : None
*******************************************************************************/
uint16_t LCD_GetTextColor( void )
{
  return LCD_TextColor;
}

/*******************************************************************************
* Function Name  : LCD_SetBackColor
* Description    : Sets the Background color.
* Input          : - Color: specifies the Background color code RGB(5-6-5).
* Output         : - BackColor: Background color global variable used by
*                  LCD_DrawChar and LCD_DrawPicture functions.
* Return         : None
*******************************************************************************/
void LCD_SetBackColor(uint16_t Color)
{
  LCD_BackColor = Color;
}

/*******************************************************************************
* Function Name  : LCD_SetTextColor
* Description    : Sets the Text color.
* Input          : - Color: specifies the Text color code RGB(5-6-5).
* Output         : - TextColor: Text color global variable used by LCD_DrawChar
*                  and LCD_DrawPicture functions.
* Return         : None
*******************************************************************************/
uint16_t LCD_GetBackColor( void )
{
  return LCD_BackColor;
}


/*******************************************************************************
* Function Name  : LCD_SetFont_EN
* Description    : Sets the Text Font.
* Input          : - 
* Output         : - 
* Return         : None
*******************************************************************************/
void LCD_SetFont_EN(sFONT *fonts)
{
	LCD_Currentfonts_EN = fonts;
}

/*******************************************************************************
* Function Name  : LCD_SetFont_EN
* Description    : Sets the Text Font.
* Input          : - 
* Output         : - 
* Return         : None
*******************************************************************************/
sFONT *LCD_GetFont_EN(void)
{
	return LCD_Currentfonts_EN;
}

/*******************************************************************************
* Function Name  : LCD_SetFont_EN
* Description    : Sets the Text Font.
* Input          : - 
* Output         : - 
* Return         : None
*******************************************************************************/
void LCD_SetFont_CH(sFONT *fonts)
{
	LCD_Currentfonts_CH = fonts;
}

/*******************************************************************************
* Function Name  : LCD_SetFont_EN
* Description    : Sets the Text Font.
* Input          : - 
* Output         : - 
* Return         : None
*******************************************************************************/
sFONT *LCD_GetFont_CH(void)
{
	return LCD_Currentfonts_CH;
}


/*******************************************************************************
* Function Name  : LCD_ClearLine
* Description    : Clears the selected line.
* Input          : - Line: the Line to be cleared.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_ClearLine(uint8_t Line)
{
}

/*******************************************************************************
* Function Name  : LCD_Clear
* Description    : Clears the hole LCD.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_Clear(void)
{
  	uint32_t i,j;

	for(i = 0; i < 480; i++)
	{
		LCD_SetCursor(0, i);
		for(j=0;j<640;j++){
			LCD_WriteRAM(LCD_BackColor);
		}
	}
	LCD_SetCursor(0,0);
}

/*******************************************************************************
* Function Name  : LCD_SetCursor
* Description    : Sets the cursor position.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	LCD_CursorX = Xpos;
	LCD_CursorY = Ypos;
	if ( LCD_CursorX >= LCD_SCR_WIDTH ){
		LCD_CursorX = LCD_CursorX - 1;
	}
	if ( LCD_CursorY >= LCD_SCR_HIGH )
		LCD_CursorY = LCD_CursorY - 1;

  	//写行列高地址
	LCD_WriteReg(LCD_ADDR_PAGE,(LCD_page&0x18) | ((LCD_CursorY>>6)&0x04) | ((LCD_CursorX>>8)&0X03));
	//写行低地址
	LCD_WriteReg(LCD_ADDR_ROW,LCD_CursorY);
	//写列低地址
	LCD_WriteReg(LCD_ADDR_COL,LCD_CursorX);
}

u16 LCD_GetCursorX(void)
{
	return LCD_CursorX;	
}

u16 LCD_GetCursorY(void)
{
	return LCD_CursorY;	
}

/*******************************************************************************
* Function Name  : LCD_SetPixel
* Description    : Draws a Pixel on LCD.
* Input          : - Xpos: the Line where to display the character shape.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Ypos: start column address.
*                  - c: pointer to the character data.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_SetPixel(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_SetCursor(x, y);
	LCD_WriteRAM(color);
}


/*******************************************************************************
* Function Name  : LCD_DrawChar
* Description    : Draws a character on LCD.
* Input          : - Xpos: the Line where to display the character shape.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Ypos: start column address.
*                  - c: pointer to the character data.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawChar(const char* mask, uint16_t width, uint16_t height)
{
	uint32_t lindex = 0, pindex = 0;
	uint32_t Yaddress = LCD_CursorY;

	for(lindex = 0; lindex < height; lindex++)
	{
		LCD_SetCursor(LCD_CursorX, LCD_CursorY+1);
		for(pindex = 0; pindex < width; pindex++)
		{
			if( mask[lindex*((width+7)>>3) + (pindex>>3)] & (0x80 >> (pindex % 8)) )
				LCD_WriteRAM(LCD_TextColor);
			else 
				LCD_WriteRAM(LCD_BackColor);
		}
	}
	LCD_SetCursor(LCD_CursorX+width, Yaddress);
}

/*******************************************************************************
* Function Name  : LCD_DisplayAscii
* Description    : Displays one character (16dots width, 24dots height).
* Input          : - Line: the Line where to display the character shape .
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Column: start column address.
*                  - Ascii: character ascii code, must be between 0x20 and 0x7E.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayAscii(int8_t Ascii)
{
	sFONT* sfont = LCD_Currentfonts_EN;
	
	Ascii -= ' ';
	LCD_DrawChar((const char*)&sfont->table[Ascii * ((sfont->Width+7)>>3)*sfont->Height],sfont->Width,sfont->Height);
}

/*******************************************************************************
* Function Name  : LCD_DrawChar
* Description    : Draws a character on LCD.
* Input          : - Xpos: the Line where to display the character shape.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Ypos: start column address.
*                  - c: pointer to the character data.
* Output         : None
* Return         : None
*******************************************************************************/
const char* LCD_GetHz(const char* ch, sFONT* font) 
{
	uint16_t i;
	const char* hz = (const char*)font->table;
	
	for(i=0;hz[0] != 0;i++){
		hz = (const char*)font->table+(((font->Width+7)>>3)*font->Height + 2)*i;
		if ( memcmp(ch, hz, 2) == 0 )
			break;
	}
	return hz;
}

/*******************************************************************************
* Function Name  : LCD_DrawChar
* Description    : Draws a character on LCD.
* Input          : - Xpos: the Line where to display the character shape.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Ypos: start column address.
*                  - c: pointer to the character data.
* Output         : None
* Return         : None
*******************************************************************************/
/*const char* LCD_GetHz_2(u8* ch) 
{
	short bReturnResult;
	u16 low=1,high=(sizeof(chinese_tab)/sizeof(Chinese_GB16) - 1),mid; //置当前查找区间上、下界的初值
	
	while( low <= high ){ //当前查找区间R[low..high]非空
		mid = ( low + high ) / 2;
		
		bReturnResult = memcmp(ch, chinese_tab[mid].index, 2); 
		
		if( bReturnResult == 0 ) 
			return (const char*)chinese_tab[mid].mask; //查找成功返回
		if( bReturnResult < 0 )
			high = mid - 1; //继续在R[low..mid-1]中查找
		else
			low = mid + 1; //继续在R[mid+1..high]中查找
	}
	return (const char*)chinese_tab[0].mask; //当low>high时表示查找区间为空，查找失败
}*/


void LCD_DisplayHz( const char* hz)
{
	sFONT* sfont = LCD_Currentfonts_CH;
	const char* mask = LCD_GetHz(hz,sfont);
	
	LCD_DrawChar(mask+2,sfont->Width,sfont->Height);
}


/*******************************************************************************
* Function Name  : LCD_DisplayString
* Description    : Displays a maximum of 200 char on the LCD.
* Input          : - Line: the starting Line where to display the character shape.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - *ptr: pointer to string to display on LCD.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayString(const char* str)
{
 	while(*str) {
		if(*str=='\n'){
			str++;
			if ( LCD_Currentfonts_EN->Height > LCD_Currentfonts_CH->Height )
				LCD_SetCursor(0, LCD_CursorY + LCD_Currentfonts_EN->Height);
			else 
				LCD_SetCursor(0, LCD_CursorY + LCD_Currentfonts_CH->Height);
		} else if ( *str == '\b' ){
			if ( LCD_CursorX > 8 ){
				LCD_SetCursor(LCD_CursorX -8, LCD_CursorY);
				LCD_DisplayAscii(' ');
				LCD_SetCursor(LCD_CursorX -8, LCD_CursorY);
			} else {
				LCD_SetCursor(LCD_SCR_WIDTH - 8, LCD_CursorY);
				LCD_DisplayAscii(' ');
				LCD_SetCursor(LCD_SCR_WIDTH - 8, LCD_CursorY);
			}
			str++;
		} else if((*  str) > 0xA0){  
			LCD_DisplayHz(str);
			str += 2;    			
		} else if ( (*str) > 0x19 && (*str) < 0x7F ){						
			LCD_DisplayAscii(*str++);
		} else 
			str ++;
 	}
}

/*******************************************************************************
* Function Name  : LCD_ScrollText
* Description    :
* Input          :
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_ScrollText(uint8_t Line, uint8_t *ptr)
{
//  uint32_t i = 0, length = 0, x = 0;
//  uint16_t refcolumn = 319;

//  /* Get the string length */
//  length = StrLength(ptr);
//
//  while(1)
//  {
//    /* Send the string character by character on lCD */
//    while ((*ptr != 0) & (i < 20))
//    {
//      /* Display one character on LCD */
//      LCD_DisplayAscii(Line, refcolumn, *ptr);
//      /* Decrement the column position by 16 */
//      refcolumn -= 16;
//      /* Point on the next character */
//      ptr++;
//      /* Increment the character counter */
//      i++;
//    }
//    vTaskDelay( 100 / portTICK_RATE_MS );
//    i = 0;
//    //LCD_ClearLine(Line);
//    ptr -= length;
//    x++;
//    if(refcolumn < 16)
//    {
//      x = 0;
//    }
//    refcolumn = 319 - (x * 16);
//  }
}



/*******************************************************************************
* Function Name  : LCD_DrawLine
* Description    : Displays a line.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
*                  - Length: line length.
*                  - Direction: line direction.
*                    This parameter can be one of the following values: Vertical
*                    or Horizontal.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawLine(int16_t X1, int16_t Y1,int16_t X2,int16_t Y2)
{
	int16_t CurrentX, CurrentY, Xinc, Yinc, 
 	Dx, Dy, TwoDx, TwoDy, 
	TwoDxAccumulatedError, TwoDyAccumulatedError;

	Dx = (X2-X1); 
	Dy = (Y2-Y1); 
	
	TwoDx = Dx + Dx;
	TwoDy = Dy + Dy;
	
	CurrentX = X1; 
	CurrentY = Y1; 
	
	Xinc = 1; 
	Yinc = 1; 
	
	if(Dx < 0)
	{
	  Xinc = -1;
	  Dx = -Dx;
	  TwoDx = -TwoDx; 
	}
	
	if (Dy < 0) 
	{
	  Yinc = -1;
	  Dy = -Dy; 
	  TwoDy = -TwoDy; 
  	}

	LCD_SetPixel(X1,Y1,LCD_TextColor); 

	if ((Dx != 0) || (Dy != 0)) {
	  	if (Dy <= Dx) { 
		    TwoDxAccumulatedError = 0;
	  	  	do {
		      	CurrentX += Xinc; 
		      	TwoDxAccumulatedError += TwoDy; 
		      	if(TwoDxAccumulatedError > Dx) {
		        	CurrentY += Yinc;
		        	TwoDxAccumulatedError -= TwoDx;
		        }
		       	LCD_SetPixel(CurrentX,CurrentY,LCD_TextColor);
		    } while (CurrentX != X2); 
		} else {
			TwoDyAccumulatedError = 0; 
	      	do {
		        CurrentY += Yinc; 
		        TwoDyAccumulatedError += TwoDx;
		        if(TwoDyAccumulatedError>Dy) {
		          CurrentX += Xinc;
		          TwoDyAccumulatedError -= TwoDy;
		        }
	        	LCD_SetPixel(CurrentX,CurrentY,LCD_TextColor); 
			}while (CurrentY != Y2);
	    }
  	}
}

/*******************************************************************************
* Function Name  : LCD_DrawRect
* Description    : Displays a rectangle.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
*                  - Height: display rectangle height.
*                  - Width: display rectangle width.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	uint16_t j;
	for (j = 0; j < h; j++) {
		LCD_SetPixel(x, y + j,LCD_TextColor);
		LCD_SetPixel(x + w - 1, y + j,LCD_TextColor);
	}
	for (j = 0; j < w; j++)	{
		LCD_SetPixel(x + j, y,LCD_TextColor);
		LCD_SetPixel(x + j, y + h - 1,LCD_TextColor);
	} 
}

/*******************************************************************************
* Function Name  : LCD_DrawCircle
* Description    : Displays a circle.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position.
*                  - Height: display rectangle height.
*                  - Width: display rectangle width.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawCircle(uint16_t cx, uint16_t cy ,uint16_t radius)
{
	uint16_t x, y, xchange, ychange, radiusError;
	x = radius;
	y = 0;
	xchange = 1 - 2 * radius;
	ychange = 1;
	radiusError = 0;
	while(x >= y)
	{
	  LCD_SetPixel(cx+x, cy+y ,LCD_TextColor); 
	  LCD_SetPixel(cx-x, cy+y ,LCD_TextColor); 
	  LCD_SetPixel(cx-x, cy-y ,LCD_TextColor);
	  LCD_SetPixel(cx+x, cy-y ,LCD_TextColor); 
	  LCD_SetPixel(cx+y, cy+x ,LCD_TextColor); 
	  LCD_SetPixel(cx-y, cy+x ,LCD_TextColor); 
	  LCD_SetPixel(cx-y, cy-x ,LCD_TextColor); 
	  LCD_SetPixel(cx+y, cy-x ,LCD_TextColor); 
	  y++;
	  radiusError += ychange;
	  ychange += 2;
	  if ( 2*radiusError + xchange > 0 )
	    {
	    x--;
		radiusError += xchange;
		xchange += 2;
		}
  }
}

/*******************************************************************************
* Function Name  : LCD_DrawMonoPict
* Description    : Displays a monocolor picture.
* Input          : - Pict: pointer to the picture array.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawMonoBMP(const uint8_t *Pict, uint16_t Xpos_Init, uint16_t Ypos_Init, uint16_t Height, uint16_t Width)
{
  int32_t index = 0, counter = 0;
  uint16_t x = Xpos_Init - Width, y = Ypos_Init - Height;

  for (index = 0; index < Height; index++)
  {
    for (counter = 0; counter < Width; counter++)
    {
      if ((Pict[(index*Width+counter)/8] & (0x80 >> (counter%8))) == 0x00)
      {
        LCD_SetPixel(x + counter, y + index,LCD_BackColor);
      }
      else
      {
        LCD_SetPixel(x + counter, y + index,LCD_TextColor);
      }
    }
  }
}

/*******************************************************************************
* Function Name  : LCD_DrawBMP
* Description    : Displays a bitmap picture loaded in the SPI Flash.
* Input          : - BmpAddress: Bmp picture address in the SPI Flash.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DrawBMP(uint32_t BmpAddress)
{
//  uint32_t i = 0;

//  LCD_WriteReg(R1, 0xD0);
//  LCD_WriteReg(R5, 0x04);
//
//  LCD_SetCursor(239, 0x013F);
//
//  SPI_FLASH_StartReadSequence(BmpAddress);
//
//  /* Disable SPI1  */
//  SPI_Cmd(SPI1, DISABLE);
//  /* SPI in 16-bit mode */
//  SPI_DataSizeConfig(SPI1, SPI_DataSize_16b);
//  /* Enable SPI1  */
//  SPI_Cmd(SPI1, ENABLE);
//
//  for(i = 0; i < 76800; i++)
//  {
//    LCD_WriteRAM(__REV_HalfWord(SPI_FLASH_SendHalfWord(0xA5A5)));
//  }
//
//  /* Deselect the FLASH: Chip Select high */
//  SPI_FLASH_ChipSelect(1);
//
//  /* Disable SPI1  */
//  SPI_Cmd(SPI1, DISABLE);
//  /* SPI in 8-bit mode */
//  SPI_DataSizeConfig(SPI1, SPI_DataSize_8b);
//  /* Enable SPI1  */
//  SPI_Cmd(SPI1, ENABLE);
}

/*-----------------------------------------------------------*/

int fputs( const char* s, FILE *f )
{
	LCD_DisplayString( s );
	return 1;
}
/*-----------------------------------------------------------*/

uint8_t pdu_output_set_btn=0;
uint8_t psu_select=0;



/*-----------------------------------------------------------*/



/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
