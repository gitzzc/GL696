/* Includes ------------------------------------------------------------------*/
/* Standard includes. */
#include <stdio.h>
#include <string.h>

#include "projdefs.h"

#include "lcd.h"
#include "fonts.h"
#include "lcdtimer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

//planar EL160.80.50
#define LCD_PIXEL_X		160L
#define LCD_PIXEL_Y		80L

/* Private macro -------------------------------------------------------------*/

#define LCD_ReadData(dat)	\
	do { 	\
		*(dat) = LCD_PixelBuf[LCD_PixelAddr];	\
	} while(0)
	
#define LCD_WriteData(dat)	\
	do { 	\
		LCD_PixelBuf[LCD_PixelAddr] = dat;	\
	} while(0)
	
#define LCD_BSRR	GPIOD->BSRR
#define LCD_ODR		GPIOD->ODR

#define LCD_VCLK	GPIO_Pin_3
#define LCD_HS		GPIO_Pin_2
#define LCD_VS		GPIO_Pin_1
//#define LCD_VID	(GPIO_Pin_4-7)

#define LCD_VCLK_SET()	LCD_BSRR = LCD_VCLK
#define LCD_VCLK_CLR()	LCD_BSRR = ((u32)LCD_VCLK<<16)
#define LCD_HS_SET()	LCD_BSRR = LCD_HS
#define LCD_HS_CLR()	LCD_BSRR = ((u32)LCD_HS<<16)
#define LCD_VS_SET()	LCD_BSRR = LCD_VS
#define LCD_VS_CLR()	LCD_BSRR = ((u32)LCD_VS<<16)

#define LCD_PORT_INIT()	\
	do {	\
	} while(0)

/* Private variables ---------------------------------------------------------*/
static vu8	LCD_refresh_enable = 0;									
static vu8 	LCD_PixelBuf[LCD_PIXEL_X*LCD_PIXEL_Y/8];
static u16 	LCD_PixelAddr = 0;

volatile uint16_t LCD_CursorX;
volatile uint16_t LCD_CursorY;


/* Global variables to set the written text color */
static  u16 LCD_TextColor = 0x0000, LCD_BackColor = 0xFFFF;

uint16_t LCD_Bright;
static sFONT *LCD_Currentfonts_EN;
static sFONT *LCD_Currentfonts_CH;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*-----------------------------------------------------------*/

void LCD_pixel_refresh( void )
{
	vu8 i;
	u16 x;
	static u16 y=0;
	static u16 addr=0;
	u8 high,low;

	if ( LCD_refresh_enable == 0 )
		return;

	if ( y == 0 ){
		addr = 0;
		LCD_VS_SET();for(i=0;i<5;i++);
		LCD_HS_CLR();for(i=0;i<5;i++);
		for (x=0;x<LCD_PIXEL_X/8;x++){
			high =  LCD_PixelBuf[addr]&0xF0;
			LCD_VCLK_SET();
			LCD_ODR  =  (LCD_ODR&0xFF07)|high;
 		  for(i=0;i<2;i++); 

			low	 = (LCD_PixelBuf[addr]<<4)&0xF0;
			LCD_VCLK_SET();
			LCD_ODR  =  (LCD_ODR&0xFF07)|low;
			addr++;
 		  for(i=0;i<2;i++); 
		} 
								 for(i=0;i<3;i++); 
		LCD_VS_SET();for(i=0;i<3;i++);
		LCD_HS_SET();for(i=0;i<4;i++);
		LCD_HS_CLR();for(i=0;i<2;i++);
		LCD_VS_CLR();for(i=0;i<3;i++);
		LCD_VCLK_SET();for(i=0;i<3;i++);
		++y;
	} else {
		for (x=0;x<LCD_PIXEL_X/8;x++){
			high =  LCD_PixelBuf[addr]&0xF0;
			LCD_VCLK_SET();
			LCD_ODR  =  (LCD_ODR&0xFF07)|high;
 		  for(i=0;i<2;i++); 

			low	 = (LCD_PixelBuf[addr]<<4)&0xF0;
			LCD_VCLK_SET();
			LCD_ODR  =  (LCD_ODR&0xFF07)|low;
			addr++;
 		  for(i=0;i<2;i++); 
		}
								 for(i=0;i<3;i++);
		LCD_HS_SET();for(i=0;i<4;i++);
		LCD_HS_CLR();for(i=0;i<2;i++);
		if ( ++y == LCD_PIXEL_Y ){
			y = 0;
			LCD_VS_CLR();
		}
	}	
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
	LCD_refresh_enable = 1;
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
	LCD_refresh_enable = 0;
}

/*******************************************************************************
* Function Name  : LCD_SetBright
* Description    : Set the LCD Bright.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_SetBright(uint32_t bright)
{
	LCD_Bright = bright;
}

/*******************************************************************************
* Function Name  : LCD_SetBright
* Description    : Set the LCD Bright.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t LCD_GetBright(void)
{
	return LCD_Bright;
}

/*******************************************************************************
* Function Name  : LCD_Port_Init
* Description    : Configures LCD control lines in Output Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_PortInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE);

  /* Configure DATA (PD.00-07) in Output Push-Pull mode */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |\
  								  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}


void LCD_Init( void )
{
	/* Initialize the LCD */
	LCD_PortInit();
	
	/* Set the Back Color */
	LCD_SetBackColor( Black );
	/* Set the Text Color */
	LCD_SetTextColor( White );
	LCD_Clear();

	LCD_SetBright(0xFFFF);
	LCD_DisplayOn();
	LCD_SetFont_EN(&EN_Font8x16);
	LCD_SetFont_CH(&CH_Font16x16);
	LcdTimerInit();
}

/*******************************************************************************
* Function Name  : LCD_SetTextColor
* Description    : Sets the Text color.
* Input          : - Color: specifies the Text color code RGB(5-6-5).
* Output         : - TextColor: Text color global variable used by LCD_DrawChar
*                  and LCD_DrawPicture functions.
* Return         : None
*******************************************************************************/
void LCD_SetTextColor(u16 Color)
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
u16 LCD_GetTextColor( void )
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
void LCD_SetBackColor(u16 Color)
{
  	LCD_BackColor = Color;
}

/*******************************************************************************
* Function Name  : LCD_SetBackColor
* Description    : Sets the Background color.
* Input          : - Color: specifies the Background color code RGB(5-6-5).
* Output         : - BackColor: Background color global variable used by
*                  LCD_DrawChar and LCD_DrawPicture functions.
* Return         : None
*******************************************************************************/
u16 LCD_GetBackColor( void )
{
  	return LCD_BackColor;
}


/**
  * @brief  Sets the Text Font.
  * @param  fonts: specifies the font to be used.
  * @retval None
  */
void LCD_SetFont_EN(sFONT *fonts)
{
  LCD_Currentfonts_EN = fonts;
}

/**
  * @brief  Gets the Text Font.
  * @param  None.
  * @retval the used font.
  */
sFONT *LCD_GetFont_EN(void)
{
  return LCD_Currentfonts_EN;
}

/**
  * @brief  Sets the Text Font.
  * @param  fonts: specifies the font to be used.
  * @retval None
  */
void LCD_SetFont_CH(sFONT *fonts)
{
  LCD_Currentfonts_CH = fonts;
}

/**
  * @brief  Gets the Text Font.
  * @param  None.
  * @retval the used font.
  */
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
void LCD_ClearLine(u8 Line)
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
	u16 i;
	u8 color;
	
	if ( LCD_TextColor )
		color = 0x00;
	else
		color = 0xFF;

	for(i=0; i<LCD_PIXEL_Y*LCD_PIXEL_X/8; i++)
		LCD_PixelBuf[i] = color;
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
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
	LCD_CursorX = Xpos;
	LCD_CursorY = Ypos;
	
	if ( LCD_CursorX >= LCD_PIXEL_X )
		LCD_CursorX = LCD_PIXEL_X-1;
	if ( LCD_CursorY >= LCD_PIXEL_Y )
		LCD_CursorY = LCD_PIXEL_Y-1;
}

u16 LCD_GetCursorX()
{
	return LCD_CursorX;	
}

u16 LCD_GetCursorY()
{
	return LCD_CursorY;	
}

//-------------------------------------------------------------------------------------------------
void LCD_SetPixel(uint16_t x, uint16_t y,uint16_t color)
{
	u8  pixels;
	
	if ( x >= LCD_PIXEL_X || y >= LCD_PIXEL_Y )
		return ;		

	LCD_PixelAddr = ((u32)LCD_PIXEL_X * y + x)/8; 
	LCD_ReadData(&pixels);
	
	if(color)
		pixels |=  (1 << (x % 8));
	else
		pixels &= ~(1 << (x % 8));
	
	LCD_WriteData(pixels);
}

uint8_t LCD_GetPixel(uint16_t x, uint16_t y)
{
	uint8_t pixels;
	
	LCD_PixelAddr = ((u32)LCD_PIXEL_X * y + x)/8; 
	LCD_ReadData(&pixels);
	return ((pixels >> (x % 8)) & 0x01);
}

uint16_t LCD_GetWindowSize(uint16_t width,uint16_t height)
{
	return (((uint32_t)height*width+7)/8);
}

void LCD_GetWindow(uint8_t* buf,uint16_t tx,uint16_t ty,uint16_t width,uint16_t height)
{
	uint16_t x,y;
	
	for(y=0;y<height;y++){
		for(x=0;x<width;x++){
			buf[(width * y + x)/8] |= LCD_GetPixel(tx+x,ty+y)<<((width * y + x)%8);
		}
	}
}

void LCD_SetWindow(const uint8_t *buf, uint16_t tx, uint16_t ty, uint16_t width, uint16_t height)
{
	uint16_t x,y;

	for(y=0;y<height;y++){
		for(x=0;x<width;x++){
			LCD_SetPixel(tx+x,ty+y,(buf[(width*y + x)/8] >> ((width*y + x)%8)) & 0x01); 
		}
	}
}

void LCD_ClearWindow(uint16_t tx, uint16_t ty, uint16_t width, uint16_t height)
{
	uint16_t x,y;

	for(y=0;y<height;y++){
		for(x=0;x<width;x++){
			LCD_SetPixel(tx+x,ty+y,LCD_BackColor); 
		}
	}
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
void LCD_DrawChar( const uint8_t *c,uint16_t width, uint16_t height )
{
	uint32_t line_index = 0, pixel_index = 0;
	
	for(line_index = 0; line_index < height; line_index++)
	{
		for(pixel_index = 0; pixel_index < width; pixel_index++)
		{
			if( c[line_index*((width+7)/8) + (pixel_index/8)] & (0x01 << (pixel_index % 8)) )
				LCD_SetPixel(LCD_CursorX+pixel_index, LCD_CursorY+line_index,LCD_TextColor);
			else 
				LCD_SetPixel(LCD_CursorX+pixel_index, LCD_CursorY+line_index,LCD_BackColor);
		}
	}
}

/*******************************************************************************
* Function Name  : LCD_DisplayChar
* Description    : Displays one character (16dots width, 24dots height).
* Input          : - Line: the Line where to display the character shape .
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Column: start column address.
*                  - Ascii: character ascii code, must be between 0x20 and 0x7E.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayChar(uint8_t Ascii)
{
  Ascii -= ' ';
  LCD_DrawChar(&LCD_Currentfonts_EN->table[Ascii * ((LCD_Currentfonts_EN->Width+7)/8)*LCD_Currentfonts_EN->Height],LCD_Currentfonts_EN->Width,LCD_Currentfonts_EN->Height);
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
	u16 i;
	const char* hz = (const char*)font->table;

	for(i=0;hz[0] != 0;i++){
		hz = (const char*)font->table+(((font->Width+7)/8)*font->Height + 2)*i;
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
void LCD_DisplayHz( const char* hz)
{
	const char* mask;
	
	mask = LCD_GetHz(hz,LCD_Currentfonts_CH);
	LCD_DrawChar((uint8_t*)mask+2,LCD_Currentfonts_CH->Width,LCD_Currentfonts_CH->Height);
	LCD_SetCursor(LCD_CursorX + LCD_Currentfonts_CH->Width,LCD_CursorY);
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
			LCD_SetCursor(0, LCD_CursorY+16);
		} else if ( *str == '\b' ){
			if ( LCD_CursorX > 8 ){
				LCD_SetCursor(LCD_CursorX -8, LCD_CursorY);
				LCD_DisplayChar( ' ');
				LCD_SetCursor(LCD_CursorX -8, LCD_CursorY);
			} else {
				LCD_SetCursor(LCD_PIXEL_X - 8, LCD_CursorY);
				LCD_DisplayChar(' ');
				LCD_SetCursor(LCD_PIXEL_X - 8, LCD_CursorY);
			}
			str++;
		} else if((*(uint8_t*)str) > 0xA0){  
			LCD_DisplayHz(str);
			str += 2;    			
		} else if ( (*str) > 0x19 && (*str) < 0x7F ){						
			LCD_DisplayChar(*str++);
			LCD_SetCursor(LCD_CursorX + LCD_Currentfonts_EN->Width, LCD_CursorY);
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
void LCD_ScrollText(u8 Line, u8 *ptr)
{
//  u32 i = 0, length = 0, x = 0;
//  u16 refcolumn = 319;

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
void LCD_DrawLine(short X1, short Y1,short X2,short Y2)
{
	short CurrentX, CurrentY, Xinc, Yinc, 
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
void LCD_DrawRect(unsigned short x, unsigned short y, unsigned short w, unsigned short h)
{
	unsigned short j;
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
void LCD_DrawCircle(unsigned short cx, unsigned short cy ,unsigned short radius)
{
	short x, y, xchange, ychange, radiusError;
	x = radius;
	y = 0;
	xchange = 1 - 2 * radius;
	ychange = 1;
	radiusError = 0;
	while(x >= y)
	{
	  LCD_SetPixel(cx+x, cy+y,LCD_TextColor); 
	  LCD_SetPixel(cx-x, cy+y,LCD_TextColor); 
	  LCD_SetPixel(cx-x, cy-y,LCD_TextColor);
	  LCD_SetPixel(cx+x, cy-y,LCD_TextColor); 
	  LCD_SetPixel(cx+y, cy+x,LCD_TextColor); 
	  LCD_SetPixel(cx-y, cy+x,LCD_TextColor); 
	  LCD_SetPixel(cx-y, cy-x,LCD_TextColor); 
	  LCD_SetPixel(cx+y, cy-x,LCD_TextColor); 
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
* Function Name  : LCD_DrawMonoBMP
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
void LCD_DrawBMP(u32 BmpAddress)
{
}

/*-----------------------------------------------------------*/

int fputs( const char* s, FILE *f )
{
	LCD_DisplayString( s );
	return 1;
}
/*-----------------------------------------------------------*/

