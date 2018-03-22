#include "sed1335.h"

const unsigned char color = 1;

void GLCD_Rectangle(unsigned short x, unsigned short y, unsigned short b, unsigned short a)
{
  unsigned short j;
  for (j = 0; j < a; j++) {
		GLCD_SetPixel(x, y + j, color);
		GLCD_SetPixel(x + b - 1, y + j, color);
	}
  for (j = 0; j < b; j++)	{
		GLCD_SetPixel(x + j, y, color);
		GLCD_SetPixel(x + j, y + a - 1, color);
	}
}


void GLCD_Circle(unsigned short cx, unsigned short cy ,unsigned short radius)
{
	short x, y, xchange, ychange, radiusError;
	x = radius;
	y = 0;
	xchange = 1 - 2 * radius;
	ychange = 1;
	radiusError = 0;
	while(x >= y)
	{
	  GLCD_SetPixel(cx+x, cy+y, color); 
	  GLCD_SetPixel(cx-x, cy+y, color); 
	  GLCD_SetPixel(cx-x, cy-y, color);
	  GLCD_SetPixel(cx+x, cy-y, color); 
	  GLCD_SetPixel(cx+y, cy+x, color); 
	  GLCD_SetPixel(cx-y, cy+x, color); 
	  GLCD_SetPixel(cx-y, cy-x, color); 
	  GLCD_SetPixel(cx+y, cy-x, color); 
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


void GLCD_Line(short X1, short Y1,short X2,short Y2)
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

	GLCD_SetPixel(X1,Y1, color); 

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
       	GLCD_SetPixel(CurrentX,CurrentY, color);
     	}while (CurrentX != X2); 
		} else {
			TwoDyAccumulatedError = 0; 
      do {
        CurrentY += Yinc; 
        TwoDyAccumulatedError += TwoDx;
        if(TwoDyAccumulatedError>Dy) {
          CurrentX += Xinc;
          TwoDyAccumulatedError -= TwoDy;
        }
        GLCD_SetPixel(CurrentX,CurrentY, color); 
			}while (CurrentY != Y2);
    }
  }
}


//
// 填充矩形框
//
// 输入：   xstart      横轴起始坐标
//          ystart      纵轴起始坐标
//          width       矩形框宽度
//          hight       矩形框高度
//          color       矩形框内部填充颜色
//
// 输出：   操作成功与否标志
//

void GLCD_FillRect(short xstart,short ystart,short width,short hight,short color)
{
    short side1, side2, x, y;
    unsigned char pashort;
    unsigned _Color = 0;

    if (ystart + hight > SED1335_SCR_HIGH || xstart + width > SED1335_SCR_WIDTH){
        return ;
    }

    side1 = 8 - (xstart % 8);
    side2 = (xstart + width) % 8;
    //
    // 光标方向向右
    //
    GLCD_WriteCommand(SED1335_CSRDIR_R);

    for (y = ystart; y < ystart + hight; y++)
    {
        //
        // 左边界处理
        //
        if (side1 < 8) {
            GLCD_SetCursor(xstart, y);
           	GLCD_WriteCommand(SED1335_MREAD);
            pashort = (color ? 0xff : 0x00) >> (8 - side1);
            pashort = pashort | ((GLCD_ReadData() >> side1) << side1);
        } else {
            pashort = color ? 0xff : 0x00;
        }

        GLCD_SetCursor(xstart, y);
        GLCD_WriteCommand(SED1335_MWRITE);
        GLCD_WriteData(pashort);

        //
        // 主体绘画
        //

        for (x = (xstart / 8) + 1; x < (xstart + width) / 8; x++) {
            GLCD_WriteData(color ? 0xff : 0x00);
        }

        //
        // 右边界处理
        //

        if (side2) {
        		GLCD_WriteCommand(SED1335_MREAD);
            pashort = (((_Color ? 0xff : 0x00) >> (8 - side2)) << (8 - side2))
                    | (GLCD_ReadData() >> side2);

            GLCD_SetCursor(xstart + width, y);
            GLCD_WriteCommand(SED1335_MWRITE);
            GLCD_WriteData(pashort);
        }
    }
}
