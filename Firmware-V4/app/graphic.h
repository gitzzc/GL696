#ifndef __graphic_h__
#define __graphic_h__

void GLCD_Rectangle(unsigned short x, unsigned short y, unsigned short b, unsigned short a);
void GLCD_Circle(unsigned short cx, unsigned short cy ,unsigned short radius);
void GLCD_Line(short X1, short Y1,short X2,short Y2);
void GLCD_FillRect(short xstart,short ystart,short width,short hight,short color);


#endif