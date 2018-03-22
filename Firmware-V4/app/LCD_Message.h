#ifndef LCD_MESSAGE_H
#define LCD_MESSAGE_H

/* The structure passed to the LCD when there is text to display. */
typedef struct
{
	long CursorX;
	long CursorY;
	signed char *pcMessage;
} xLCDMessage;


#endif

