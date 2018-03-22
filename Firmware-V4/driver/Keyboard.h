#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#define MSG_KEY_NULL	0

#define MSG_KEY_ZJ		(1<<8)
#define MSG_KEY_BX		(1<<7)
#define MSG_KEY_DC		(1<<6)
#define MSG_KEY_UP		(1<<5)

#define MSG_KEY_DOWN	(1<<4)
#define MSG_KEY_ENTER	(1<<3)
#define MSG_KEY_F1		(1<<2)
#define MSG_KEY_F2		(1<<1)
#define MSG_KEY_CANEL	(1<<0)

#define MSG_KEY_CONNC	(0<<14)
#define MSG_KEY_BREAK	(1<<14)
#define MSG_KEY_HOLD	(2<<14)

extern volatile uint16_t LED_Value;

unsigned portBASE_TYPE KB_Init(void);
unsigned portBASE_TYPE KB_Get(uint16_t *pcRxedkey, portTickType xBlockTime );

void LED_Set(uint16_t led);

#endif

