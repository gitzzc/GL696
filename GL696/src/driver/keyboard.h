#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#define MSG_KEY_NULL	0
#define MSG_KEY_F1		(1<<0)
#define MSG_KEY_F2		(1<<1)
#define MSG_KEY_F3		(1<<2)
#define MSG_KEY_F4		(1<<3)

#define MSG_KEY_DOWN	(0<<8)
#define MSG_KEY_BREAK	(1<<8)
#define MSG_KEY_HOLD	(2<<8)

//#define KB_GET_SCANCODE()	((~GPIO_ReadInputData(KEY_PORT))&KEY_MASK)
#define KB_GET_SCANCODE()	(KeyScanCode)

unsigned portBASE_TYPE KB_Init(void);
unsigned portBASE_TYPE KB_Get(uint16_t *pcRxedkey, portTickType xBlockTime );
void KB_Task(void);
void KB_SetScanCode(uint32_t key);

#endif

