#include "projdefs.h"
#include "common.h"
#include "keyboard.h"
#include "timer.h"
/*-----------------------------------------------------------*/

/* KB HARDWARE defines. */
#define KEY_SCAN_TIME 		(HZ_TICK/100)	//10mS
#define KEY_FILTER  			(HZ_TICK/20)/KEY_SCAN_TIME
#define KEY_REPEAT_DELAY	(HZ_TICK)/KEY_SCAN_TIME
#define KEY_REPEAT_SPEED	(HZ_TICK/20)/KEY_SCAN_TIME
#define KEY_BUF_SIZE		8

/*-----------------------------------------------------------*/
static uint16_t key_qbuf[KEY_BUF_SIZE];
static QUEUE key_queue;
static sTIMEOUT key_to;

volatile uint32_t KeyScanCode=0;

/*-----------------------------------------------------------*/
void KB_SetScanCode(uint32_t key)
{
	KeyScanCode = key;
}
/*-----------------------------------------------------------*/
void KB_PIN_Init(void)
{
}

/*-----------------------------------------------------------*/
unsigned portBASE_TYPE KB_Init(void)
{
	KB_PIN_Init();

	if ( init_queue(&key_queue,key_qbuf,KEY_BUF_SIZE) <= 0 )
		return pdFALSE;
	return pdTRUE;
}

/*-----------------------------------------------------------*/
void KB_scan(void)
{
	uint16_t key;
	static uint16_t prev_key=0,break_key=0;
 	static uint8_t  key_count=0,key_filter=KEY_FILTER,key_repeat=0;
	
	key = KB_GET_SCANCODE();
	
	if ( key == prev_key ){
		if ( key_count ++ == key_filter ){
			if ( key == 0 ){
				if ( break_key ){
					key = break_key | MSG_KEY_BREAK;
					break_key = 0;
					enqueue(&key_queue, key);
				} else 
					key_count = 0;
			} else {
				break_key = key;
				enqueue(&key_queue, key);
				if ( key_repeat )
					key_count = 0;
			}
		} else if (key_count >= KEY_REPEAT_DELAY){
			key_repeat = 1;
			key_count = 0;
			key_filter = KEY_REPEAT_SPEED;
		}
	} else {
		prev_key  = key;
		key_filter = KEY_FILTER;
		key_repeat = 0;
		key_count = 0;
	}
}

/*-----------------------------------------------------------*/
void KB_Task(void)
{
	uint8_t ret;

	if ( (ret = get_timeout(&key_to)) == TO_TIMEOUT ){
		start_timeout(&key_to,KEY_SCAN_TIME);
		KB_scan();
	} else if ( ret != TO_RUNING ){
		start_timeout(&key_to,KEY_SCAN_TIME);
	}	
}

/*-----------------------------------------------------------*/
unsigned portBASE_TYPE KB_Get(uint16_t *pcRxedkey, portTickType xBlockTime )
{
	if( dequeue(&key_queue,pcRxedkey) >= 0 )
		return pdTRUE;
	else
		return pdFALSE;
}

