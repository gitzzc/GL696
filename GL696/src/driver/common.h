#ifndef __COMMON_H__
#define __COMMON_H__


//-----------------QUEUE-----------------------------------------------
typedef struct 
{
	uint16_t * queue;
	uint8_t size;
	uint8_t front;
	uint8_t rear;
} QUEUE,*pQUEUE;

short init_queue(pQUEUE q,uint16_t *buf,uint8_t size);
short enqueue(pQUEUE q,uint16_t da);
short dequeue(pQUEUE q,uint16_t * da);
//---------------------------------------------------------------------
uint8_t asc2hex(int8_t asc);
uint8_t str2uchar(int8_t *str);
uint16_t str2uint(int8_t *str);
uint32_t str2ulong (int8_t *str);

#endif
