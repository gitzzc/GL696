#ifndef __COMMON_H__
#define __COMMON_H__

#define ContainOf(x) 			(sizeof(x)/sizeof(x[0]))

//-----------------QUEUE-----------------------------------------------
typedef struct 
{
	uint16_t *queue;
	uint8_t size;
	uint8_t front;
	uint8_t rear;
} QUEUE,*pQUEUE;

uint16_t init_queue(pQUEUE q,uint16_t *buf,uint8_t size);
uint16_t enqueue(pQUEUE q,uint16_t da);
uint16_t dequeue(pQUEUE q,uint16_t * da);
//---------------------------------------------------------------------
uint8_t asc2hex(int8_t asc);
uint8_t str2uchar(int8_t *str);
uint16_t str2uint(int8_t *str);
uint32_t str2ulong (int8_t *str);

uint32_t GetTickElapse(uint32_t uiPreTick);
void exchange_sort16(uint16_t* pData,uint16_t Count);
uint16_t get_average16(uint16_t* dat, uint16_t len);

#endif
