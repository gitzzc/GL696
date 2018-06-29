#include "stdlib.h"

#include "projdefs.h"
#include "common.h"

//-------------------------------------------------------------------------
const uint8_t HEX2ASCII_TAB[]  = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
#define hex2axc(x) (HEX2ASCII_TAB[x])

uint8_t asc2hex(int8_t asc)
{
  uint8_t hex;
  if (asc >= '0' && asc <= '9')
    hex = asc - '0';
  else if (asc >= 'A' && asc <= 'F')
    hex = asc - 'A' + 0xA;
  else if (asc >= 'a' && asc <= 'f')
    hex = asc - 'a' + 0xA;
  else 
    hex = 0;
  return hex;
}

uint8_t str2uchar(int8_t *str)
{
  uint8_t ub;
  ub = (asc2hex(str[0])<<4) | asc2hex(str[1]);
  return ub;
}

uint16_t str2uint(int8_t *str)
{
  uint16_t ui;
  ui  = (uint16_t)str2uchar(str)<<8;
  ui |= str2uchar(str+2);
  return ui;
}

uint32_t str2ulong (int8_t *str)
{
  uint32_t ul;
  ul  = (uint32_t)str2uint(str)<<16;
  ul |= str2uint(str+4);
  return ul;
}

//-------------------------------------------------------------------------
short init_queue(pQUEUE q,uint16_t *buf,uint8_t size)
{
	if ( q == NULL || buf == NULL )
		return -1;
		
	q->queue = buf;
	q->size  = size;
	q->front = q->rear = 0;
	return 0;
}

//插入队尾
short enqueue(pQUEUE q,uint16_t da)
{
	q->queue[q->rear++] = da;
	q->rear %= q->size;
	
	if (q->rear == q->front){
	q->front ++;
		q->front %= q->size;
		return -1;
	}
	return 0;
}

//删除队列头元素，用x返回
short dequeue(pQUEUE q, uint16_t* x)
{
	if(q->front == q->rear)
		return -1;

	*x = q->queue[q->front];
	q->front = (q->front+1)%q->size;
	return 0;
}

//返回队列头元素
void firstqueue(pQUEUE q, uint16_t* x)
{
	*x = q->queue[q->front];
}
