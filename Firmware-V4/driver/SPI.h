#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f10x.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "semphr.h"

void SPI_Bus_init(void);
uint16_t SPI_Send(SPI_TypeDef* SPIx,uint16_t dat);
signed portBASE_TYPE SPI_Take( SPI_TypeDef* SPIx, portTickType delay);
signed portBASE_TYPE SPI_Give( SPI_TypeDef* SPIx );

#endif 
