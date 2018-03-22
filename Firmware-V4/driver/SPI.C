/**
  ******************************************************************************
  * @file    spi.c 
  * @author  
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @copy
  *
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "spi.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static xSemaphoreHandle xSemaphore_SPI1;
static xSemaphoreHandle xSemaphore_SPI2;

/* Private function prototypes -----------------------------------------------*/

void SPI_Bus_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//ETHERNET
	/* Enable the Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure SPI CS as output push-pull */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOE, &GPIO_InitStructure );
	GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_SET);

	//touchscreen & spi_flash
	/* Enable the Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure CS in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
	
	vSemaphoreCreateBinary( xSemaphore_SPI1 );
    if( xSemaphore_SPI1 == NULL )
		while(1);

	vSemaphoreCreateBinary( xSemaphore_SPI2 );
    if( xSemaphore_SPI2 == NULL )
		while(1);
	
}

uint16_t SPI_Send(SPI_TypeDef* SPIx,uint16_t dat)
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send byte through the SPIx peripheral */
	SPI_I2S_SendData(SPIx, dat);
	
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPIx);
}

signed portBASE_TYPE SPI_Take( SPI_TypeDef* SPIx, portTickType delay)
{
	if ( SPIx == SPI1 )
		return xSemaphoreTake( xSemaphore_SPI1, delay );
	else if ( SPIx == SPI2 )
		return xSemaphoreTake( xSemaphore_SPI2, delay );
	else
		return pdFAIL;
}

signed portBASE_TYPE SPI_Give( SPI_TypeDef* SPIx)
{
	if ( SPIx == SPI1 )
		return xSemaphoreGive( xSemaphore_SPI1 );
	else if ( SPIx == SPI2 )
		return xSemaphoreGive( xSemaphore_SPI2 );
	else
		return pdFAIL;}


