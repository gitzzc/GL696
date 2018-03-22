#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"

//#include "SPI_FLASH.h"
#include "spi.h"

#define SPI_FLASH_BSRR	GPIOB->BSRR
#define SPI_FLASH_NWP		GPIO_Pin_11
#define SPI_FLASH_NCS		GPIO_Pin_12

#define SPI_FLASH_WP_EN()		SPI_FLASH_BSRR = ((uint32_t)SPI_FLASH_NWP<<16)
#define SPI_FLASH_WP_DIS()	SPI_FLASH_BSRR = SPI_FLASH_NWP

#define SPI_FLASH_CS_EN()		SPI_FLASH_BSRR = ((uint32_t)SPI_FLASH_NCS<<16)
#define SPI_FLASH_CS_DIS()	SPI_FLASH_BSRR = SPI_FLASH_NCS

void SPI_FLASH_PortInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Configure control pins in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin 	= SPI_FLASH_NWP | SPI_FLASH_NCS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*****************************
 * clk=Fosc/64 
 * SPI MASTER
 * MSB FIRST
 * CPOL = 1;CPHA = 0;
 *
*******************************/
void SPI_FLASH_ConfigSPI(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef		SPI_InitStructure;

  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure SPI2 pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI2 disable */
	SPI_Cmd(SPI2, DISABLE);
	
	/* Enable SPI2 clock  */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/* SPI2 Config */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode 			= SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize 	= SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL 		= SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA 		= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 		= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit 	= SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	
	/* SPI2 enable */
	SPI_Cmd(SPI2, ENABLE);
}


void SPI_FLASH_short_delay(void)
{
	#define SPI_FLASH_DELAY	1000
	volatile uint32_t i;
	
	for(i=0;i<SPI_FLASH_DELAY;i++);
}


void SPI_FLASH_init()
{
	SPI_FLASH_PortInit();
	SPI_FLASH_ConfigSPI();
//	SPI_FLASH_powerdown();
}


void SPI_FLASH_set_out(uint8_t ch ,uint16_t dac)
{
	uint16_t tdat;

	
	SPI_FLASH_CS_EN();	SPI_FLASH_short_delay();
	
	tdat = (((uint16_t)ch<<14) | (dac&0x0FFF));
	SPI_Write( SPI2, tdat );	SPI_FLASH_short_delay();
	
	SPI_FLASH_CS_DIS();	SPI_FLASH_short_delay();
}



