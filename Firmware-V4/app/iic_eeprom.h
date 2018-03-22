/**
  ******************************************************************************
  * @file    stm3210b_eval.h
  * @author  MCD Application Team
  * @version V4.2.0
  * @date    04/16/2010
  * @brief   This file contains definitions for STM3210B_EVAL's Leds, push-buttons
  *          COM ports, SD Card (on SPI), sFLASH (on SPI) and Temperature sensor 
  *          sEE (on I2C) hardware resources.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210B_EVAL_H
#define __STM3210B_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f10x.h"

#include "stm32_eval_i2c_ee.h"

/** @addtogroup STM3210C_EVAL_LOW_LEVEL_I2C_EE
  * @{
  */
/**
  * @brief  I2C EEPROM Interface pins
  */  
#define sEE_I2C                          I2C1
#define sEE_I2C_CLK                      RCC_APB1Periph_I2C1
#define sEE_I2C_SCL_PIN                  GPIO_Pin_6                  /* PB.06 */
#define sEE_I2C_SCL_GPIO_PORT            GPIOB                       /* GPIOB */
#define sEE_I2C_SCL_GPIO_CLK             RCC_APB2Periph_GPIOB
#define sEE_I2C_SDA_PIN                  GPIO_Pin_7                  /* PB.07 */
#define sEE_I2C_SDA_GPIO_PORT            GPIOB                       /* GPIOB */
#define sEE_I2C_SDA_GPIO_CLK             RCC_APB2Periph_GPIOB
#define sEE_I2C_WP_PIN                   GPIO_Pin_6                  /* PD.06 */
#define sEE_I2C_WP_GPIO_PORT             GPIOD                       /* GPIOD */
#define sEE_I2C_WP_GPIO_CLK              RCC_APB2Periph_GPIOD
#define sEE_M24C64_32

#define sEE_I2C_DMA                      DMA   
#define sEE_I2C_DMA_CHANNEL_TX           DMA1_Channel6
#define sEE_I2C_DMA_CHANNEL_RX           DMA1_Channel7 
#define sEE_I2C_DMA_FLAG_TX_TC           DMA1_IT_TC6   
#define sEE_I2C_DMA_FLAG_TX_GL           DMA1_IT_GL6 
#define sEE_I2C_DMA_FLAG_RX_TC           DMA1_IT_TC7 
#define sEE_I2C_DMA_FLAG_RX_GL           DMA1_IT_GL7    
#define sEE_I2C_DMA_CLK                  RCC_AHBPeriph_DMA1
#define sEE_I2C_DR_Address               ((uint32_t)0x40005410)
#define sEE_USE_DMA
   
#define sEE_I2C_DMA_TX_IRQn              DMA1_Channel6_IRQn
#define sEE_I2C_DMA_RX_IRQn              DMA1_Channel7_IRQn
#define sEE_I2C_DMA_TX_IRQHandler        DMA1_Channel6_IRQHandler
#define sEE_I2C_DMA_RX_IRQHandler        DMA1_Channel7_IRQHandler   
#define sEE_I2C_DMA_PREPRIO              0
#define sEE_I2C_DMA_SUBPRIO              0   
   
#define sEE_DIRECTION_TX                 0
#define sEE_DIRECTION_RX                 1  

/** @addtogroup STM3210B_EVAL_LOW_LEVEL_TSENSOR_I2C
  * @{
  */
/**
  * @brief  sEE Temperature Sensor I2C Interface pins
  */  
#define sEE_I2C                         I2C1
#define sEE_I2C_CLK                     RCC_APB1Periph_I2C1
#define sEE_I2C_SCL_PIN                 GPIO_Pin_6                  /* PB.06 */
#define sEE_I2C_SCL_GPIO_PORT           GPIOB                       /* GPIOB */
#define sEE_I2C_SCL_GPIO_CLK            RCC_APB2Periph_GPIOB
#define sEE_I2C_SDA_PIN                 GPIO_Pin_7                  /* PB.07 */
#define sEE_I2C_SDA_GPIO_PORT           GPIOB                       /* GPIOB */
#define sEE_I2C_SDA_GPIO_CLK            RCC_APB2Periph_GPIOB
#define sEE_I2C_SMBUSALERT_PIN          GPIO_Pin_5                  /* PB.05 */
#define sEE_I2C_SMBUSALERT_GPIO_PORT    GPIOB                       /* GPIOB */
#define sEE_I2C_SMBUSALERT_GPIO_CLK     RCC_APB2Periph_GPIOB


#define sEE_WRITE_ADDRESS1        0xa0
#define sEE_READ_ADDRESS1         0xa0

#define TC_CFG_FLAG_ADDR		0xF0
#define TC_CFG_FLAG				0xA55A

#define PDU_OUTPUT_ADDR			0xE0


/** @defgroup STM3210B_EVAL_LOW_LEVEL_Exported_Functions
  * @{
  */ 
void sEE_LowLevel_DeInit(void);
void sEE_LowLevel_Init(void); 
void sEE_LowLevel_DMAConfig(uint32_t pBuffer, uint32_t BufferSize, uint32_t Direction);

/**
  * @}
  */ 
    
#ifdef __cplusplus
}
#endif
  
#endif /* __STM3210B_EVAL_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */  

/**
  * @}
  */    

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
