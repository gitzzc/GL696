/**
  ******************************************************************************
  * @file    ADC/RegSimul_DualMode/main.c 
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "adc.h"
#include "config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    		((uint32_t)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t ADC_ConvertedValueTab[ADC_SAMPLE_TIMES][ADC_CHANNEL];
psADC_CONFIG psADC_Config = NULL;
    
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void ADC_RCC_Configuration(void)
{
	/* ADCCLK = PCLK2/6 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	/* Enable peripheral clocks ------------------------------------------------*/
	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable ADC1, GPIOB and GPIOC clock */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_ADC1 | \
		                    RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, \
							ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void ADC_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Configure analog input ----------------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief   ADC_Init program
  * @param  None
  * @retval None
  */
void ADC_InitChannel(void)
{
	ADC_InitTypeDef ADC_InitStructure;
//  DMA_InitTypeDef DMA_InitStructure;
	
	/* System clocks configuration ---------------------------------------------*/
	ADC_RCC_Configuration();
	
	/* GPIO configuration ------------------------------------------------------*/
	ADC_GPIO_Configuration();
#if 0
  /* DMA1 channel1 configuration ----------------------------------------------*/
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr 		= (uint32_t)ADC_ConvertedValueTab;
	DMA_InitStructure.DMA_DIR 				= DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize 			= ADC_CHANNEL*ADC_SAMPLE_TIMES;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode 				= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority 			= DMA_Priority_High;
	DMA_InitStructure.DMA_M2M 				= DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 Channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
#endif
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	/* ADC1 regular channels configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8,   1, ADC_SampleTime_239Cycles5);    
	/*
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9,   2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10,  3, ADC_SampleTime_239Cycles5);    
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11,  4, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12,  5, ADC_SampleTime_239Cycles5);    
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13,  6, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14,  7, ADC_SampleTime_239Cycles5);    
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15,  8, ADC_SampleTime_239Cycles5);
	*/
	/* Enable ADC1 DMA */
	//ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable Vrefint channel17 */
	ADC_TempSensorVrefintCmd(ENABLE);
	
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* Start ADC1 Software Conversion */ 
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
	/* Test on DMA1 channel1 transfer complete flag */
	//while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
	/* Clear DMA1 channel1 transfer complete flag */
	//DMA_ClearFlag(DMA1_FLAG_TC1);
}

uint16_t ADC_Get(uint8_t ch,uint16_t* buf,uint16_t count)
{
	uint16_t i;
	
	ADC_RegularChannelConfig(ADC1, ch,1, ADC_SampleTime_239Cycles5);    
	for(i=0;i<count;i++){
		ADC_Cmd(ADC1, ENABLE);
		while( ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET );
		buf[i] = ADC_GetConversionValue(ADC1);
	}
	//ADC_Cmd(ADC1, DISABLE);
	return 0;
}

void exchange_sort16(uint16_t* pData,uint16_t Count)
{
	uint16_t iTemp;
	uint16_t i,j;
	
	for(i=0;i<Count-1;i++){
		for(j=i+1;j<Count;j++){
			if(pData[j]<pData[i]){
				iTemp = pData[i];
				pData[i] = pData[j];
				pData[j] = iTemp;
			}
		}
	}
}

uint16_t get_average16(uint16_t* dat, uint16_t len)
{
	uint32_t sum;
	uint16_t i;
	
	for(sum=0,i=0;i<len;i++)
		sum += dat[i];	
	return (sum/len);
}

void ADC_SetConfig(	psADC_CONFIG cfg)
{
	if ( cfg == NULL )
		return;
	psADC_Config = cfg;
}

uint16_t ADC_GetAverage(uint32_t ch)
{
	uint16_t buf[ADC_SAMPLE_TIMES];
	uint32_t i;
	uint32_t sample;

	if ( ch >= ADC_CHANNEL || psADC_Config == NULL )
		return 0;
	

	for(i=0;i<ADC_SAMPLE_TIMES;i++)
		buf[i] = ADC_ConvertedValueTab[i][ch];
	exchange_sort16(buf,ADC_SAMPLE_TIMES);
	sample = get_average16(buf + 4, ADC_SAMPLE_TIMES - 4*2);
	return (sample*psADC_Config[ch].scale/10000);
}

uint32_t ADC_GetConfig(psADC_CONFIG cfg)
{
	uint32_t ret;
	uint32_t i;

	ret = ConfigRead(CONFIG_ADDRESS1,(uint8_t*)cfg,sizeof(sADC_CONFIG)*ADC_CHANNEL);
	if ( ret == pdFALSE ){
		ret = ConfigRead(CONFIG_ADDRESS2,(uint8_t*)cfg,sizeof(sADC_CONFIG)*ADC_CHANNEL);
		if ( ret == pdFALSE ){
			for(i=0;i<ADC_CHANNEL;i++){
				cfg[i].scale = 10000;		
				cfg[i].offset= 0;
			}
			ret = ConfigWrite(CONFIG_ADDRESS1,(uint8_t*)cfg,sizeof(sADC_CONFIG)) ;
			if ( (ConfigWrite(CONFIG_ADDRESS2,(uint8_t*)cfg,sizeof(sADC_CONFIG)) == pdFALSE) && (ret == pdFALSE) )
				return pdFALSE;
		}
	}
	
	return ret;
}



/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
