#ifndef __ADC_H__
#define __ADC_H__

#define ADC_BITS	12UL
#define ADC_VREF	2500UL

#define ADC_CHANNEL			12UL
#define ADC_SAMPLE_TIMES	32UL

#define ADC_GET_MV(adc)	(adc*2L*ADC_VREF/((1<<ADC_BITS)-1))

typedef struct _ADC_CONFIG_
{
	uint32_t offset;
	uint32_t scale;
}sADC_CONFIG,*psADC_CONFIG;

//-------------------------------------------------------------------------

extern volatile uint16_t ADC_ConvertedValueTab[ADC_SAMPLE_TIMES][ADC_CHANNEL];
//-------------------------------------------------------------------------

void ADC_InitChannel(void);
void ADC_SetConfig(psADC_CONFIG cfg);
uint16_t ADC_Get(uint8_t ch,uint16_t* buf,uint16_t count);
void exchange_sort16(uint16_t* pData,uint16_t Count);
uint16_t get_average16(uint16_t* dat, uint16_t len);
uint16_t ADC_GetAverage(uint32_t ch);
int32_t Get_Temp(uint16_t ch);
uint32_t ADC_GetConfig(psADC_CONFIG cfg);


#endif //__ADC_H__
