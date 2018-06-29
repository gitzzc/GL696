#ifndef __ADC_H__
#define __ADC_H__

#define ADC_BITS			12UL
#define ADC_VREF			3000L

#define ADC_CHANNEL			11UL
#define ADC_SAMPLE_TIMES	32UL

#define ADC_GET_MV(adc)	(adc*2L*ADC_VREF/((1<<ADC_BITS)-1))

typedef struct _ADDA_CONFIG_
{
	int16_t offset;
	int16_t scale;
	int16_t cutoff;
}sADDA_CFG,*psADDA_CFG;

//-------------------------------------------------------------------------

extern volatile uint16_t ADC_ConvertedValueTab[ADC_SAMPLE_TIMES][ADC_CHANNEL];
//-------------------------------------------------------------------------

void ADC_InitChannel(void);
void ADC_SetConfig(psADDA_CFG cfg);
uint16_t ADC_Get(uint8_t ch,uint16_t* buf,uint16_t count);
void exchange_sort16(uint16_t* pData,uint16_t Count);
uint16_t get_average16(uint16_t* dat, uint16_t len);
int32_t ADC_GetAverage(uint32_t ch);


#endif //__ADC_H__
