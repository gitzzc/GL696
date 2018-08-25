#ifndef __PWM_DAC_H__ 
#define __PWM_DAC_H__

void PWM_DAC_INIT(void);
void PWM_DAC_Set(uint8_t ch,uint16_t value);
void PWM_DAC_SetmV(uint8_t ch,uint16_t mv);

#endif
