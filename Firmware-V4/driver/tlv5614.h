#ifndef __TLV5614_H__
#define __TLV5614_H__


#define DACA (0<<14)
#define DACB (1<<14)
#define DACC (2<<14)
#define DACD (3<<14)


#define TLV5614_PDWN (1<<13)
#define TLV5614_FSPD (1<<12)

#define TLV5614_REG_VAL_MASK 0X0FFF
#define TLV5614_CHANNEL_MASK 0xC000


#define DAC_BITS 		12
#define DAC_FULL 		5000
#define DAC_VREF		2500
#define DAC_CHANNEL		4


void TLV5614_init(void);
void TLV5614_set_out(uint8_t ch ,uint16_t dac);
void dac_set_mv(uint8_t ch ,uint16_t mv);


#endif
