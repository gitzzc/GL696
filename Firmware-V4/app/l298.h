#ifndef __l298_h__ 
#define __l298_h__ 

typedef struct __L298N_chip {
  AT91PS_PIO pPIO_INA; int PIN_INA;
  AT91PS_PIO pPIO_INB; int PIN_INB;
  AT91PS_PIO pPIO_EN0; int PIN_EN0;
  AT91PS_PIO pPIO_INC; int PIN_INC;
  AT91PS_PIO pPIO_IND; int PIN_IND;
  AT91PS_PIO pPIO_EN1; int PIN_EN1;
} *pL298N_chip,L298N_chip;

enum L298N_CH {
CHANNEL_A,
CHANNEL_B,
CHANNEL_0,
CHANNEL_C,
CHANNEL_D,
CHANNEL_1
};

typedef struct __L298N_control{
  pL298N_chip chip;
  enum L298N_CH channel;
}*pL298N_ctrl,L298N_ctrl;

extern L298N_chip l298n0;
//extern L298N_chip l298n1;

extern L298N_ctrl l293d_ch0 ;
extern L298N_ctrl l293d_ch1 ;
extern L298N_ctrl l293d_ch2 ;
extern L298N_ctrl l293d_ch3 ;


void l298n_init(pL298N_chip l298n);
void l298n_control(pL298N_ctrl l298n, int st);
void l298n(int ch, int st);

#endif 

