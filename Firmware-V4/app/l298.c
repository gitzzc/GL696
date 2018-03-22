#include "project.h"
#include "l298.h"

L298N_chip l298n0 = {  //U14
  AT91C_BASE_PIOA,  AT91C_PIO_PA2,	//PIN_INA	//MOTOR
  AT91C_BASE_PIOA,  AT91C_PIO_PA3,	//PIN_INB //MAGNET
  AT91C_BASE_PIOA,  0, 							//PIN_EN1
  AT91C_BASE_PIOA,  AT91C_PIO_PA4,  //PIN_INC	
  AT91C_BASE_PIOA,  AT91C_PIO_PA7,	//PIN_IND	
  AT91C_BASE_PIOA,  0,  						//PIN_EN2
};

L298N_ctrl l293d_ch0 = {&l298n0,CHANNEL_A};
L298N_ctrl l293d_ch1 = {&l298n0,CHANNEL_B};
L298N_ctrl l293d_ch2 = {&l298n0,CHANNEL_C};
L298N_ctrl l293d_ch3 = {&l298n0,CHANNEL_D};

void l298n_init(pL298N_chip l298n)
{
  AT91F_PIO_CfgOutput( l298n->pPIO_INA,  l298n->PIN_INA);
  AT91F_PIO_SetOutput( l298n->pPIO_INA,  l298n->PIN_INA);
  AT91F_PIO_CfgPullup( l298n->pPIO_INA,  l298n->PIN_INA);

  AT91F_PIO_CfgOutput( l298n->pPIO_INB,  l298n->PIN_INB);
  AT91F_PIO_SetOutput( l298n->pPIO_INB,  l298n->PIN_INB);
  AT91F_PIO_CfgPullup( l298n->pPIO_INB,  l298n->PIN_INB);

  AT91F_PIO_CfgOutput( l298n->pPIO_EN0,  l298n->PIN_EN0);
  AT91F_PIO_SetOutput( l298n->pPIO_EN0,  l298n->PIN_EN0);
  AT91F_PIO_CfgPullup( l298n->pPIO_EN0,  l298n->PIN_EN0);

  AT91F_PIO_CfgOutput( l298n->pPIO_INC,  l298n->PIN_INC);
  AT91F_PIO_SetOutput( l298n->pPIO_INC,  l298n->PIN_INC);
  AT91F_PIO_CfgPullup( l298n->pPIO_INC,  l298n->PIN_INC);

  AT91F_PIO_CfgOutput( l298n->pPIO_IND,  l298n->PIN_IND);
  AT91F_PIO_SetOutput( l298n->pPIO_IND,  l298n->PIN_IND);
  AT91F_PIO_CfgPullup( l298n->pPIO_IND,  l298n->PIN_IND);

  AT91F_PIO_CfgOutput( l298n->pPIO_EN1,  l298n->PIN_EN1);
  AT91F_PIO_SetOutput( l298n->pPIO_EN1,  l298n->PIN_EN1);
  AT91F_PIO_CfgPullup( l298n->pPIO_EN1,  l298n->PIN_EN1);
}

void l298n_control(pL298N_ctrl l298n, int st)
{
  switch(l298n->channel){
    case CHANNEL_A:
	    if (st==ON || st==RUN || st == FORWARD )
				AT91F_PIO_ClearOutput (l298n->chip->pPIO_INA,  l298n->chip->PIN_INA);
	    else 
	      AT91F_PIO_SetOutput (l298n->chip->pPIO_INA,  l298n->chip->PIN_INA);
	    break;
    case CHANNEL_B:
	    if (st==ON || st==RUN || st == FORWARD )
	      AT91F_PIO_ClearOutput (l298n->chip->pPIO_INB,  l298n->chip->PIN_INB);
	    else 
	      AT91F_PIO_SetOutput (l298n->chip->pPIO_INB,  l298n->chip->PIN_INB);
	    break;
    case CHANNEL_C:
	    if (st==ON || st==RUN || st == FORWARD )
	    	AT91F_PIO_ClearOutput (l298n->chip->pPIO_INC,  l298n->chip->PIN_INC);
	    else 
	      AT91F_PIO_SetOutput (l298n->chip->pPIO_INC,  l298n->chip->PIN_INC);
	    break;
    case CHANNEL_D:
	    if (st==ON || st==RUN || st == FORWARD )
	      AT91F_PIO_ClearOutput (l298n->chip->pPIO_IND,  l298n->chip->PIN_IND);
	    else 
	      AT91F_PIO_SetOutput (l298n->chip->pPIO_IND,  l298n->chip->PIN_IND);
	    break;
    case CHANNEL_0:
	    if (st == FORWARD ){
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INA,  l298n->chip->PIN_INA);
	      AT91F_PIO_ClearOutput(l298n->chip->pPIO_INB,  l298n->chip->PIN_INB);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_EN0,  l298n->chip->PIN_EN0);
	    } else if ( st == BACKWARD ){
	      AT91F_PIO_ClearOutput(l298n->chip->pPIO_INA,  l298n->chip->PIN_INA);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INB,  l298n->chip->PIN_INB);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_EN0,  l298n->chip->PIN_EN0);
	    } else if ( st == HOLD ){
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INA,  l298n->chip->PIN_INA);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INB,  l298n->chip->PIN_INB);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_EN0,  l298n->chip->PIN_EN0);
	    } else if ( st == FREE || st == STOP || st == OFF ){
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INA,  l298n->chip->PIN_INA);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INB,  l298n->chip->PIN_INB);
	      AT91F_PIO_ClearOutput(l298n->chip->pPIO_EN0,  l298n->chip->PIN_EN0);
	    }
	    break;
    case CHANNEL_1:
	    if (st == FORWARD ){
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INC,  l298n->chip->PIN_INC);
	      AT91F_PIO_ClearOutput(l298n->chip->pPIO_IND,  l298n->chip->PIN_IND);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_EN1,  l298n->chip->PIN_EN1);
	    } else if ( st == BACKWARD ){
	      AT91F_PIO_ClearOutput(l298n->chip->pPIO_INC,  l298n->chip->PIN_INC);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_IND,  l298n->chip->PIN_IND);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_EN1,  l298n->chip->PIN_EN1);
	    } else if ( st == HOLD ){
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INC,  l298n->chip->PIN_INC);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_IND,  l298n->chip->PIN_IND);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_EN1,  l298n->chip->PIN_EN1);
	    } else if ( st == FREE || st == STOP || st == OFF ){
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_INC,  l298n->chip->PIN_INC);
	      AT91F_PIO_SetOutput  (l298n->chip->pPIO_IND,  l298n->chip->PIN_IND);
	      AT91F_PIO_ClearOutput(l298n->chip->pPIO_EN1,  l298n->chip->PIN_EN1);
	    } 
	    break;
    default :
	    break;
  }
}


void l298n(int ch, int st)
{
  switch ( ch ){
  case 0:
    l298n_control(&l293d_ch0, st);
    break;
  case 1:
    l298n_control(&l293d_ch1, st);
    break;
  case 2:
    l298n_control(&l293d_ch2, st);
    break;
  case 3:
    l298n_control(&l293d_ch3, st);
    break; 
  default :
    break;
  }
    
}

