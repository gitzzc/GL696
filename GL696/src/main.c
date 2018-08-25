/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    07/16/2010 
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
#include <string.h>

//#include "stm32_eth.h"
#include "ethernetif.h"
#include "netconf.h"
#include "lwip/mem.h"
#include "lwip/def.h"
#include "lwip/udp.h"
#include "lwip/igmp.h"

#include "main.h"
#include "projdefs.h"
#include "gpio.h"
#include "led.h"
#include "modbus.h"
#include "serial.h"
#include "config.h"
#include "lcd.h"
#include "window.h"
#include "win_main.h"
#include "keyboard.h"

/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);

void tcp_server_init(void);
void MB_tcp_server_init(void);

/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;
pFunction Jump_To_Application;
uint32_t JumpAddress;

const sSYS_CFG sSys_def_cfg = 
{
	//------0-7----------
	0,					//year
	0,					//date
	0,					//hour
	0,					//time
	0x0003,			//hwid
	0x0003,			//apid
	0x0025,			//fmver
	0x0000,			//key
	
	//------8-15---------
	{0x0000,0x0000},//serial[]
	0,					//update
	{0,},				//reserve1[];	

	//------16-63---------
	{0x0002,0x0000,0x0000},			//mac[3]
	0,									//ip_mode;
	{9	,2	,168,192},	//ip_addr[4]
	{0	,255,255,255},	//ip_mask[4]
	{253,2	,168,192},	//gateway[4]
	{0	,0	,0	,0	},	//dns1[4]
	{0	,0	,0	,0	},	//dns2[4]
	{1	,2	,168,192},	//tcp_host[4]
	15009,							//tcp_host_port;
	15000,							//tcp_client_port;
	{9	,2	,1	,227},	//udp_host[4]
	5029,								//udp_host_port;
	5028,								//udp_client_port;
	{0,},								//reserve2[];	
	
	//------64-111---------
	{						//sADC_cfg[0...15]
		{0,1000,0},{0,1000,0},{0,1000,0},{0,1000,0},			//sADC_cfg[ 0... 3]
		{0,1000,0},{0,1000,0},{0,1000,0},{0,1000,0},			//sADC_cfg[ 4... 7]
		{0,1000,0},{0,1000,0},{0,1000,0},{0,1000,0},			//sADC_cfg[ 8...11]
		{0,1000,0},{0,1000,0},{0,1000,0},{0,1000,0},			//sADC_cfg[12...15]
	},
};

sSYS_CFG 			sSys_cfg;
sLED_CONFIG 	sLED_cfg;

uint8_t DI_Bit_Buf[32/8];
uint32_t alarm_new,alarm_total = 0;
uint8_t LastTimeCode[14];
extern uint8_t MB_FrameBuf[];

/* Private function prototypes -----------------------------------------------*/
void System_Periodic_Handle(void);

#if 0
void MBM_Read(uint8_t slave)
{
	uint8_t buf[16];
	uint8_t index=0;

	buf[index++] 	= slave+1;	//addr
	buf[index++]	= 0x03;		//fn
	buf[index++]	= 0x00;		//start
	buf[index++]	= 0x00;		//
	buf[index++]	= 0x00;		//reg_len
	buf[index++]	= 0x05;		//
	MB_SerialSend(0,buf,index,MB_FRAME_RTU);
	//vSerialWaitTx(0);
}

void MBM_WriteReg(uint8_t slave,uint16_t reg,uint16_t value)
{
	uint8_t buf[16];
	uint8_t index=0;

	buf[index++] 	= slave+1;//addr
	buf[index++]	= 0x10;		//fn
	buf[index++]	= reg>>8;	//start
	buf[index++]	= reg;		//
	buf[index++]	= 0x00;		//reg_len
	buf[index++]	= 0x01;		//
	buf[index++]	= 0x02;		//reg_len*2
	buf[index++]	= value>>8;//dat
	buf[index++]	= value;	 
	MB_SerialSend(0,buf,index,MB_FRAME_RTU);
	//vSerialWaitTx(0);
}

void MBM_Task(void)
{
	#define FRAME_TIME	HZ_TICK/20
	
	static sTIMEOUT sFTime;
	static uint32_t rx_index=0;
	static uint8_t  slave_id=0;
	static uint8_t  try=0;
	signed char ch;
	uint16_t	read_err=0;
	uint8_t *frame;
	uint8_t *pcAdu;
	uint8_t type;
	uint8_t  i;
	uint16_t byte_count;
	uint16_t reg_value,reg_addr;
	uint8_t to_ret;
	
	if ( (to_ret=get_timeout(&sFTime)) == TO_TIMEOUT ){
		start_timeout(&sFTime,FRAME_TIME);
		
		if ( (pcAdu = MB_CheckFrame(MB_FrameBuf,rx_index,&type)) > 0 ){
			if ( pcAdu[MB_ADU_ADDR] == slave_id + 1 ) {		//设备地址？
				sSys_cfg.spmu[slave_id].status &= 0xF0FF;		//清除通讯中断计数
				sSys_cfg.spmu[slave_id].status &= ~PMU_TO;	//通讯中断标志
				switch( pcAdu[MB_ADU_FN] ){
					case 0x03:
						byte_count = pcAdu[0x02];
						for(i=0;i<byte_count/2 && i<16;i++){
							reg_value = ((uint16_t)pcAdu[0x03+i*2]<<8) | pcAdu[0x04+i*2];
							switch(i){
								case 0: //电压
									sSys_cfg.spmu[slave_id].vol 	= reg_value*10; 
									break;
								case 1: //电流
									sSys_cfg.spmu[slave_id].cur 	= reg_value*10; 
									break;
								case 2: //过压
									if ( reg_value == 0x00 ) 
												sSys_cfg.spmu[slave_id].status &= ~PMU_OV;
									else	sSys_cfg.spmu[slave_id].status |=  PMU_OV; 
								break;
								case 3: //过流
									if ( reg_value == 0x00 ) 
												sSys_cfg.spmu[slave_id].status &= ~PMU_OC;
									else	sSys_cfg.spmu[slave_id].status |=  PMU_OC; 
									break;
								case 4: //过温
									if ( reg_value == 0x00 )
												sSys_cfg.spmu[slave_id].status &= ~PMU_OT;
									else	sSys_cfg.spmu[slave_id].status |=  PMU_OT; 
									break;
								case 5: //自检状态
									if ( reg_value == 0x00 )
												sSys_cfg.spmu[slave_id].status &= ~PMU_ERR;
									else	sSys_cfg.spmu[slave_id].status |=  PMU_ERR;
									break;
								default: break;
							}
						}
						break;
					case 0x10:
						reg_addr = ((uint16_t)pcAdu[0x02]<<8) | pcAdu[0x03];
						switch( reg_addr ){
							case 0x0101:
								sSys_cfg.spmu[slave_id].status &=~(PMU_UPDATE_CTRL);break;
							case 0x0100:
								sSys_cfg.spmu[slave_id].status &=~(PMU_UPDATE_VOL);	break;
							default: break;
						}
						break;
					default: 
						break;
				}
			}
		} else {
			read_err = (sSys_cfg.spmu[slave_id].status >> 8)&0x0F;
			if ( read_err++ > 10 ){	//通讯中断计数
				read_err = 10;
				sSys_cfg.spmu[slave_id].status |= PMU_TO;	//通讯中断标志
				
				sSys_cfg.spmu[slave_id].vol = 0;	//2016.12.2 通讯中断后清零
				sSys_cfg.spmu[slave_id].cur = 0;	//2016.12.2 通讯中断后清零
			}
			sSys_cfg.spmu[slave_id].status = (sSys_cfg.spmu[slave_id].status & 0xF0FF) | (read_err<<8);
				
			if 				( (try&0x01) == 0 && sSys_cfg.spmu[slave_id].status & PMU_UPDATE_CTRL ) {	//有新状态
				try |= 0x01;
				MBM_WriteReg(slave_id,0x0101,sSys_cfg.spmu[slave_id].ctrl);
			} else if ( (try&0x02) == 0 && sSys_cfg.spmu[slave_id].status & PMU_UPDATE_VOL  ) {	//有新状态
				try |= 0x02;
				MBM_WriteReg(slave_id,0x0100,sSys_cfg.spmu[slave_id].vol_set/10);
			} else {
				try = 0;
				if ( ++slave_id >= PMU_MAX )
					slave_id = 0;
				MBM_Read(slave_id);
			}
		}
		rx_index = 0;
	} else if ( to_ret != TO_RUNING ){
		start_timeout(&sFTime,FRAME_TIME);
	}
	
	if ( xSerialGetChar( 0, &ch, 0 ) == pdTRUE ){
		start_timeout(&sFTime,FRAME_TIME);
		if ( rx_index >= MB_BUF_SIZE )
			rx_index = 0;
		MB_FrameBuf[rx_index++] = ch;
	}
}

void PAU_task(void)
{
	#define PAU_TIME	HZ_TICK/100	
	static sTIMEOUT sTO_Dio;
	static uint8_t 	dio_count=0;
	static uint8_t	adc_mux=ADC_VOL3;
	uint8_t ret;
	uint32_t key;

	if ( (ret=get_timeout(&sTO_Dio)) == TO_TIMEOUT ){	//10ms
		start_timeout(&sTO_Dio,PAU_TIME);
		dio_count ++;

		DO_DelayTask(10);
		
		DI_ReadAll(DI_Bit_Buf,sizeof(DI_Bit_Buf));
		alarm_new = DIO_MonitorTask(bitFilter,DI_Bit_Buf,32,&alarm_total);
		key = 0;
		key |= (bitFilter[FT_KEY1 ].level==1)?(1<<0 ):0;
		key |= (bitFilter[FT_KEY2 ].level==1)?(1<<1 ):0;
		key |= (bitFilter[FT_KEY3 ].level==1)?(1<<2 ):0;
		key |= (bitFilter[FT_KEY4 ].level==1)?(1<<3 ):0;
		KB_SetScanCode(key);

		sSys_cfg.pout  = 0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT1 ].bit==1)?(1<<0 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT2 ].bit==1)?(1<<1 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT3 ].bit==1)?(1<<2 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT4 ].bit==1)?(1<<3 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT5 ].bit==1)?(1<<4 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT6 ].bit==1)?(1<<5 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT7 ].bit==1)?(1<<6 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT8 ].bit==1)?(1<<7 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT9 ].bit==1)?(1<<8 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT10].bit==1)?(1<<9 ):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT11].bit==1)?(1<<10):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT12].bit==1)?(1<<11):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT13].bit==1)?(1<<12):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT14].bit==1)?(1<<13):0;
		sSys_cfg.pout |= (bitFilter[FT_VOUT15].bit==1)?(1<<14):0;

		if ( (dio_count % 50) == 0 ) {	//500ms
			sSys_cfg.oc	   = 0;
			sSys_cfg.oc 	|= (bitFilter[FT_OC1 ].bit==1)	?(1<<0 ):0;
			sSys_cfg.oc 	|= (bitFilter[FT_OC2 ].bit==1)	?(1<<1 ):0;
			sSys_cfg.oc 	|= (bitFilter[FT_OC3 ].bit==1)	?(1<<2 ):0;
			sSys_cfg.oc 	|= (bitFilter[FT_OC4 ].bit==1)	?(1<<3 ):0;
			sSys_cfg.oc 	|= (bitFilter[FT_OC5 ].bit==1)	?(1<<4 ):0;
			sSys_cfg.oc 	|= (bitFilter[FT_OC6 ].bit==1)	?(1<<5 ):0;

			sSys_cfg.pau_st	 = 0;
			sSys_cfg.pau_st |= (bitFilter[FT_STATE1].bit==1)?(1<<1 ):0;

			sSys_cfg.vol[0] = ADC_GetAverage(ADC_VOL1)*ADC_VREF/3000*4200 /4096;//42.00V - 3.0V
			if ( sSys_cfg.vol[0] < 1000 ) sSys_cfg.vol[0] = 0;	//<10.00V
			sSys_cfg.vol[1] = sSys_cfg.vol[0];
			sSys_cfg.vol[6] = ((sSys_cfg.pout>>6 )&0x01)?sSys_cfg.vol[0]:0;
			sSys_cfg.vol[7] = ((sSys_cfg.pout>>7 )&0x01)?sSys_cfg.vol[0]:0;
			sSys_cfg.vol[8] = ((sSys_cfg.pout>>8 )&0x01)?sSys_cfg.vol[0]:0;
			sSys_cfg.vol[9] = ((sSys_cfg.pout>>9 )&0x01)?sSys_cfg.vol[0]:0;

			sSys_cfg.vol[2] = ADC_GetAverage(ADC_VOL2)*ADC_VREF/3000*12000/4096;//120.00V- 3.0V
			if ( sSys_cfg.vol[2] < 1000 ) sSys_cfg.vol[2] = 0;//<10.00V

			sSys_cfg.cur[0] = (int16_t)(ADC_GetAverage(ADC_CUR1)*ADC_VREF/2500*5000	/4096);//50.00A - 2.5V
			//if ( sSys_cfg.cur[0] < 20 ) sSys_cfg.cur[0] = 0;	//<0.2A
			sSys_cfg.cur[1] = (int16_t)(ADC_GetAverage(ADC_CUR2)*ADC_VREF/2500*5000	/4096);//50.00A - 2.5V
			//if ( sSys_cfg.cur[1] < 20 ) sSys_cfg.cur[1] = 0;
			sSys_cfg.cur[2] = (int16_t)(ADC_GetAverage(ADC_CUR3)*ADC_VREF/3000*3000 /4096);//30.00A - 3.0V
			//if ( sSys_cfg.cur[2] < 20 ) sSys_cfg.cur[2] = 0;
			sSys_cfg.cur[3] = (int16_t)(ADC_GetAverage(ADC_CUR4)*ADC_VREF/3000*5000 /4096);//80.00A - 3.0V
			//if ( sSys_cfg.cur[3] < 20 ) sSys_cfg.cur[3] = 0;
			sSys_cfg.cur[4] = (int16_t)(ADC_GetAverage(ADC_CUR5)*ADC_VREF/3000*5000 /4096);//80.00A - 3.0V
			//if ( sSys_cfg.cur[4] < 20 ) sSys_cfg.cur[4] = 0;
			sSys_cfg.cur[5] = (int16_t)(ADC_GetAverage(ADC_CUR6)*ADC_VREF/2500*5000 /4096);//50.00A - 2.5V
			//if ( sSys_cfg.cur[5] < 40 ) sSys_cfg.cur[5] = 0;
		}

		if ( (dio_count % 10) == 0 ) {	//100ms
			switch(adc_mux){
			case ADC_VOL3:
				sSys_cfg.vol[3] 	=  (int16_t)(ADC_GetAverage(ADC_MUX			 )*ADC_VREF/3000*4200/4096);//42.00V - 3.0V
				if ( sSys_cfg.vol[3] < 1000 ) sSys_cfg.vol[3] = 0;//<10.00V
				sSys_cfg.vol[4] 	= sSys_cfg.vol[3];
				adc_mux = ADC_VOL4;
				break;
			case ADC_VOL4:
				sSys_cfg.vol[5] 	=  (int16_t)(ADC_GetAverage(ADC_MUX			 )*ADC_VREF/3000*4200/4096);//42.00V - 3.0V
				if ( sSys_cfg.vol[5] < 1000 ) sSys_cfg.vol[5] = 0;//<10.00V
				sSys_cfg.vol[12]	= (sSys_cfg.pout>>12)&0x01?sSys_cfg.vol[5]:0;
				sSys_cfg.vol[13]	= (sSys_cfg.pout>>13)&0x01?sSys_cfg.vol[5]:0;
				adc_mux = ADC_LD1;
				break;
			case ADC_LD1:	
				sSys_cfg.cur_ld[0]= (int16_t)((ADC_GetAverage(ADC_MUX)*ADC_VREF/4096-1650)*500/1650);//0-3.3V,0V->-5V,1.65V-0V,3.3V->5V
				if ( sSys_cfg.cur_ld[0] > -50 && sSys_cfg.cur_ld[0] < 50 ) sSys_cfg.cur_ld[0] = 0;//0.50V
				if ( sSys_cfg.cur_ld[0] >  380 ) sSys_cfg.cur_ld[0] =  500;//3.0-1.65 -> 5V
				if ( sSys_cfg.cur_ld[0] < -500 ) sSys_cfg.cur_ld[0] = -500;
				adc_mux = ADC_LD3;
				break;
			case ADC_LD3:	
//				sSys_cfg.cur_ld[2]= (int16_t)((ADC_GetAverage(ADC_MUX)-2048)*ADC_VREF/3000*500 /2048);//5.00V	- 3.0V
				sSys_cfg.cur_ld[2]= (int16_t)((ADC_GetAverage(ADC_MUX)*ADC_VREF/4096-1650)*500/1650);//0-3.3V,0V->-5V,1.65V-0V,3.3V->5V
				if ( sSys_cfg.cur_ld[2] > -50 && sSys_cfg.cur_ld[2] < 50 ) sSys_cfg.cur_ld[2] = 0;//0.50V
				if ( sSys_cfg.cur_ld[2] >  500 ) sSys_cfg.cur_ld[2] =  500;
				if ( sSys_cfg.cur_ld[2] < -500 ) sSys_cfg.cur_ld[2] = -500;
				adc_mux = ADC_LD4;
				break;
			case ADC_LD4:	
//				sSys_cfg.cur_ld[3]= (int16_t)((ADC_GetAverage(ADC_MUX)-2048)*ADC_VREF/3000*500 /2048);//5.00V	- 3.0V
				sSys_cfg.cur_ld[3]= (int16_t)((ADC_GetAverage(ADC_MUX)*ADC_VREF/4096-1650)*500/1650);//0-3.3V,0V->-5V,1.65V-0V,3.3V->5V
				if ( sSys_cfg.cur_ld[3] > -50 && sSys_cfg.cur_ld[3] < 50 ) sSys_cfg.cur_ld[3] = 0;//0.50V
				if ( sSys_cfg.cur_ld[3] >  500 ) sSys_cfg.cur_ld[3] =  500;
				if ( sSys_cfg.cur_ld[3] < -500 ) sSys_cfg.cur_ld[3] = -500;
				adc_mux = ADC_VOL3;
				break;
			default:
				adc_mux = ADC_VOL3;
				break;
			}
			ADC_SetChannel(adc_mux);		
		}	
	} else if ( ret != TO_RUNING ){
		start_timeout(&sTO_Dio,PAU_TIME);
	}
}


/* Private functions ---------------------------------------------------------*/
void UDP_task(void)
{
	#define UDP_HZ (HZ_TICK/20)		//50ms

	static sTIMEOUT sTO_upd;
	struct udp_pcb *upcb;
	struct ip_addr ipaddr;
	struct pbuf *p;
	uint8_t ret;
	uint8_t* buf;
	uint16_t i,index,sum;		
	int iRet = 1; 	

	if ( (ret=get_timeout(&sTO_upd)) == TO_TIMEOUT ){	//50ms
		start_timeout(&sTO_upd,UDP_HZ);
		
		/* Create a new UDP control block  */
		upcb = udp_new();
		if ( upcb == NULL ) return ;
	
    IP4_ADDR(&ipaddr, sSys_cfg.udp_host[3],sSys_cfg.udp_host[2],sSys_cfg.udp_host[1],sSys_cfg.udp_host[0]);
		#if LWIP_IGMP
		iRet = igmp_joingroup(IP_ADDR_ANY,(struct ip_addr *) (&ipaddr));
		#endif		
		
		/* Connect the upcb  */
		udp_connect(upcb, &ipaddr, sSys_cfg.udp_host_port);
		p = pbuf_alloc(PBUF_TRANSPORT, 200, PBUF_RAM);
		if ( p == NULL )		return;
		
		//----------------------------------------------------------------------------
		index = 0;
		buf = ((uint8_t *)p->payload); 
		buf[index++] = 0xEB;//HEAD
		buf[index++] = 0x90;
		buf[index++] = 0x00;//LENGTH
		buf[index++] = 0x00;
		for(i=0;i<14;i++)
			buf[index++] = LastTimeCode[i];
		for(i=0;i<10;i++){
			buf[index++] = sSys_cfg.vol[i];		//Utl1-10
			buf[index++] = sSys_cfg.vol[i]>>8;
		}
		buf[index++] = sSys_cfg.vol[12];		//Utl3
		buf[index++] = sSys_cfg.vol[12]>>8;
		buf[index++] = sSys_cfg.vol[13];		//Utl4
		buf[index++] = sSys_cfg.vol[13]>>8;
		for(i=0;i<6;i++){
			buf[index++] = sSys_cfg.cur[i];		//Itl1-6
			buf[index++] = sSys_cfg.cur[i]>>8;
		}
		for(i=0;i<10;i++){
			buf[index++] = (sSys_cfg.pout	>>i )&0x01;		//Stl1-10
		}
		buf[index++] = (sSys_cfg.pout	>>10)&0x01;		//Stl11-1
		buf[index++] = (sSys_cfg.pout	>>10)&0x01;		//Stl11-2
		buf[index++] = (sSys_cfg.pout	>>11)&0x01;		//Stl12-1
		buf[index++] = (sSys_cfg.pout	>>11)&0x01;		//Stl12-2
		buf[index++] = (sSys_cfg.pout	>>12)&0x01;		//Stl13
		buf[index++] = (sSys_cfg.pout	>>13)&0x01;		//Stl14
		for(i=0;i<6;i++){
			buf[index++] = (sSys_cfg.oc	>>i)&0x01;		//Sgl1-6
		}
		for(i=0;i<6;i++){
			buf[index++] = (sSys_cfg.reset>>i)&0x01;		//Sfw1-6
		}
		//----------------------------------------------------------------------------
		if ( sSys_cfg.cur_ld[0] >= 0 ) {
			buf[index++] = -sSys_cfg.cur_ld[0];					//Ild1+
			buf[index++] = -sSys_cfg.cur_ld[0]>>8;
			buf[index++] = 0;					//Ild1-
			buf[index++] = 0;
		} else {
			buf[index++] = 0;					//Ild1+
			buf[index++] = 0;
			buf[index++] = -sSys_cfg.cur_ld[0];					//Ild1-
			buf[index++] = -sSys_cfg.cur_ld[0]>>8;
		}
		if ( sSys_cfg.cur_ld[2] >= 0 ) {
			buf[index++] = -sSys_cfg.cur_ld[2];					//Ild3+
			buf[index++] = -sSys_cfg.cur_ld[2]>>8;
			buf[index++] = 0;					//Ild3-
			buf[index++] = 0;
		} else {
			buf[index++] = 0;					//Ild3+
			buf[index++] = 0;
			buf[index++] = -sSys_cfg.cur_ld[2];					//Ild3-
			buf[index++] = -sSys_cfg.cur_ld[2]>>8;
		}
		if ( sSys_cfg.cur_ld[3] >= 0 ) {
			buf[index++] = -sSys_cfg.cur_ld[3];					//Ild4+
			buf[index++] = -sSys_cfg.cur_ld[3]>>8;
			buf[index++] = 0;					//Ild4-
			buf[index++] = 0;
		} else {
			buf[index++] = 0;					//Ild4+
			buf[index++] = 0;
			buf[index++] = -sSys_cfg.cur_ld[3];					//Ild4-
			buf[index++] = -sSys_cfg.cur_ld[3]>>8;
		}
		//----------------------------------------------------------------------------
		for(i=0;i<PMU_MAX;i++){
			buf[index++] = sSys_cfg.spmu[i].vol;		//电源模块电压1-8
			buf[index++] = sSys_cfg.spmu[i].vol>>8;
			buf[index++] = sSys_cfg.spmu[i].cur;		//电源模块电流1-8
			buf[index++] = sSys_cfg.spmu[i].cur>>8;
			buf[index++] = (sSys_cfg.spmu[i].status&PMU_OV )?1:0;	//电源模块过压1-8
			buf[index++] = (sSys_cfg.spmu[i].status&PMU_OC )?1:0;	//电源模块过流1-8
			buf[index++] = (sSys_cfg.spmu[i].status&PMU_ERR)?1:0;	//电源模块自检1-8
		}
		//----------------------------------------------------------------------------
		buf[index++] = (sSys_cfg.pau_st>>0)&0x01;	//Spdkz
		buf[index++] = (sSys_cfg.pau_st>>1)&0x01;	//Spdkz1
		buf[index++] = (sSys_cfg.pau_st>>2)&0x01;	//Spdkz2
		//----------------------------------------------------------------------------
		for(sum=0,i=2;i<index;i++)
			sum += buf[i];
		buf[index++] = sum;
		buf[index++] = sum>>8;
		buf[index++] = 0x14;
		buf[index++] = 0x6F;
		buf[2]		 =  index-6;		//length
		buf[3] 		 = (index-6)>>8;	//length>>8
		//----------------------------------------------------------------------------
		
		p->tot_len = index;
		p->len 		 = index;
		
		/* Send out an UDP datagram to inform the server that we have strated a client application */
		udp_send(upcb, p);   
		/* Reset the upcb */
		udp_disconnect(upcb);
		/* Free the upcb sturct */
		udp_remove(upcb);
		/* Free the p buffer */
		pbuf_free(p);

	} else if ( ret != TO_RUNING ){
		start_timeout(&sTO_upd,UDP_HZ);
	}
}
#endif

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	uint8_t ch;

	System_Setup();
	IWDG_init();

	DIO_Init(); 
	ConfigRead(&sSys_cfg,&sSys_def_cfg);
	ADC_SetConfig(&sSys_cfg.sADC_cfg[0]);
	ADC_InitChannel();
	xSerialPortInitMinimal(serCOM1,ser115200,serNO_PARITY,serBITS_8,serSTOP_1);
//	vSerialPutString(0,"hello word.\r\n",0);
	//MB_Init(MB_DEV_ADDR,9600);
	//KB_Init();
	//LCD_Init();
	//Win_Init();

	/* Initilaize the LwIP stack */
	//LwIP_Init();
	//tcp_server_init();
	//MB_tcp_server_init();

	LED_SetStartup(&sLED_cfg,HZ_TICK/10,HZ_TICK/10,10);
	LED_SetDefault(&sLED_cfg,HZ_TICK/2,HZ_TICK);

	/* Infinite loop */
	while (1)
	{
		LED_Task(&sLED_cfg);
		//PAU_task();
		//MBM_Task();
		//UDP_task();
		//KB_Task();
		//Win_Task();

		/* Periodic tasks */
		System_Periodic_Handle();

		/* Reload IWDG counter */
		IWDG_ReloadCounter();
	}
}


/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

/**
  * @brief  Handles the periodic tasks of the system
  * @param  None
  * @retval None
  */
void System_Periodic_Handle(void)
{
  /* LwIP periodic services are done here */
  LwIP_Periodic_Handle(LocalTime);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc( int ch, FILE *f )
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	(void)f;
	
	xSerialPutChar( 0, ch, 0 );
	return ch;
}

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
 
  printf("%s %d\r\n",file,line);

  /* Infinite loop */
  while (1)
  {}
}
#endif


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
