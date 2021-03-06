#include <stdlib.h>
#include <math.h>
//#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x.h"

#include "timerout.h"

#include "gl_696h.h"
#include "adc.h"
#include "gpio.h"
#include "pwm_dac.h"
#include "serials.h"
#include "mb_reg_map.h"
#include "modbus.h"
//#include "gsm.h"

//----------------------------------------------------------------
#define FD110A_HEAD			0x80
#define FD110A_STOP			0x80
#define FD110A_START		0x81
#define FD110A_LOW_SP		0x82
#define FD110A_HIGH_SP		0x83
#define FD110A_STATUS		0x84

#define HVR				'R'
#define HVL				'L'
//----------------------------------------------------------------
#define HVL_VOL_ADC_CH	ADC_Channel_8
#define HVL_CUR_ADC_CH	ADC_Channel_12
#define HVL_VOL_DAC_CH	0
#define HVL_CUR_DAC_CH	2
#define HVL_POWER_CH	PWR_0

#define HVR_VOL_ADC_CH	ADC_Channel_9
#define HVR_CUR_ADC_CH	ADC_Channel_13
#define HVR_VOL_DAC_CH	1
#define HVR_CUR_DAC_CH	3
#define HVR_POWER_CH	PWR_1

#define VOL_DAC_FULL	5000
#define CUR_DAC_FULL	5000

//----------------------------------------------------------------
#define DO_RELAY_ON 	pdLOW
#define DO_RELAY_OFF 	pdHIGH

#define DO_POWER_ON 	pdLOW
#define DO_POWER_OFF 	pdHIGH

#define RELAY_POWERPUMP		RELAY8	//机械泵，分子泵使用同一继电器
#define RELAY_VMETER 		RELAY9

#define RELAY_BLEED_VALVE 	RELAY10
#define RELAY_BAFFLE 		RELAY11

#define RELAY_SAMPLE_MOTO	RELAY12
#define RELAY_SAMPLE_LED 	RELAY13
#define RELAY_SAMPLE_LED1 	RELAY14

//--------------------------------------------------
HVS hvsr;
HVS hvsl;
//检测定时器
sTIMEOUT hvl_vol_set_to,hvl_vol_check_to;
sTIMEOUT hvl_cur_set_to,hvl_cur_check_to;

sTIMEOUT hvr_vol_set_to,hvr_vol_check_to;
sTIMEOUT hvr_cur_set_to,hvr_cur_check_to;
sTIMEOUT sec_to;
//----------------------------------------------------------------
//真空计开关定时器，机械泵打开20S后开真空计，机械泵关闭前先关闭真空计
sTIMEOUT led_to;
//----------------------------------------------------------------
float vmeter;	//真空计测量得到的真空度
uint32_t auto_st_end  = 0xff;

//---------------------------------------------------------------------
#define CUR_ADC_FILTER_SIZE 	24
#define VOL_ADC_FILTER_SIZE 	6
#define SAMPLE_ADC_FILTER_SIZE 	12

uint16_t cur_buf_l[CUR_ADC_FILTER_SIZE];
uint16_t cur_buf_r[CUR_ADC_FILTER_SIZE];
uint16_t vol_buf_l[VOL_ADC_FILTER_SIZE];
uint16_t vol_buf_r[VOL_ADC_FILTER_SIZE];

uint16_t sample_buf[SAMPLE_ADC_FILTER_SIZE];

//uint16_t usRegHoldingBuf[256];
//uint16_t usRegInputBuf[64];

void hvs_update_to_modbus(HVS* hvs);
void hv_enbale(HVS* hvs,int32_t st);
void hvs_update_from_modbus(HVS* hvs);
//-----------------------------------------------------------------------
QUEUE vol_queue_l;
QUEUE vol_queue_r;
QUEUE cur_queue_l;
QUEUE cur_queue_r;

QUEUE sample_queue;

void init_queue(pQUEUE q,uint16_t *buf,uint8_t size)
{
	q->queue = buf;
	q->size  = size;
	q->front = q->rear = 0;
}

uint8_t enqueue(pQUEUE q,uint16_t da)
{
	q->queue[q->rear++] = da;
	if (q->rear == q->size)
		q->rear = 0;

	if (q->rear == q->front){
		q->front ++;
		if (q->front == q->size)
			q->front = 0;
		return 1;
	}

	return 0;
}

//-----------------------------------------------------------------------------------
void vmeter_set_reg(float v)
{
	vmeter = v;
	
	eMBRegInput_Write(MB_VMETER0,(((uint8_t*)&vmeter)[3]<<8) | ((uint8_t*)&vmeter)[2]);
	eMBRegInput_Write(MB_VMETER1,(((uint8_t*)&vmeter)[1]<<8) | ((uint8_t*)&vmeter)[0]);
}

//真空计电源控制
int32_t vmeter_ctl(int32_t cmd)
{
	if ( cmd & VMETER_PWR_OFF ) {
		DIO_Write( RELAY_VMETER, DO_RELAY_OFF );
		eMBRegInput_Write(MB_VMETER_ST,VMETER_PWR_OFF);

		vmeter_set_reg(1.0E3);
	} else if ( cmd & VMETER_PWR_ON ) {
		DIO_Write( RELAY_VMETER, DO_RELAY_ON );
		eMBRegInput_Write(MB_VMETER_ST,VMETER_PWR_ON);
	}
	return 0;
}

static xComPortHandle vmeter_Port = NULL;

void vmeter_init(void)
{
	eMBRegHolding_Write(VMETER_START_DELAY,10*1000);	//10S
	eMBRegHolding_Write(VMETER_STOP_DELAY,3*1000);	//3S

	vmeter_ctl(VMETER_PWR_OFF);

	vmeter_Port = xSerialPortInit( serCOM3, ser1200, serNO_PARITY, serBITS_8, serSTOP_1, 256 );
}

//-----------------------------------------------------------------------------------
//读取真空计数据，转换成32bit浮点数，存储在MODBUS寄存器中，
//其中，usRegInputBuf[MB_VMETER0]为bit31-16,usRegInputBuf[MB_VMETER1]为bit15-0
int32_t vmeter_task()
{
	static uint8_t buf[32];
	static int32_t rx_len=0;
	float vmeter;
	int32_t i;
  	
	if ( eMBRegInput_Read(MB_VMETER_ST) & VMETER_PWR_OFF )
		return 0;
	
	if ( rx_len + 9 > sizeof(buf) )
		rx_len = 0;

	i = xSerialGet(vmeter_Port,buf+rx_len,sizeof(buf)-rx_len,configTICK_RATE_HZ/100);
	if ( i == 0 )
		return 0;
	/*
	buf[0] = 0x2A;
	memcpy(buf+1,"2313----",8);
	*/
	
	rx_len += i;
	for ( i=0; rx_len-i>=9; i++ ){
		if ( buf[i] == 0x2A ){
			rx_len = 0;
			buf[i+9] = '\0';
			//usart_printf(DBGU,"\r\n真空计:%s,",buf+i+1);
			
			if ( buf[i+2+4] != '-' )//只要电离规有数据，则以些为准 
				i += 4;
			else if ( buf[i+2] == '?' || buf[i+2] == '-'){
				rx_len=0 ;
				break;
			}
				
			vmeter = buf[i+1] - '0';
			vmeter += (float)(buf[i+2] - '0')/10;
			if ( buf[i+3] == '+' )
				vmeter *= pow(10,(buf[i+4]-'0'));
			else if ( buf[i+3] == '-' )
				vmeter *= pow(10,-(buf[i+4]-'0'));

			vmeter_set_reg(vmeter);
			break;
		}
	}	
	return 0;
}

//-----------------------------------------------------------------------------------
//机械泵和分子泵共用一个继电器
void powerpump_init(void)
{
	eMBRegInput_Write(MB_POWERPUMP_ST,POWERPUMP_PWR_OFF);
}

int32_t powerpump_ctl(int32_t cmd)
{
	if ( cmd & POWER_OFF ) {
		eMBRegInput_Write(MB_SYS_AUTOCTL_ST,0);
		eMBRegHolding_Write(MB_SYS_AUTOCTL,SYS_AUTO_OFF);
	} else if ( cmd & POWER_ON ) {
		eMBRegInput_Write(MB_SYS_AUTOCTL_ST,0);
		eMBRegHolding_Write(MB_SYS_AUTOCTL,SYS_AUTO_ON);
		auto_st_end = 1;
	}
	return 0;
}

//-----------------------------------------------------------------------------------

//放气阀控制
int32_t bleed_valve_ctl(int32_t cmd)
{
	if ( cmd & POWER_ON ) {
		//放气前，机械泵与分子泵必须先停止
		if ( eMBRegInput_Read( MB_POWERPUMP_ST ) & POWER_ON )
			return -1;

		DIO_Write(RELAY_BLEED_VALVE,DO_RELAY_ON);	
		eMBRegInput_Write(MB_BLEED_VALVE_ST,POWER_ON);
	} else if ( cmd & POWER_OFF ) {
		DIO_Write(RELAY_BLEED_VALVE,DO_RELAY_OFF);	
		eMBRegInput_Write(MB_BLEED_VALVE_ST,POWER_OFF);
	}
	return 0;
}

void bleed_value_init(void)
{
	bleed_valve_ctl(POWER_OFF);	
}

//-----------------------------------------------------------------------------------
//挡板控制
int32_t baffle_ctl(int32_t cmd)
{
	if ( cmd & POWER_ON ) {
		DIO_Write(RELAY_BAFFLE,DO_RELAY_ON);	
		eMBRegInput_Write(MB_BAFFLE_ST,POWER_ON);
	}	else if ( cmd & POWER_OFF ) {
		DIO_Write(RELAY_BAFFLE,DO_RELAY_OFF);	
		eMBRegInput_Write(MB_BAFFLE_ST,POWER_OFF);
	}
	return 0;
}

void baffle_init(void)
{
	baffle_ctl(	POWER_OFF );
}

//-----------------------------------------------------------------------------------
static sTIMEOUT sample_start,sample_interval,baffle_interval;
int32_t sample_monitor_ctl( int32_t cmd );
	
int32_t sample_led_ctl( int32_t cmd )
{
	if ( cmd & SLED_PWR_ON ) {
		eMBRegInput_Write(MB_SAMPLE_LED_ST,SLED_PWR_ON);
		DIO_Write(RELAY_SAMPLE_LED,DO_RELAY_ON);	
	}	else if ( cmd & SLED_PWR_OFF ) {
		eMBRegInput_Write(MB_SAMPLE_LED_ST,SLED_PWR_OFF);
		DIO_Write(RELAY_SAMPLE_LED,DO_RELAY_OFF);	
	}
	if ( cmd & SLED1_PWR_ON ) {
		eMBRegInput_Write(MB_SAMPLE_LED_ST,SLED1_PWR_ON);
		DIO_Write(RELAY_SAMPLE_LED1,DO_RELAY_ON);	
	}	else if ( cmd & SLED1_PWR_OFF ) {
		eMBRegInput_Write(MB_SAMPLE_LED_ST,SLED1_PWR_OFF);
		DIO_Write(RELAY_SAMPLE_LED1,DO_RELAY_OFF);	
	}
	return 0;
}

//-----------------------------------------------------------------------------------
static xComPortHandle FD110A_Port = NULL;

void mpump_ctl_from_com(int32_t cmd)
{
	uint8_t buf[8];
	
	buf [0] = 0xAB;	//HEAD
	buf [1] = cmd;	//CMD
	buf [2] = 0x00;	//LEN
	buf [3] = cmd;	//xor

	vSerialPut( FD110A_Port, buf, 4);
	vTaskDelay( configTICK_RATE_HZ/10 );
	//vSerialPut( FD110A_Port, buf, 4);
}

int32_t mpump_task()
{
	static uint8_t buf[64];
	static int32_t rx_len=0;
	int32_t i;
	//uint8_t xor;

	if ( xSerialIsArrive(FD110A_Port) == pdFALSE )
		return 0;
		
	if ( rx_len + 11 > sizeof(buf) )
		rx_len = 0;
		
	i = xSerialGet(FD110A_Port,buf+rx_len,sizeof(buf)-rx_len,configTICK_RATE_HZ/100);
	rx_len += i;
	
	for ( i=0; rx_len-i>=11; i++ ){
		if ( buf[i] == 0xAB && buf[i+1] == 0x91 && buf[i+2] == 0x07 ){
			//for(xor=0,j=1;j<buf[i+2]+2;j++)
			//	xor ^= buf[i+j];
			//if ( xor != buf[10] ){
			//	rx_len = 0;
			//	return 0;
			//}
			
			eMBRegInput_Write(MB_MPUMP_ST	,buf[i+3]);//添加低8位状态
			eMBRegInput_Write(MB_MPUMP_FREQ	,buf[i+4]*100 + buf[i+5]);
			eMBRegInput_Write(MB_MPUMP_VOL	,buf[i+6]*100 + buf[i+7]);
			eMBRegInput_Write(MB_MPUMP_CUR	,buf[i+8]*100 + buf[i+9]);
			if ( eMBRegInput_Read(MB_MPUMP_FREQ) > 999 )
				eMBRegInput_Write(MB_MPUMP_FREQ,999);
			rx_len = 0;
			break;
		}
	}	
	return 0;		
}

void mpump_init(void)
{
	float vmeter_set0 = 8.0e0;
	
	eMBRegInput_Write( MB_MPUMP_ST	,0);
	eMBRegInput_Write( MB_MPUMP_FREQ,0);
	eMBRegInput_Write( MB_MPUMP_CUR	,0);
	eMBRegInput_Write( MB_MPUMP_VOL	,0);
	
	eMBRegHolding_Write( MB_MPUMP_PWR_OFF_FREQ	,10);

	eMBRegHolding_Write( MB_VMETER_SET0	,(((uint8_t*)&vmeter_set0)[0]<<8)|(((uint8_t*)&vmeter_set0)[1]));
	eMBRegHolding_Write( MB_VMETER_SET1	,(((uint8_t*)&vmeter_set0)[2]<<8)|(((uint8_t*)&vmeter_set0)[3]));
  
	FD110A_Port = xSerialPortInit( serCOM2, ser4800, serNO_PARITY, serBITS_8, serSTOP_1, 256 );
	mpump_ctl(POWER_OFF | MPUMP_STOP);
}

int32_t mpump_ctl( uint16_t cmd )
{	
	uint8_t vmeter_set[4];
	uint16_t reg;
	
	if ( cmd & MPUMP_PWR_OFF ){
		//int32_t temp;
		reg = eMBRegInput_Read(MB_MPUMP_ST) & ~MPUMP_RUN;
		eMBRegInput_Write( MB_MPUMP_ST	,reg);
		mpump_ctl_from_com(FD110A_STOP);

		//如果分子泵还有转速，则不可关闭！
		if ( eMBRegInput_Read( MB_MPUMP_FREQ ) )
			return -1;
		
		eMBRegInput_Write(MB_MPUMP_ST,MPUMP_PWR_OFF);
	} else if ( cmd & MPUMP_PWR_ON ) {
		//如果机械泵还没有上电，则不可以开启分子泵
		if ( !(eMBRegInput_Read(MB_POWERPUMP_ST) & POWER_ON) ) {
			//usart_printf(DBGU,"机械泵未上电!\r\n");
			return -3;
		}
			
		//DIO_Write( MPUMP_RELAY_CH, DO_RELAY_ON );
		eMBRegInput_Write(MB_MPUMP_ST,MPUMP_PWR_ON);
	}
	
	if ( (eMBRegInput_Read(MB_MPUMP_ST) & MPUMP_PWR_ON) != MPUMP_PWR_ON )
		return -5;
	
	reg = eMBRegInput_Read(MB_MPUMP_ST);
	if ( cmd & MPUMP_LOW_SP ) {
		mpump_ctl_from_com(FD110A_LOW_SP);
		eMBRegInput_Write(MB_MPUMP_ST,(reg & ~MPUMP_HIGH_SP));
		eMBRegInput_Write(MB_MPUMP_ST,(reg | MPUMP_LOW_SP));
	} else if ( cmd & MPUMP_HIGH_SP ) {
		mpump_ctl_from_com(FD110A_HIGH_SP);
		eMBRegInput_Write(MB_MPUMP_ST, (reg & ~MPUMP_LOW_SP));
		eMBRegInput_Write(MB_MPUMP_ST, (reg | MPUMP_HIGH_SP));
	} 
	
	if ( cmd & MPUMP_STOP ) {
		//如果还存在高压，则不可以停止分子泵
		//if ( hvsl.vol_fb > 1000 || hvsr.vol_fb > 1000 )
		//	return -2;

		mpump_ctl_from_com(FD110A_STOP);
		eMBRegInput_Write(MB_MPUMP_ST,( reg & ~MPUMP_RUN));
	} else if ( cmd & MPUMP_RUN ) {
		//真空度达不到8.0e0，则不可以开启分子泵
		vmeter_set[0] = eMBRegHolding_Read(MB_VMETER_SET0)>>8;
		vmeter_set[1] = eMBRegHolding_Read(MB_VMETER_SET0);
		vmeter_set[2] = eMBRegHolding_Read(MB_VMETER_SET1)>>8;
		vmeter_set[3] = eMBRegHolding_Read(MB_VMETER_SET1);
		
		if ( vmeter > *(float*)vmeter_set ){
			return -4;
		}
		
		mpump_ctl_from_com(FD110A_START);
		eMBRegInput_Write(MB_MPUMP_ST, (reg | MPUMP_RUN));
	} 

	return 0;
}


//----------------------------------------------------------------
int32_t auto_ctl_task(void)
{
	int32_t auto_st;
	static int32_t timer = 0;
	static sTIMEOUT vmeter_to;
	uint16_t reg = eMBRegHolding_Read(MB_SYS_AUTOCTL);

	if ( reg & SYS_AUTO_ON ){
		auto_st = (eMBRegInput_Read(MB_SYS_AUTOCTL_ST) >> SYS_AUTO_ON_ST) & 0x0F;
		if ( auto_st != 0x0F && auto_st > auto_st_end ){
			auto_st = 6;
			auto_st_end = 0xff;
		}
		switch ( auto_st ){
		case 0:
			DIO_Write( RELAY_POWERPUMP, DO_RELAY_ON );
			eMBRegInput_Write(MB_POWERPUMP_ST,POWERPUMP_PWR_ON);
	
			//先开机械泵，延迟数秒后，再开启真空计
			start_timeout(&vmeter_to,eMBRegHolding_Read(VMETER_START_DELAY));
			hv_enbale(&hvsl,ENABLE);
			hv_enbale(&hvsr,ENABLE);
			auto_st ++;
			break;
		case 1:
			if ( get_timeout(&vmeter_to) == TO_TIMEOUT ) {
				vmeter_ctl(VMETER_PWR_ON);
				auto_st ++;
			}
			break;
		case 2:
			if ( mpump_ctl(MPUMP_PWR_ON) == 0 )
				auto_st++;
			break;
		case 3:
			if ( mpump_ctl(MPUMP_HIGH_SP) == 0 )
				auto_st++;
			break;
		case 4:
			if ( mpump_ctl(MPUMP_RUN) == 0 )
				auto_st++;
			break;
		case 5:
			hv_enbale(&hvsl,ENABLE);
			hv_enbale(&hvsr,ENABLE);
			auto_st++;
			break;
		case 6:
			eMBRegHolding_Write(MB_SYS_AUTOCTL,reg & ~SYS_AUTO_ON);
			auto_st = 0x0F;
			break;
		default :
			auto_st = 0;
			break;
		}		
		eMBRegInput_Write(MB_SYS_AUTOCTL_ST, (auto_st << SYS_AUTO_ON_ST) | SYS_AUTO_ON);
	} else if ( reg & SYS_AUTO_OFF ){
		auto_st = (eMBRegInput_Read(MB_SYS_AUTOCTL_ST) >> SYS_AUTO_OFF_ST) & 0x0F;
		switch( auto_st ){
		case 0:
			hvsl.vol_set = 0;
			hvsr.vol_set = 0;
			hvsl.cur_set = 0;
			hvsr.cur_set = 0;
			hvs_update_to_modbus(&hvsl);
			hvs_update_to_modbus(&hvsr);
			auto_st ++;
			timer = 0;
			break;
		case 1:
			if ( hvsl.vol_fb < 1000 && hvsr.vol_fb < 1000 || timer ++ > 5){
				PWM_DAC_SetmV( HVL_CUR_DAC_CH, 0 );//关比例阀
				PWM_DAC_SetmV( HVR_CUR_DAC_CH, 0 );//关比例阀
				DIO_Write(hvsl.power_ch,DO_POWER_OFF);	
				DIO_Write(hvsr.power_ch,DO_POWER_OFF);	
				hv_enbale(&hvsl,DISABLE);
				hv_enbale(&hvsr,DISABLE);
				hvsl.vol_ctl = 0;
				hvsr.vol_ctl = 0;
				hvsl.cur_ctl = 0;
				hvsr.cur_ctl = 0;
				hvsl.vol_fb = 0;
				hvsr.vol_fb = 0;
				hvsl.cur_fb = 0;
				hvsr.cur_fb = 0;
				hvs_update_to_modbus(&hvsl);
				hvs_update_to_modbus(&hvsr);

				auto_st ++;
			}
			break;
		case 2:
			mpump_ctl( MPUMP_PWR_OFF | MPUMP_STOP );
			auto_st ++;
			break;
		case 3:
			if ( eMBRegInput_Read(MB_MPUMP_FREQ) < eMBRegHolding_Read(MB_MPUMP_PWR_OFF_FREQ) )
				auto_st ++;
			break;
		case 4:
			//关闭机械泵之前先关闭真空计
			vmeter_ctl(VMETER_PWR_OFF);

			start_timeout(&vmeter_to,eMBRegHolding_Read(VMETER_STOP_DELAY));
			auto_st ++;
			break;
		case 5:
			if ( get_timeout(&vmeter_to) == TO_TIMEOUT ) {
				vmeter_ctl(VMETER_PWR_OFF);
				//真空计关闭一段时间后关闭机械泵
				DIO_Write( RELAY_POWERPUMP, DO_RELAY_OFF );
				eMBRegInput_Write(MB_POWERPUMP_ST,POWERPUMP_PWR_OFF);
				auto_st ++;
			}
			break;		
		case 6:
			reg &= ~SYS_AUTO_OFF;
			auto_st = 0x0F;
			break;
		default :
			auto_st = 0;
			break;
		}
		eMBRegInput_Write(MB_SYS_AUTOCTL_ST,(auto_st << SYS_AUTO_OFF_ST) | SYS_AUTO_OFF);
	}
	eMBRegHolding_Write(MB_SYS_AUTOCTL,reg);
	return 0;
}

//-----------------------------------------------------------------------------------

void hv_init(void)
{
	//memset(usRegInputBuf,0,128);
	//memset(usRegHoldingBuf,0,128);

	init_queue(&vol_queue_l,vol_buf_l,VOL_ADC_FILTER_SIZE);	
	init_queue(&vol_queue_r,vol_buf_r,VOL_ADC_FILTER_SIZE);	
	init_queue(&cur_queue_l,cur_buf_l,CUR_ADC_FILTER_SIZE);	
	init_queue(&cur_queue_r,cur_buf_r,CUR_ADC_FILTER_SIZE);	

	hvsl.vol_max 			= 15000;
	hvsl.vol_scale 			= 5;
	hvsl.vol_err_rate		= 5;
	hvsl.vol_step 			= 10;
	hvsl.vol_step_interval	= 200;
	hvsl.vol_step_timeout 	= 10000;
	hvsl.vol_level1 		= 1000;

	hvsl.cur_max 			= 3000;
	hvsl.cur_err_rate		= 40;
	hvsl.cur_scale 			= 1;
	hvsl.cur_step 			= 1;
	hvsl.cur_step_interval	= 10000;
	hvsl.cur_step_timeout	= 30000;
	hvsl.cur_ctl_start		= 1300;
	hvsl.vol_set	= 0;
	hvsl.cur_set	= 0;
	hvsl.vol_set	= 0;
	hvsl.cur_set	= 0;

	hvsr.vol_max 			= 15000;
	hvsr.vol_scale 			= 5;
	hvsr.vol_err_rate		= 5;
	hvsr.vol_step 			= 10;
	hvsr.vol_step_interval	= 200;
	hvsr.vol_step_timeout 	= 10000;
	hvsr.vol_level1 		= 1000;
	
	hvsr.cur_max 			= 3000;
	hvsr.cur_err_rate		= 40;
	hvsr.cur_scale 			= 1;
	hvsr.cur_step 			= 1;
	hvsr.cur_step_interval	= 10000;
	hvsr.cur_step_timeout	= 30000;
	hvsr.cur_ctl_start		= 1300;
	hvsr.vol_set	= 0;
	hvsr.cur_set	= 0;
	hvsr.vol_set	= 0;
	hvsr.cur_set	= 0;
	
	hvsl.id 			= HVL;
	hvsl.vol_ctl 		= 0;
	hvsl.cur_ctl 		= 0;
	hvsl.vol_set		= 0;
	hvsl.cur_set 		= 0;
	hvsl.vol_fb			= 0;
	hvsl.cur_fb 		= 0;
	hvsl.vol_adc_ch 	= HVL_VOL_ADC_CH;
	hvsl.vol_dac_ch 	= HVL_VOL_DAC_CH;
	hvsl.cur_adc_ch 	= HVL_CUR_ADC_CH;
	hvsl.cur_dac_ch 	= HVL_CUR_DAC_CH;
	hvsl.power_ch		= HVL_POWER_CH;
	hvsl.vol_queue 		= &vol_queue_l;
	hvsl.cur_queue 		= &cur_queue_l;
	hvsl.vol_set_to 	= creat_timeout(&hvl_vol_set_to);
	hvsl.vol_check_to 	= creat_timeout(&hvl_vol_check_to);
	hvsl.cur_set_to 	= creat_timeout(&hvl_cur_set_to);
	hvsl.cur_check_to 	= creat_timeout(&hvl_cur_check_to);
	hvs_update_to_modbus(&hvsl);
	hvs_update_from_modbus(&hvsl);
	
	hvsr.id 			= HVR;
	hvsr.vol_ctl 		= 0;
	hvsr.cur_ctl 		= 0;
	hvsr.vol_set		= 0;
	hvsr.cur_set 		= 0;
	hvsr.vol_fb			= 0;
	hvsr.cur_fb 		= 0;
	hvsr.vol_adc_ch 	= HVR_VOL_ADC_CH;
	hvsr.vol_dac_ch 	= HVR_VOL_DAC_CH;
	hvsr.cur_adc_ch 	= HVR_CUR_ADC_CH;
	hvsr.cur_dac_ch 	= HVR_CUR_DAC_CH;
	hvsr.power_ch		= HVR_POWER_CH;
	hvsr.vol_queue 		= &vol_queue_r;
	hvsr.cur_queue 		= &cur_queue_r;
	hvsr.vol_set_to 	= creat_timeout(&hvr_vol_set_to);
	hvsr.vol_check_to 	= creat_timeout(&hvr_vol_check_to);	
	hvsr.cur_set_to 	= creat_timeout(&hvr_cur_set_to);	
	hvsr.cur_check_to 	= creat_timeout(&hvr_cur_check_to);
	hvs_update_to_modbus(&hvsr);
	hvs_update_from_modbus(&hvsr);

	hv_enbale(&hvsl,ENABLE);
	hv_enbale(&hvsr,ENABLE);

	DIO_Write(hvsl.power_ch,DO_POWER_OFF);
	DIO_Write(hvsr.power_ch,DO_POWER_OFF);
}

void hvs_update_from_modbus(HVS* hvs)
{

	hvs->vol_max 			= eMBRegHolding_Read(MB_VOL_MAX);
	hvs->vol_scale 			= eMBRegHolding_Read(MB_VOL_SCALE);
	hvs->vol_err_rate		= eMBRegHolding_Read(MB_VOL_ERR_RATE);
	hvs->vol_step 			= eMBRegHolding_Read(MB_VOL_STEP);
	hvs->vol_step_interval	= eMBRegHolding_Read(MB_VOL_STEP_INTERVAL);
	hvs->vol_step_timeout 	= eMBRegHolding_Read(MB_VOL_STEP_TIMEOUT);
	hvs->vol_level1 		= eMBRegHolding_Read(MB_VOL_LEVEL1);
	
	hvs->cur_max 			= eMBRegHolding_Read(MB_CURRRENT_MAX);
	hvs->cur_err_rate		= eMBRegHolding_Read(MB_CUR_ERR_RATE);
	hvs->cur_scale 			= eMBRegHolding_Read(MB_CUR_SCALE);
	hvs->cur_step 			= eMBRegHolding_Read(MB_CUR_STEP);
	hvs->cur_step_interval	= eMBRegHolding_Read(MB_CUR_STEP_INTERVAL);
	hvs->cur_step_timeout	= eMBRegHolding_Read(MB_CUR_STEP_TIMEOUT);
	hvs->cur_ctl_start		= eMBRegHolding_Read(MB_CUR_CTL_START);

	switch( hvs->id )	{
	case HVL:
		hvs->vol_set	= eMBRegHolding_Read(MB_VOL_SET_L);
		hvs->cur_set	= eMBRegHolding_Read(MB_CUR_SET_L);
		break;
	case HVR:
		hvs->vol_set	= eMBRegHolding_Read(MB_VOL_SET_R);
		hvs->cur_set	= eMBRegHolding_Read(MB_CUR_SET_R);
		break;
	default:
		break;
	}	
}

void hvs_update_to_modbus(HVS* hvs)
{
	switch( hvs->id )	{
		case HVL:
			eMBRegHolding_Write	(MB_VOL_SET_L,hvs->vol_set);
			eMBRegHolding_Write (MB_CUR_SET_L,hvs->cur_set);
			
			eMBRegInput_Write(MB_VOL_SET_L_ST	,hvs->vol_set);
			eMBRegInput_Write(MB_CUR_SET_L_ST	,hvs->cur_set);
			eMBRegInput_Write(MB_HV_ST_L	,hvs->st);
			eMBRegInput_Write(MB_VOL_FB_L	,hvs->vol_fb);
			eMBRegInput_Write(MB_VOL_CTL_L	,hvs->vol_ctl);
			eMBRegInput_Write(MB_CUR_FB_L	,hvs->cur_fb);
			eMBRegInput_Write(MB_CUR_CTL_L	,hvs->cur_ctl);
			break;
		case HVR:
			eMBRegHolding_Write(MB_VOL_SET_R,hvs->vol_set);
			eMBRegHolding_Write(MB_CUR_SET_R,hvs->cur_set);
			
			eMBRegInput_Write(MB_VOL_SET_R_ST	,hvs->vol_set);
			eMBRegInput_Write(MB_CUR_SET_R_ST	,hvs->cur_set);
			eMBRegInput_Write(MB_HV_ST_R	,hvs->st);
			eMBRegInput_Write(MB_VOL_FB_R	,hvs->vol_fb);
			eMBRegInput_Write(MB_VOL_CTL_R	,hvs->vol_ctl);
			eMBRegInput_Write(MB_CUR_FB_R	,hvs->cur_fb);
			eMBRegInput_Write(MB_CUR_CTL_R	,hvs->cur_ctl);
			break;
		default:
			break;
	}	
}

void hv_enbale(HVS* hvs,int32_t st)
{
	if ( st == DISABLE )
		hvs->st &= ~HV_ENABLE;
	else if ( st == ENABLE )
		hvs->st |= HV_ENABLE;	
}

void hv_update_vol(HVS* hvs)
{ 
	uint16_t adc[32];
	uint32_t temp;
	uint32_t i,sum,valid;
	
	if ( hvs->id == HVL ){
	} else if ( hvs->id == HVR ){
	} else 
		return ;

	if ( (hvs->st & HV_PWR) == 0 ){
		hvs->vol_fb = 0;
		return ;
	}
	
	//update voltage	
	ADC_Get(hvs->vol_adc_ch,adc,32);	
	exchange_sort16(adc,32);
	temp = get_average16(adc+8,32-2*8);
//	enqueue(hvs->vol_queue,temp);
//	exchange_sort16(hvs->vol_queue->queue,hvs->vol_queue->size);//sort the adc data
//	for(i=1,sum=0,valid=0;i<hvs->vol_queue->size-1*2;i++){
//  	//temp = (uint32_t)hvs->vol_queue->queue[i]*10 / hvs->vol_queue->queue[hvs->vol_queue->size/2];
//  	//	if ( temp > 5 && temp < 15 )
//		{
//			sum += hvs->vol_queue->queue[i];
//			valid++;
//		}
//	}
//	temp = sum/valid;
	
	hvs->vol_fb = temp*2500*2/4095 * hvs->vol_scale;
	if ( hvs->vol_fb < 500)
		hvs->vol_fb = 0;
}

void hv_update_cur(HVS* hvs)
{ 
	uint16_t adc[32];
	uint32_t temp;
	uint32_t i,sum,valid;

	if ( hvs->id == HVR ){
	} else if ( hvs->id == HVL ){
	} else 
		return ;
	
	if ( (hvs->st & HV_PWR) == 0 ){
		hvs->cur_fb = 0;
		return ;
	}
	
	ADC_Get(hvs->cur_adc_ch,adc,32);		
	exchange_sort16(adc,32);
	temp = get_average16(adc+0,32-2*0);
	enqueue(hvs->cur_queue,temp);
	exchange_sort16(hvs->cur_queue->queue,hvs->cur_queue->size);//sort the adc data
	for(i=1,sum=0,valid=0;i<hvs->cur_queue->size-1*2;i++){
  		//temp = (uint32_t)hvs->cur_queue->queue[i]*10 / hvs->cur_queue->queue[hvs->cur_queue->size/2];
  		//if ( temp > 5 && temp < 20 )
		{
			sum += hvs->cur_queue->queue[i];
			valid++;
		}
	}
	temp = sum/valid;
	
	hvs->cur_fb = temp*2500*2/4095 * hvs->cur_scale;
	if ( hvs->cur_fb < 150 )
		hvs->cur_fb = 0;
}
	
void hv_vol_update(HVS* hvs)
{
	hvs->st &= ~HV_PWR;
	start_timeout(hvs->vol_check_to, hvs->vol_step_interval);
}

int32_t hv_vol_task(HVS* hvs)
{
	int32_t to_status;
	uint16_t temp;
	uint16_t step;
	
	if ( (hvs->st & HV_ENABLE) == 0)
		return 0;

	hv_update_vol(hvs);		//更新电压电流状态寄存器
	if ( (to_status = get_timeout(hvs->vol_check_to)) == TO_TIMEOUT ) {//定时完成
		start_timeout(hvs->vol_check_to, hvs->vol_step_interval);
		//--------------------------------------------------------------------------
		hvs_update_from_modbus(hvs);
		//--------------------------------------------------------------------------
		
		step = hvs->vol_step;
		//高压输出在误差范围内
		if ( hvs->vol_set == 0 /*&& hvs->vol_fb < 1000 */) {
			hvs->st &= ~(HV_SET_TO | HV_INCTRL);	//清除故障状态标志
			hvs->st &= ~(HV_SET_OK | HV_PWR);
			PWM_DAC_SetmV( hvs->vol_dac_ch, (hvs->vol_ctl=0) );
			DIO_Write(hvs->power_ch,DO_POWER_OFF);	
		} else if ( abs(hvs->vol_set - hvs->vol_fb) < hvs->vol_set * hvs->vol_err_rate / 1000 ) {
			hvs->st &= ~(HV_SET_TO | HV_INCTRL);	//清除故障状态标志
			hvs->st |=  HV_SET_OK;
		} else {//电压需要调整
			//开启高压电源
			DIO_Write(hvs->power_ch,DO_POWER_ON);
			hvs->st |= HV_PWR;

			//if ( hvs->st & HV_SET_OK ){
				//start_timeout(hvs->vol_check_to, hvs->vol_step_interval*10);
			//} else 
			{
			step = hvs->vol_step;	
							
				if ( labs( hvs->vol_fb - hvs->vol_set) > 1000 )
					step *= 50;
				else if ( labs( hvs->vol_fb - hvs->vol_set) > 100 )
					step *= 5;		
			}
			
			if ( hvs->vol_fb > hvs->vol_set ) {
				if ( hvs->vol_ctl > step )
					hvs->vol_ctl -= step;
				else 
					hvs->vol_ctl = 0;
			}	else {
				if ( (hvs->vol_set - hvs->vol_fb > 500) && (hvs->cur_fb > 100) )	//放电
				{	//hvs->cur_ctl = hvs->cur_ctl*9/10;
				}	else 
					hvs->vol_ctl += step;
			}
			
			if ( hvs->vol_ctl - hvs->vol_set > 3000 )	//误差调整保护
				hvs->vol_ctl = hvs->vol_set+3000;
			if ( hvs->vol_ctl > 3000 && hvs->vol_fb < 1000 )	//电源不受控保护
				hvs->vol_ctl = 3000;
				PWM_DAC_SetmV( hvs->vol_dac_ch, hvs->vol_ctl / hvs->vol_scale );
		}	//if ( abs(
		hvs_update_to_modbus(hvs);
	} else if ( to_status != TO_RUNING ) {	//冗余处理
		start_timeout(hvs->vol_check_to, hvs->vol_step_interval);
	}	//if ( (to_status 

  return 0;
}

int32_t hv_cur_task(HVS* hvs)
{
	uint16_t step,interval;
	uint32_t temp;
	int32_t to_status;
	uint32_t err;

	if ( (hvs->st & HV_ENABLE) == 0)
		return 0;
		
	hv_update_cur(hvs);
  	if ( (to_status = get_timeout(hvs->cur_check_to)) == TO_TIMEOUT ){
		start_timeout(hvs->cur_check_to, hvs->cur_step_interval);

		step = hvs->cur_step;
		interval = hvs->cur_step_interval;		

		if ( hvs->cur_set == 0 ) {
			PWM_DAC_SetmV( hvs->cur_dac_ch, (hvs->cur_ctl=0) );
			//hvs_update_to_modbus(hvs);
			return 0;
		} if ( (hvs->vol_set - hvs->vol_fb > 500) && (hvs->cur_fb > 100) ) {	//放电
			if ( hvs->cur_ctl > hvs->cur_step*50 ){
				hvs->cur_ctl -= hvs->cur_step*50;
			} else 
				hvs->cur_ctl = 0;
			PWM_DAC_SetmV( hvs->cur_dac_ch, hvs->cur_ctl );
			return 0;
		}
					
		if ( (temp = abs ( hvs->cur_set - hvs->cur_fb )) < (hvs->cur_set * hvs->cur_err_rate / 1000) ) {
			hvs->st |= HV_CUR_SET_OK;
		} else {
			hvs->st &= ~HV_CUR_SET_OK;

			err = temp*100 / hvs->cur_set;//%
			if ( hvs->cur_fb < 150 ) {
				step 		= hvs->cur_step*50;
				interval = hvs->cur_step_interval/2;		
			} else {
				interval = hvs->cur_step_interval*1;		
				if ( temp > 1000 )
					step 	 = hvs->cur_step * 20;
				else if ( temp > 500 )		
					step 	 = hvs->cur_step * 10;
				else if ( temp > 100 )		
					step 	 = hvs->cur_step * 2;
				else	
					step 	 = hvs->cur_step * 1;
			}
			//if ( hvs->cur_fb > hvs->cur_set || (hvs->st & HV_SET_OK) == 0) {
			if ( hvs->cur_fb > hvs->cur_set ) {
				if ( hvs->cur_ctl > step ){
					hvs->cur_ctl -= step;
				} else 
					hvs->cur_ctl = 0;
			}	else {
				hvs->cur_ctl += step;//mV
				if ( hvs->cur_ctl > CUR_DAC_FULL )
					hvs->cur_ctl = CUR_DAC_FULL;
			}
			PWM_DAC_SetmV( hvs->cur_dac_ch, hvs->cur_ctl );
		    start_timeout(hvs->cur_check_to, interval);
		}	//if ( abs ...
		
		hvs_update_to_modbus(hvs);
	} else if ( to_status != TO_RUNING ) {	//冗余处理
		start_timeout(hvs->cur_check_to, hvs->cur_step_interval);
	}
	
	return 0;
}			

void update_adc_modbus()
{
	uint16_t buf16[32];
	uint16_t i;
	
	for(i=0;i<8;i++){
		ADC_Get(ADC_Channel_8+i,buf16,32);	
		exchange_sort16(buf16,32);
		vPortEnterCritical();
		eMBRegInput_Write(MB_ADC0+i,get_average16(buf16+6,32-2*6));
		vPortExitCritical();
	}
}

int32_t gl_696h_init()
{
	//relay_init();
	//led_init();
	DIO_Init();
	powerpump_init();
	bleed_value_init();
	mpump_init();
	vmeter_init();
	baffle_init();
	hv_init();

	return 0;
}


void vGL696H_Task( void *pvParameters )
{
	uint32_t i=0,j,motor = 0;
	uint32_t sec=0;
	uint32_t ret;
	portTickType xLastWakeTime;

	(void)pvParameters;

	gl_696h_init(); 
	xLastWakeTime = xTaskGetTickCount ();
	motor = MOTOR_FORWARD;
	
	while(1){
		system_tick += 10;
		
	//	led_task();
	  
		//---------------------------------------------------------------------------------
	    hv_vol_task(&hvsl);
	    hv_cur_task(&hvsl);
	    
	    hv_vol_task(&hvsr);
	    hv_cur_task(&hvsr);

	    vmeter_task();
		update_adc_modbus();

	    //---------------------------------------------------------------------------------
	    if ( (ret = get_timeout(&sec_to)) == TO_TIMEOUT) {
			start_timeout(&sec_to,1000);
			sec ++;
	 
			//usart_printf(DBGU, " adc : ");
	    	/*for(j=0;j<8;j++){
		    	ADC_Get(8+j,buf,32);
		    	//usart_printf(DBGU, " %4dmV ,",adc_get_mv(buf[0]));
		    }
		    //usart_printf(DBGU, "\r\n");
	   		
			i = 1000;
			PWM_DAC_SetmV(0,i);i += 100;
			PWM_DAC_SetmV(1,i);i += 100;
			PWM_DAC_SetmV(2,i);i += 100;
			PWM_DAC_SetmV(3,i);i += 100;   
			PWM_DAC_SetmV(4,i);i += 100;
			PWM_DAC_SetmV(5,i);i += 100;
			PWM_DAC_SetmV(6,i);i += 100;
			PWM_DAC_SetmV(7,i);i += 100;   
	  	    */
	      	//----------自动控制---------------------------------------------------------------
	    	auto_ctl_task();
	      
	      	mpump_task();
	      	if ( (sec % 2) == 0 ) {
				mpump_ctl_from_com(FD110A_STATUS);
	    	} 
			
			if ( (sec % 60 ) == 0 ){
				i = eMBRegInput_Read(MB_MOTOR_ST);
				i =  motor;
				/*if ( (i & MOTOR_ENABLE) )*/ {
					if ( i & MOTOR_FORWARD ){
						motor = MOTOR_BACKWARD;
						DIO_Write(RELAY0,DO_RELAY_ON);
						DIO_Write(RELAY1,DO_RELAY_ON);
					} else {
						motor = MOTOR_FORWARD;
						DIO_Write(RELAY0,DO_RELAY_OFF);
						DIO_Write(RELAY1,DO_RELAY_OFF);
					}
				}
			}
	    } else if ( ret != TO_RUNING ){
			start_timeout(&sec_to,1000);
		}

		//-----------------------------------------------------------------------------------
		vTaskDelayUntil( &xLastWakeTime, configTICK_RATE_HZ/100 );
	}//	while(1){

}


void vGL696H_Test_Task( void *pvParameters )
{
	uint32_t i=0,j,len = 0;
	uint32_t sec=0;
	uint32_t ret;
	portTickType xLastWakeTime;
	uint32_t vol=0;
	uint8_t buf[256];
	xComPortHandle Port1 = NULL;

	(void)pvParameters;

	xLastWakeTime = xTaskGetTickCount ();

	Port1		= xSerialPortInit( serCOM1, ser115200, serNO_PARITY, serBITS_8, serSTOP_1, 256 );
	FD110A_Port = xSerialPortInit( serCOM2, ser115200, serNO_PARITY, serBITS_8, serSTOP_1, 256 );
	vmeter_Port = xSerialPortInit( serCOM3, ser115200, serNO_PARITY, serBITS_8, serSTOP_1, 256 );
	
	while(1){
#if 1
		system_tick += 10;
		
		//usart_printf(DBGU, " adc : ");
    	/*for(j=0;j<8;j++){
	    	ADC_Get(8+j,buf,32);
	    	//usart_printf(DBGU, " %4dmV ,",adc_get_mv(buf[0]));
	    }
	    //usart_printf(DBGU, "\r\n");
   		*/
		for(i=0;i<8;i++)
			PWM_DAC_SetmV(i,vol);
		if ( vol < 5000 )	vol += 5;   else vol = 0;
		vol = 2500;
  	    
		if ( (len = xSerialGet(Port1,buf,sizeof(buf),configTICK_RATE_HZ/1000)) ){
			vSerialPut( Port1, buf, len);
		}
		if ( (len = xSerialGet(FD110A_Port,buf,sizeof(buf),configTICK_RATE_HZ/1000)) ){
			vSerialPut( FD110A_Port, buf, len);
		}
		if ( (len = xSerialGet(vmeter_Port,buf,sizeof(buf),configTICK_RATE_HZ/1000)) ){
			vSerialPut( vmeter_Port, buf, len);
		}
#endif
		//-----------------------------------------------------------------------------------
		vTaskDelayUntil( &xLastWakeTime, configTICK_RATE_HZ/100 );
	}//	while(1){

}
