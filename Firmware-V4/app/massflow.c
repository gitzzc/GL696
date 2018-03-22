#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x.h"

#include "gl_696h.h"
#include "adc.h"
#include "gpio.h"
#include "pwm_dac.h"
#include "serials.h"
#include "mb_reg_map.h"
#include "modbus.h"

//----------------------------------------------------------------
#define ON	1
#define OFF	0

#define VOL_DAC_FULL	5000
#define CUR_DAC_FULL	5000

//----------------------------------------------------------------
#define HV_POWER_PORT	GPIOA
#define HV_POWER_CH0	GPIO_Pin_4
#define HV_POWER_CH1	GPIO_Pin_5
#define HV_POWER_CH2	GPIO_Pin_6
#define HV_POWER_CH3	GPIO_Pin_7

uint32_t hv_channel[4] = { GPIO_Pin_4,GPIO_Pin_5,GPIO_Pin_6,GPIO_Pin_7};
//----------------------------------------------------------------
uint32_t sys_tick=0;


//---------------------------------------------------------------------
int32_t hv_pwr_ctl(uint32_t ch,int32_t cmd)
{
	if ( cmd & POWER_ON ) {
		GPIO_WriteBit(HV_POWER_PORT, hv_channel[ch], Bit_RESET);
	}	else if ( cmd & POWER_OFF ) {
		GPIO_WriteBit(HV_POWER_PORT, hv_channel[ch], Bit_SET);
	}
	return 0;
}	

//-----------------------------------------------------------------------------------
static xComPortHandle Temp_Port = NULL;
#define MB_TEMP_CTL_ADDR	0x11

void temp_init(void)
{  
	Temp_Port = xSerialPortInit( serCOM2, ser19200, serNO_PARITY, serBITS_8, serSTOP_1, 256 );
}


void temp_read_write_reg(void)
{
	uint8_t buf[32];
	uint16_t usCRC16;
	uint16_t len;
	static uint32_t read = 0;

	vPortEnterCritical();
	if ( read  ){
		read = 0;
		len = 0;
		buf [len++] = MB_TEMP_CTL_ADDR;
		buf [len++] = 0x03;
		buf [len++] = 0x00;
		buf [len++] = 0x00;
		buf [len++] = 0x00;
		buf [len++] = 0x04;
	} else {
		read = 1;
		len = 0;
		buf [len++] = MB_TEMP_CTL_ADDR;
		buf [len++] = 0x10;
		buf [len++] = 0x00;
		buf [len++] = 0x38;
		buf [len++] = 0x00;
		buf [len++] = 0x04;
		buf [len++] = usRegHoldingBuf[MB_TEMP_SET00];
		buf [len++] = usRegHoldingBuf[MB_TEMP_SET00]>>8;
		buf [len++] = usRegHoldingBuf[MB_TEMP_SET01];
		buf [len++] = usRegHoldingBuf[MB_TEMP_SET01]>>8;
		buf [len++] = usRegHoldingBuf[MB_TEMP_SET10];
		buf [len++] = usRegHoldingBuf[MB_TEMP_SET10]>>8;
		buf [len++] = usRegHoldingBuf[MB_TEMP_SET11];
		buf [len++] = usRegHoldingBuf[MB_TEMP_SET11]>>8;
	}		
	vPortExitCritical();
    /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
    usCRC16 = usMBCRC16( ( UCHAR * ) buf, len );
    buf [len++] = ( usCRC16 & 0xFF );
    buf [len++] = ( usCRC16 >> 8 );
	vSerialPut( Temp_Port, buf, len);
	vTaskDelay( configTICK_RATE_HZ/10 );
}

int32_t temp_task()
{
	static uint8_t buf[64];
	static int32_t rx_len=0;
	static uint32_t time = 0;
	int32_t i;

	if ( xSerialIsArrive(Temp_Port) == pdTRUE ) {
			
		if ( rx_len + 7 > sizeof(buf) )
			rx_len = 0;
			
		i = xSerialGet(Temp_Port,buf+rx_len,sizeof(buf)-rx_len,configTICK_RATE_HZ/10);
		rx_len += i;
		
		for ( i=0; rx_len-i>=7; i++ ){
			if ( buf[i] == MB_TEMP_CTL_ADDR && buf[i+1] == 0x03 && buf[i+2] >= 8 ){
				rx_len = 0;
				vPortEnterCritical();
				usRegInputBuf[MB_TEMP00] = ((buf[i+3] << 8) | buf[i+4]);
				usRegInputBuf[MB_TEMP01] = ((buf[i+5] << 8) | buf[i+6]);
				usRegInputBuf[MB_TEMP10] = ((buf[i+7] << 8) | buf[i+8]);
				usRegInputBuf[MB_TEMP11] = ((buf[i+9] << 8) | buf[i+10]);
				vPortExitCritical();
				break;
			}
		}	
	} else if ( (time++ % 5) == 0 ){
		temp_read_write_reg();
	}
	return 0;	
}

//-----------------------------------------------------------------------------------
void hv_init(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	uint32_t i;

	/* Enable the Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= HV_POWER_CH0 | HV_POWER_CH1 | HV_POWER_CH2 | HV_POWER_CH3;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;
	GPIO_Init(HV_POWER_PORT, &GPIO_InitStructure);
	
	for(i=0;i<4;i++)
		hv_pwr_ctl(i,POWER_OFF);
}

uint16_t adc_scale[8] = 
{
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000
};

void update_adc_modbus()
{
	uint16_t buf16[32];
	uint16_t i;
	
	for(i=0;i<8;i++){
		ADC_Get(ADC_Channel_8+i,buf16,32);	
		exchange_sort16(buf16,32);
		vPortEnterCritical();
		usRegInputBuf[MB_ADC0+i] = get_average16(buf16+6,32-2*6)*1000/adc_scale[i];
		vPortExitCritical();
	}
}

void vMassFlow_Task( void *pvParameters )
{
	uint32_t i=0,j,motor = 0;
	uint32_t sec=0;
//	uint16_t buf[128];
	portTickType xLastWakeTime;

	(void)pvParameters;

	hv_init();
	temp_init();
	PWM_DAC_SetmV(0,1000);
	
	xLastWakeTime = xTaskGetTickCount ();

	while(1){
		sys_tick += 10;
		
		//---------------------------------------------------------------------------------
		update_adc_modbus();
      	temp_task();

		//-----------------------------------------------------------------------------------
		vTaskDelayUntil( &xLastWakeTime, configTICK_RATE_HZ/100 );
	}//	while(1){
}
