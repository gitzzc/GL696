/*
 *	文件名称：modbus.c
 *	摘    要：modbus 协议分析程序
 *           完成RTU和ASCII两种帧格式的解码，
 *          
 *
 */

#include <string.h>
#include <math.h>

#include "projdefs.h"
#include "config.h"
#include "serial.h"
#include "modbus.h"
#include "timer.h"
#include "adc.h"
#include "gpio.h"

uint8_t MB_FrameBuf[MB_BUF_SIZE];		//MODBUS帧接收缓冲区


//----------------------------------------------------------
uint8_t  mb_addr;						//MODBUS Slave device address
uint16_t mb_event_count;				//MODBUS通信事件计数器
uint16_t mb_crc_err_count=0;
uint16_t mb_package_err_count=0;
uint32_t mb_package_count=0;
uint16_t mb_frame_type=0;
uint16_t frame_index;
uint16_t protocol_id;


//----------------------------------------------------------

//----------------------------------------------------------
//----------------------------------------------------------
/* Table of CRC values for high order byte */
const uint8_t auchCRCHi[] = 
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;

/* Table of CRC values for low order byte */
const uint8_t auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
} ;

uint16_t MB_CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
	uint8_t uchCRCHi = 0xFF ; 	    /* high byte of CRC initialized */
	uint8_t uchCRCLo = 0xFF ; 	    /* low byte of CRC initialized */
	uint8_t uIndex ; 				/* will index into CRC lookup table */

	while( usDataLen-- ) {
		uIndex   = uchCRCLo ^ *( puchMsg++ );
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return ((uint16_t)uchCRCLo << 8 | uchCRCHi);
}

/* function:        modbus_init
 * argument:        none
 * return value:    none
 * description:     完成modbus协议处理程序相关的初始化工作
 */
int8_t MB_Init(uint8_t ucSlaveAddress, uint32_t ulBaudRate)
{
	mb_addr 			= MB_DEV_ADDR;
	mb_event_count= 0;
	mb_package_err_count = 0;
	
  return 0;
}

void MB_SerialSend(uint8_t port,uint8_t* buf,uint16_t tx_len,uint8_t type)
{
	uint16_t i;
	uint16_t crc16=0;

	if ( type == MB_FRAME_TCP ){ 	//MB_FRAME_TCP
		for(i=0;i<tx_len + 6;i++)
			xSerialPutChar(port,buf[i],0);
	} else 	if ( type == MB_FRAME_RTU ){
		crc16 = MB_CRC16(buf,tx_len);
		for(i=0;i<tx_len;i++)
			xSerialPutChar(port,buf[i],0);
		xSerialPutChar(port,crc16>>8,0);	//MSB first
		xSerialPutChar(port,crc16,0);			//then LSB
	}
}

uint8_t* MB_CheckFrame(uint8_t* frame,uint16_t len,uint8_t* type)
{
	uint8_t *pcAdu = NULL;
	uint16_t i;
	
	if ( frame == NULL )	return NULL;

	for(i=0;i<len;i++){
		if ( len >= 12 && frame[2] == 0x00 && frame[3] == 0x00 
			 && (((uint16_t)frame[4]<<8)|frame[5])+6 == len ){
			pcAdu = frame + 6;
			*type = MB_FRAME_TCP;
			return pcAdu;
		} else if ( len >= 6 && MB_CRC16(frame,len) == 0 ){
			pcAdu = frame;
			*type = MB_FRAME_RTU;
			return pcAdu;
		}
		frame++; 
		len --;
	}
	return NULL;
}

uint8_t* MB_SerialTask(uint8_t port)
{
#define FRAME_TIME	HZ_TICK/50

	static sTIMEOUT sFTime;
	static uint32_t index=0;
	int8_t ch;
	uint8_t* pcAdu;
	uint16_t tx_len;
	uint8_t type;
	
	if ( index >= MB_BUF_SIZE )	index = 0;
	
	if ( get_timeout(&sFTime) == TO_TIMEOUT || index >= MB_BUF_SIZE ){
		index = 0;
		pcAdu = MB_CheckFrame(MB_FrameBuf,index,&type); 
		if ( pcAdu ){
			tx_len = MB_Process(pcAdu);
			if ( tx_len )
				MB_SerialSend(MB_SERIAL_PORT,MB_FrameBuf,tx_len,type);
		}
	} else if ( xSerialGetChar( MB_SERIAL_PORT, &ch, 0 ) == pdTRUE ){
		start_timeout(&sFTime,FRAME_TIME);
		MB_FrameBuf[index++] = ch;
	}
	return NULL;
}

uint16_t MB_Process(uint8_t*  pcAdu)
{
	uint8_t ret=0;
	uint16_t tx_len=0;

	switch( pcAdu[MB_ADU_FN] ){	//fn
		case 0x03:
			ret = MB_ReadHoldingRegister(pcAdu,&tx_len);
			mb_event_count++;
			break;
		case 0x10:
			ret = MB_PresetMultipleRegister(pcAdu,&tx_len);
			mb_event_count++;
			break;
		default: 
			ret = 0;
			tx_len = 0;
			break;
	}
	if ( ret ){
		MB_ExceptionResponse(pcAdu,ret,&tx_len);
	}

	return (tx_len);		
}

//--------------------------------------------------------------

/* function:        exception_response
 * argument:        none
 * return value:    int8_t 程序执行状态
 * description:     
 *					
 *					
 */

uint8_t MB_ExceptionResponse(uint8_t* pcAdu,uint8_t excep,uint16_t* tx_len)
{
	pcAdu[1] |= 0x80; //fun code
	pcAdu[2] = excep; //exception code
	*tx_len = 3;
	
	return 0;
}


/* function:        read_holding_register (fun_no:0x03)
 * argument:        none
 * return value:    int 读取寄存器个数
 * description:     读取寄存器内容
 */
uint8_t MB_ReadHoldingRegister(uint8_t* pcAdu,uint16_t* tx_len)
{
	uint16_t start_addr,read_len;
	uint16_t i,index;

	start_addr 	= (pcAdu[2]<<8) + pcAdu[3];	//holding register address
	read_len	= (pcAdu[4]<<8) + pcAdu[5];	//No. of register

//	pcAdu[0] = pcAdu[MB_ADU_ADDR];//slave address 
//	pcAdu[1] = pcAdu[MB_ADU_FN];	//fun No.
	pcAdu[2] = read_len * 2;			//byte counter

	if ( pcAdu[MB_ADU_ADDR] == MB_DEV_ADDR ){
		if ( (start_addr + read_len ) > 0x10 )
			return ILLEGAL_DATA_ADDRESS;
		for(index=3,i=0;i<read_len;i++){
			switch(i+start_addr){
			/*case 0x00:
				pcAdu[index++] = sSys_cfg.vol1a>>8;
				pcAdu[index++] = sSys_cfg.vol1a;
				break;
			case 0x03:
				pcAdu[index++] = sSys_cfg.cur1a>>8;
				pcAdu[index++] = sSys_cfg.cur1a;
				break;
			case 0x07:
				pcAdu[index++] = sSys_cfg.stat1>>8;
				pcAdu[index++] = sSys_cfg.stat1;
				break;
			case 0x08:
				pcAdu[index++] = sSys_cfg.vol2a>>8;
				pcAdu[index++] = sSys_cfg.vol2a;
				break;
			case 0x0B:
				pcAdu[index++] = sSys_cfg.cur2a>>8;
				pcAdu[index++] = sSys_cfg.cur2a;
				break;
			case 0x0F:
				pcAdu[index++] = sSys_cfg.stat2>>8;
				pcAdu[index++] = sSys_cfg.stat2;
				break;
			case 0x01:
			case 0x02:
			case 0x04:
			case 0x05:
			case 0x06:
			case 0x09:
			case 0x0A:
			case 0x0C:
			case 0x0D:
			case 0x0E:
				pcAdu[index++] = 0;
				pcAdu[index++] = 0;
				break;	*/		
			default:
				break;
			}
		}
		*tx_len = pcAdu[2] + 3;
	} else if ( pcAdu[MB_ADU_ADDR] == MB_DEV_ADDR_TEST ){
		if ( (start_addr + read_len ) > sizeof(sSYS_CFG)/2 )
			return ILLEGAL_DATA_ADDRESS;
		for(index=3,i=0;i<read_len;i++){
			pcAdu[index++] = *((uint8_t*)&sSys_cfg+2*(i+start_addr)+1);
			pcAdu[index++] = *((uint8_t*)&sSys_cfg+2*(i+start_addr));
		}
		*tx_len = pcAdu[2] + 3;
	} else
		*tx_len = 0;
			
	return 0;
}

/* function:        force_multiple_regs(fun_no:0x10)
 * argument:        none
 * return value:    
 * description:     设置寄存器内容
 *		            
 */
uint8_t MB_PresetMultipleRegister(uint8_t* pcAdu,uint16_t* tx_len)
{
	uint16_t start_addr,write_len,reg_value;
	int8_t i,	count=0;

	start_addr	= (pcAdu[2]<<8) + pcAdu[3];	//register address
	write_len	= (pcAdu[4]<<8) + pcAdu[5];	//No. of register
  
	if ( pcAdu[MB_ADU_ADDR] == MB_DEV_ADDR ){
		if ( (start_addr + write_len ) > 0x10 )
			return ILLEGAL_DATA_ADDRESS;
		for(i=0;i<write_len;i++){
			reg_value = (pcAdu[7+2*i]<<8) + pcAdu[8+2*i];
			switch (i+start_addr){
			case 0:
				//dat_set = reg_value;
				break;
			default:
				return ILLEGAL_DATA_ADDRESS;
			}
		}
	} else if ( pcAdu[MB_ADU_ADDR] == MB_DEV_ADDR_TEST ){
		if ( (start_addr + write_len ) > sizeof(sSYS_CFG)/2 && !(start_addr == 10000 &&  write_len == 8) )
			return ILLEGAL_DATA_ADDRESS;
		for(i=0;i<write_len;i++){
			reg_value = (pcAdu[7+2*i]<<8) + pcAdu[8+2*i];
			switch(i+start_addr){
			case 0: 
			case 1: 
			case 2: 
			case 3: 
			case 4: 
			case 5:
			case 6: break;
			case 7:
				if ( reg_value == ENABLE_KEY )
					sSys_cfg.key = reg_value;
				else if ( reg_value == WRITE_KEY && sSys_cfg.key == ENABLE_KEY ){
					sSys_cfg.key = 1;
					ConfigWrite(&sSys_cfg,&sSys_def_cfg);
				} else if ( reg_value == RESET_KEY && sSys_cfg.key == ENABLE_KEY ){
					memcpy(&sSys_cfg,&sSys_def_cfg,sizeof(sSYS_CFG));
					sSys_cfg.key = 2;
					ConfigWrite(&sSys_cfg,&sSys_def_cfg);
				} else
					sSys_cfg.key = 0;
				break;
			case 10000:	if ( reg_value == 0xAA55 ) count = 1; else count = 0; break;
			case 10001:	if ( reg_value == 0x55AA ) count ++ ; else count = 0; break;
			case 10002:	if ( reg_value == 0x00FF ) count ++ ; else count = 0; break;
			case 10003:	if ( reg_value == 0xFF00 ) count ++ ; else count = 0; break;
			case 10004:	if ( reg_value == 0x0123 ) count ++ ; else count = 0; break;
			case 10005:	if ( reg_value == 0x4567 ) count ++ ; else count = 0; break;
			case 10006:	if ( reg_value == 0x89AB ) count ++ ; else count = 0; break;
			case 10007:	if ( reg_value == 0xCDEF ) count ++ ; else count = 0; 
						if ( count == 8 && write_len == 8 ) NVIC_SystemReset(); break;
			default:
				if ( sSys_cfg.key == ENABLE_KEY ){
					*((uint8_t*)&sSys_cfg+2*(i+start_addr))   = reg_value;
					*((uint8_t*)&sSys_cfg+2*(i+start_addr)+1) = reg_value>>8;
				}
				break;
			}
		}
		*tx_len = 6;
	} else
		*tx_len = 0;

	return 0;
}
