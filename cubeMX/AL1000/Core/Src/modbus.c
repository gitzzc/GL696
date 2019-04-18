/*
 *	文件名称：modbus.c
 *	摘    要：modbus 协议分析程序
 *           完成RTU和ASCII两种帧格式的解码，
 *          
 *
 */

#include <string.h>
#include <math.h>

#include "main.h"
#include "config.h"
#include "serial.h"
#include "modbus.h"
#include "gl_696h.h"

//----------------------------------------------------------
uint8_t MBS_RxBuf[MB_BUF_SIZE];		//MODBUS帧接收缓冲区
uint8_t MBS_TxBuf[MB_BUF_SIZE];		//MODBUS帧接收缓冲区
sMBS_CTL mbs;
uint8_t ucMB_Address=MB_DEV_ADDR;

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
int8_t MB_Init(psMBS_CTL psmbs,uint8_t ucSlaveAddress)
{
	psmbs->ucAddress			= ucSlaveAddress;
	psmbs->ulCrc_err_count		= 0;
	psmbs->ulPackage_err_count	= 0;
	psmbs->ulPackage_count		= 0;
	psmbs->ulEeventCount		= 0;
	psmbs->pcRxBuf				= MBS_RxBuf;
	psmbs->usRxBufSize			= MB_BUF_SIZE;
	psmbs->pcTxBuf				= MBS_RxBuf;
	psmbs->usTxBufSize			= MB_BUF_SIZE;
	
	return 0;
}

void MBS_SerialSend(UART_HandleTypeDef* port,psMBS_CTL psmbs)
{
	uint16_t crc16=0;
	uint8_t* pcAdu = psmbs->pcAdu;

	if ( psmbs->ucType == MB_FRAME_TCP ){ 	//MB_FRAME_TCP
	} else if ( psmbs->ucType == MB_FRAME_RTU ){
		crc16 = MB_CRC16(pcAdu,psmbs->usTxLength);
		pcAdu[psmbs->usTxLength++] = crc16>>8;
		pcAdu[psmbs->usTxLength++] = crc16;
	}
	SerialPutBuf(port,pcAdu,psmbs->usTxLength);
}

uint8_t* MBS_CheckFrame(psMBS_CTL psmbs)
{
	uint8_t *buf;
	uint16_t len;
	uint16_t i;
	
	if ( psmbs->pcRxBuf == NULL )	return NULL;

	buf = psmbs->pcRxBuf;
	len = psmbs->usRxLength;
	for(i=0;i<=len-6;i++){
		if ( len >= 12 && buf[2] == 0x00 && buf[3] == 0x00
			 && (((uint16_t)buf[4]<<8)|buf[5])+6 == len ){
			psmbs->pcAdu 	= buf + 6;
			psmbs->ucType 	= MB_FRAME_TCP;
			psmbs->ulPackage_count ++;
			return psmbs->pcAdu;
		} else if ( MB_CRC16(buf,len) == 0 ){
			psmbs->pcAdu 	= buf;
			psmbs->ucType	= MB_FRAME_RTU;
			psmbs->ulPackage_count ++;
			return psmbs->pcAdu;
		}
		buf++;
		len --;
	}
	psmbs->ulPackage_err_count ++;
	return NULL;
}

uint8_t MB_SerialTask(UART_HandleTypeDef *huart)
{
#define FRAME_TIME	50

	static uint32_t mb_tick=0;
	uint8_t* pcAdu;
	uint8_t rx;
	uint8_t ret=0;
	
	if ( SerialGetChar(huart,&rx) == pdTRUE ){
		mb_tick = HAL_GetTick();
		mbs.pcRxBuf[mbs.usRxLength] = rx;
		if ( mbs.usRxLength ++ >= mbs.usRxBufSize  )
			mbs.usRxLength = 0;
	} else if ( GetTickElapse(mb_tick) >= FRAME_TIME ){
		if ( mbs.usRxLength > 6 ) {
			pcAdu = MBS_CheckFrame(&mbs);
			if ( pcAdu ){
				MBS_Process(&mbs);
				if ( mbs.usTxLength ) {
					MBS_SerialSend(huart,&mbs);
					ret = 1;
				}
			}
		}
		mbs.usRxLength = 0;
	}
	return ret;
}

uint16_t MBS_Process(psMBS_CTL psmbs)
{
	uint8_t ret=0;

	switch( psmbs->pcAdu[MB_ADU_FN] ){
		case 0x03:
			ret = MBS_ReadHoldingRegister(psmbs);
			psmbs->ulEeventCount++;
			break;
		case 0x10:
			ret = MBS_PresetMultipleRegister(psmbs);
			psmbs->ulEeventCount++;
			break;
		default: 
			ret = 0;
			psmbs->usTxLength = 0;
			break;
	}
	if ( ret ){
		MBS_ExceptionResponse(psmbs,ret);
	}

	return (ret);
}

//--------------------------------------------------------------

/* function:        exception_response
 * argument:        none
 * return value:    int8_t 程序执行状态
 * description:     
 *					
 *					
 */

uint8_t MBS_ExceptionResponse(psMBS_CTL psmbs,uint8_t excep)
{
	psmbs->pcAdu[1] 	|= 0x80; 	//fun code
	psmbs->pcAdu[2]  	 = excep; 	//exception code
	psmbs->usTxLength 	 = 3;
	
	return 0;
}


/* function:        read_holding_register (fun_no:0x03)
 * argument:        none
 * return value:    int 读取寄存器个数
 * description:     读取寄存器内容
 */
uint8_t MBS_ReadHoldingRegister(psMBS_CTL psmbs)
{
	uint16_t start_addr,read_len;
	uint16_t i,index;
	uint8_t* pcAdu = psmbs->pcAdu;

	start_addr 	= (pcAdu[2]<<8) + pcAdu[3];		//holding register address
	read_len	= (pcAdu[4]<<8) + pcAdu[5];		//No. of register

	if ( pcAdu[MB_ADU_ADDR] == MB_DEV_ADDR_TEST || pcAdu[MB_ADU_ADDR] == psmbs->ucAddress ){
		if ( (start_addr >= MB_CONFIG_BASE_ADDR) && (start_addr < MB_CONFIG_BASE_ADDR + sizeof(sGL_config)/2) ){
			if ( start_addr + read_len > MB_CONFIG_BASE_ADDR + sizeof(sGL_config)/2 )
				return ILLEGAL_DATA_ADDRESS;
			for(index=3,i=0;i<read_len;i++){
				pcAdu[index++] = *((uint8_t*)&sGL_config+2*(i+start_addr)+1);
				pcAdu[index++] = *((uint8_t*)&sGL_config+2*(i+start_addr));
			}
			pcAdu[2] = read_len * 2;
			psmbs->usTxLength = read_len * 2 + 3;
		} else if ( start_addr >= MB_STATUS_BASE_ADDR && start_addr < MB_STATUS_BASE_ADDR + sizeof(sGL_STATUS)/2 ) {
			if ( start_addr + read_len > MB_STATUS_BASE_ADDR + sizeof(sGL_STATUS)/2 )
				return ILLEGAL_DATA_ADDRESS;
			for(index=3,i=0;i<read_len;i++){
				pcAdu[index++] = *((uint8_t*)&sGL_status+2*(i+start_addr-MB_STATUS_BASE_ADDR)+1);
				pcAdu[index++] = *((uint8_t*)&sGL_status+2*(i+start_addr-MB_STATUS_BASE_ADDR));
			}
			pcAdu[2] = read_len * 2;
			psmbs->usTxLength = read_len * 2 + 3;
		} else if ( start_addr >= 10000 && start_addr < 10008 ) {
			if ( start_addr + read_len > 10008 )
				return ILLEGAL_DATA_ADDRESS;
		} else
			return ILLEGAL_DATA_ADDRESS;
	} else
		psmbs->usTxLength = 0;
	return 0;
}

/* function:        force_multiple_regs(fun_no:0x10)
 * argument:        none
 * return value:    
 * description:     设置寄存器内容
 *		            
 */
uint8_t MBS_PresetMultipleRegister(psMBS_CTL psmbs)
{
	uint16_t start_addr,write_len,reg_value;
	uint8_t i,count=0;
	uint8_t* pcAdu = psmbs->pcAdu;

	start_addr	= (pcAdu[2]<<8) + pcAdu[3];	//register address
	write_len	= (pcAdu[4]<<8) + pcAdu[5];	//No. of register
  
	if ( pcAdu[MB_ADU_ADDR] == MB_DEV_ADDR_TEST || pcAdu[MB_ADU_ADDR] == psmbs->ucAddress ){
		if ( (start_addr >= MB_CONFIG_BASE_ADDR) && (start_addr < MB_CONFIG_BASE_ADDR + sizeof(sGL_config)/2) ){
			if ( start_addr + write_len > MB_CONFIG_BASE_ADDR + sizeof(sGL_config)/2 )
				return ILLEGAL_DATA_ADDRESS;
		} else if ( start_addr >= MB_STATUS_BASE_ADDR && start_addr < MB_STATUS_BASE_ADDR + sizeof(sGL_STATUS)/2 ) {
			if ( start_addr + write_len > MB_STATUS_BASE_ADDR + sizeof(sGL_STATUS)/2 )
				return ILLEGAL_DATA_ADDRESS;
		} else if ( start_addr >= 10000 && start_addr < 10008 ) {
			if ( start_addr + write_len > 10008 )
				return ILLEGAL_DATA_ADDRESS;
		} else
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
			case 6:
			case 7: break;
			case 8:
				if ( reg_value == ROOT_KEY || reg_value == CONFIG_KEY )
					sGL_config.usKey = reg_value;
				else if ( reg_value == WRITE_KEY && (sGL_config.usKey == ROOT_KEY || sGL_config.usKey == CONFIG_KEY) ){
					sGL_config.usKey = 1;
					ConfigWrite((uint32_t*)&sGL_config,sizeof(sGL_config)/4,(uint32_t*)&sGL_default_config);
				} else if ( reg_value == RESET_KEY && sGL_config.usKey == CONFIG_KEY ){
					memcpy((uint8_t*)&sGL_config+32*2,(uint8_t*)&sGL_default_config+32*2,sizeof(sGL_config)-32*2);
					sGL_config.usKey = 2;
					ConfigWrite((uint32_t*)&sGL_config,sizeof(sGL_config)/4,(uint32_t*)&sGL_default_config);
				} else if ( reg_value == RESET_KEY && sGL_config.usKey == ROOT_KEY ){
					memcpy(&sGL_config,&sGL_default_config,sizeof(sGL_config));
					sGL_config.usKey = 3;
					ConfigWrite((uint32_t*)&sGL_config,sizeof(sGL_config)/4,(uint32_t*)&sGL_default_config);
				} else
					sGL_config.usKey = 0;
				break;
			case 9:
			case 10:
			case 11:
			case 12:
			case 13:
			case 14:
			case 15:
				if ( sGL_config.usKey == ROOT_KEY ){
					*((uint8_t*)&sGL_config+2*(i+start_addr))   = reg_value;
					*((uint8_t*)&sGL_config+2*(i+start_addr)+1) = reg_value>>8;
				}
				break;

			case 10000:	if ( reg_value == (((uint16_t)'E'<<8)|'n') ) count = 1; else count = 0; break;
			case 10001:	if ( reg_value == (((uint16_t)'t'<<8)|'e') ) count ++ ; else count = 0; break;
			case 10002:	if ( reg_value == (((uint16_t)'r'<<8)|' ') ) count ++ ; else count = 0; break;
			case 10003:	if ( reg_value == (((uint16_t)'b'<<8)|'o') ) count ++ ; else count = 0; break;
			case 10004:	if ( reg_value == (((uint16_t)'o'<<8)|'t') ) count ++ ; else count = 0; break;
			case 10005:	if ( reg_value == (((uint16_t)'l'<<8)|'o') ) count ++ ; else count = 0; break;
			case 10006:	if ( reg_value == (((uint16_t)'a'<<8)|'d') ) count ++ ; else count = 0; break;
			case 10007:	if ( reg_value == (((uint16_t)'e'<<8)|'r') ) count ++ ;	else count = 0;
						if ( (count == 8) && (write_len == 8) )	{NVIC_SystemReset();}
						break;

			default:
				if ( i+start_addr >= MB_STATUS_BASE_ADDR ) {
					if ( sGL_config.usKey == ROOT_KEY ){
						*((uint8_t*)&sGL_status+2*(i+start_addr-MB_STATUS_BASE_ADDR))   = reg_value;
						*((uint8_t*)&sGL_status+2*(i+start_addr-MB_STATUS_BASE_ADDR)+1) = reg_value>>8;
						MBS_PresetMultipleRegisterCallback(i+start_addr,reg_value);
					}
				} else {
					if ( sGL_config.usKey == CONFIG_KEY || sGL_config.usKey == ROOT_KEY ){
						*((uint8_t*)&sGL_config+2*(i+start_addr))   = reg_value;
						*((uint8_t*)&sGL_config+2*(i+start_addr)+1) = reg_value>>8;
						MBS_PresetMultipleRegisterCallback(i+start_addr,reg_value);
					}
				}
				break;
			}

		}
		psmbs->usTxLength = 6;
	} else
		psmbs->usTxLength = 0;

	return 0;
}
