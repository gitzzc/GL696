/*
 *	���뻷����avr-gcc,avr-lib
 *	�ļ����ƣ�modbus.h
 *	ժ    Ҫ��modbus Э���������
 *           	���RTU��ASCII����֡��ʽ�Ľ��룬
 *
 */

#ifndef __MODBUS_H__
#define __MODBUS_H__

//-----------------------------------------------------------------------
#define MB_MAX_INPUT_REG 	9

#define MB_BUF_SIZE 		280

#define MB_MAX_PACKAGE_INTERVAL	60*100	//120*100ms=120 SEC

#define MB_MIN_LEN			0x04

#define MB_START 			':'
#define MB_END0  			'\r'
#define MB_END1  			'\n'

#define MB_ADU_ADDR    		0X00
#define MB_ADU_FN    		0X01

#define MB_BROADCAST_ADDR 	0

#define MB_DEV_ADDR 		0x01
#define MB_DEV_ADDR_TEST	255
#define MB_CONFIG_BASE_ADDR	0x0000
#define MB_STATUS_BASE_ADDR 0x0100

#define MB_FRAME_OK   		0xA0
#define MB_ADDR_MATCH 		0xA1
#define MB_LRC_ERR    		0xA2
#define MB_CRC_ERR			0xA3
#define MB_TIMEOUT			0xA4

#define RESET_KEY			0x5253	//RS
#define ROOT_KEY			0x5254	//RT
#define CONFIG_KEY			0x4346	//CF
#define WRITE_KEY			0x5752	//WR

//exception codes 
#define ILLEGAL_FUNCTION		0x01
#define ILLEGAL_DATA_ADDRESS	0x02
#define ILLEGAL_DATA_VALUE		0x03
#define SLAVE_DEVICE_FAILURE	0x04
#define ACKNOWLDEGE				0x05
#define SLAVE_DEVICE_BUSY		0x06
#define NEGATIVE_ACKNOWLEDGE	0x07
#define MEMORY_RARITY_ERROR		0x08

#define MB_FRAME_RTU			0x01
#define MB_FRAME_ASICC			0x02
#define MB_FRAME_TCP			0x03

#define MB_SERIAL_PORT			0

typedef struct __MBFRAME__
{
	uint8_t		ucAddress;
	uint8_t 	ucType;
	uint16_t 	usTransId;
	uint16_t 	usProtlId;
	uint16_t 	usLength;
	uint8_t 	ucUnitId;
	uint8_t*  	pcAdu;
	uint16_t 	usTxLength;
	uint16_t 	usRxLength;
	uint16_t 	usRxBufSize;
	uint16_t 	usTxBufSize;
	uint8_t*  	pcRxBuf;
	uint8_t*  	pcTxBuf;
	uint32_t	ulCrc_err_count;
	uint32_t	ulPackage_err_count;
	uint32_t	ulPackage_count;
	uint32_t	ulEeventCount;
}sMBS_CTL,*psMBS_CTL;

extern sMBS_CTL mbs;

//-----------------------------------------------------------------------
int8_t MB_Init(psMBS_CTL psmbs,uint8_t ucSlaveAddress);
void MB_SerialSend(uint8_t port,psMBS_CTL psmbs);
uint8_t* MB_CheckFrame(uint8_t* frame,uint16_t len,uint8_t* type);
uint8_t* MB_GetFrame(void);
uint16_t MBS_Process(psMBS_CTL psmbs);
uint8_t  MB_SerialTask(UART_HandleTypeDef *huart);

uint8_t MBS_ExceptionResponse(psMBS_CTL psmbs,uint8_t excep);
uint8_t MBS_ReadHoldingRegister(psMBS_CTL psmbs);
uint8_t MBS_PresetMultipleRegister(psMBS_CTL psmbs);

#endif
