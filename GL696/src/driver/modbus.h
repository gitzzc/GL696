/*
 *	编译环境：avr-gcc,avr-lib
 *	文件名称：modbus.h
 *	摘    要：modbus 协议分析程序
 *           	完成RTU和ASCII两种帧格式的解码，
 *
 */

#ifndef __MODBUS_H__
#define __MODBUS_H__

//-----------------------------------------------------------------------
#define MB_MAX_INPUT_REG 	9

#define MB_BUF_SIZE 		280

#define MB_MAX_PACKAGE_INTERVAL	60*100	//120*100ms=120 SEC

#define MB_MIN_LEN		0x04

#define MB_START 			':'
#define MB_END0  			'\r'
#define MB_END1  			'\n'

#define MB_ADU_ADDR    		0X00
#define MB_ADU_FN    			0X01

#define MB_BROADCAST_ADDR 0
#define MB_DEV_ADDR 			0x01 //0x10-0x11
#define MB_DEV_ADDR_TEST	248


#define MB_FRAME_OK   		0xA0
#define MB_ADDR_MATCH 		0xA1
#define MB_LRC_ERR    		0xA2
#define MB_CRC_ERR				0xA3
#define MB_TIMEOUT				0xA4

#define ENABLE_KEY	0xA55A
#define WRITE_KEY		0xFFAA
#define RESET_KEY		0xAAFF

//exception codes 
#define ILLEGAL_FUNCTION			0x01
#define ILLEGAL_DATA_ADDRESS	0x02
#define ILLEGAL_DATA_VALUE		0x03
#define SLAVE_DEVICE_FAILURE	0x04
#define ACKNOWLDEGE						0x05
#define SLAVE_DEVICE_BUSY			0x06
#define NEGATIVE_ACKNOWLEDGE	0x07
#define MEMORY_RARITY_ERROR		0x08

#define MB_FRAME_RTU					0x01
#define MB_FRAME_ASICC				0x02
#define MB_FRAME_TCP					0x03

#define MB_SERIAL_PORT				0

typedef struct __MBFRAME__
{
	uint8_t 	ucType;
	uint16_t 	usTransId;
	uint16_t 	usProtlId;
	uint16_t 	usLength;
	uint8_t 	ucUnitId;
	uint8_t*  pcAdu;
	uint16_t 	usTXLength;
	uint16_t 	usRXLength;
	uint16_t 	usBufSize;
	uint8_t*  pcBuf;
}sMBFRAME,*psMBFRAME;

//-----------------------------------------------------------------------



#ifdef MB_RTU_MODE
extern volatile uint8_t  mb_rtu_frame_flag;
#endif

int8_t MB_Init(uint8_t ucSlaveAddress, uint32_t ulBaudRate);
uint8_t* MB_CheckFrame(uint8_t* frame,uint16_t len,uint8_t* type);
uint8_t* MB_GetFrame(void);
uint16_t MB_Process(uint8_t*  pcAdu);
uint8_t* MB_SerialTask(uint8_t port);
uint8_t MB_ExceptionResponse(uint8_t* pcAdu,uint8_t excep,uint16_t* tx_len);
uint8_t MB_ReadHoldingRegister(uint8_t* pcAdu,uint16_t* tx_len);
uint8_t MB_PresetMultipleRegister(uint8_t* pcAdu,uint16_t* tx_len);
void MB_SerialSend(uint8_t port,uint8_t* buf,uint16_t tx_len,uint8_t type);

#endif
