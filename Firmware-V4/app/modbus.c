/*
 *	文件名称：modbus.c
 *	承要：modbus 协议分析程序
 *           完成RTU和ASCII两种帧格式的解码
 *	
 *
 */

#include <stdlib.h>

#include "modbus.h"
#include "adc.h"
#include "gl_696h.h"
//#include "gsm.h"

#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define MB_DBG(args,...) do {usart_printf(DBGU,args,...);} while(0);
#else
#define MB_DBG
#endif 

#if defined MB_RTU_MODE || defined MB_ASCII_MODE
#else
#error 请选择MODBUS帧格式：MB_RTU_MODE,MB_ASCII_MODE
#endif

unsigned short reg40000[256];

#define MB_RX_BUF_SIZE 		256
#define MB_RX_BUF_MASK 		(MB_RX_BUF_SIZE-1)

#define MB_TX_BUF_SIZE 		256
#define MB_TX_BUF_MASK 		(MB_RX_BUF_SIZE-1)

unsigned char mb_rx_buf[MB_RX_BUF_SIZE];		//MODBUS帧接收缓冲区
unsigned char mb_rx_len;						//MODBUS帧接收缓冲区长度计数
unsigned char mb_tx_buf[MB_TX_BUF_SIZE];		//MODBUS帧发送缓冲区
unsigned int mb_event_count;

unsigned short reg30000[256];

//-----------------------------
#ifdef MB_RTU_MODE
volatile unsigned int mb_rtu_timer_en;
unsigned int mb_rtu_timer;//帧超时长
unsigned int mb_rtu_tick;//

/* Table of CRC values for high order byte */
const char auchCRCHi[] = 
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
const char auchCRCLo[] = {
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


unsigned short mb_CRC16(char *puchMsg, unsigned short usDataLen)
{
  char uchCRCHi = 0xFF ; 	    /* high byte of CRC initialized */
  char uchCRCLo = 0xFF ; 	    /* low byte of CRC initialized */
  char uIndex ; 				    /* will index into CRC lookup table */

/*   while (usDataLen--) { 			    /\* pass through message buffer *\/ */
/*     uIndex = uchCRCHi ^ *puchMsg++ ;    /\* calculate the CRC *\/ */
/*     uchCRCHi = uchCRCLo ^ pgm_read_byte(auchCRCHi+uIndex) ; */
/*     uchCRCLo = pgm_read_byte(auchCRCLo+uIndex); */
/*   } */
/*   return (uchCRCHi << 8 | uchCRCLo); */

	while( usDataLen-- ) {
		uIndex   = uchCRCLo ^ *( puchMsg++ );
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return uchCRCHi << 8 | uchCRCLo;
}

#elif defined MB_ASCII_MODE

char LRC(char *auchMsg,unsigned short usDataLen)
{
  char uchLRC=0;
  while (usDataLen--)
    uchLRC += *auchMsg ++;
  return ((char)(-((char) uchLRC)));
}

#endif

/* function:        get_frame
 * argument:        none
 * return value:    int 0:正在接收数据
 *					            1:接收到一个正确的数据帧，
 *                      存放在mb_rx_buf䶿长度为mb_rx_len个字线description:     
 */
int get_frame(void)
{
	int ret=0;
	unsigned short itemp;
	
	if ( xSerialIsArrive(com0) ){	       	//检测桢接收状徿	
		mb_rx_len = xSerialGet(com0,mb_rx_buf,sizeof(mb_rx_buf),configTICK_RATE_HZ/100);
		if ( mb_rx_len == 0) 
			return 0;
		//if ( mb_rx_len < 3 )
			//usart_printf(DBGU,"mb_rx_len < 3!\n");
		if (mb_rx_buf[0]!= MB_DEV_ADDR && mb_rx_buf[0]!=0 )
			return 0;
	} else 
		return 0;
	
	itemp = mb_CRC16(mb_rx_buf,mb_rx_len - 2);//CRC校验
	if ( itemp == (mb_rx_buf[mb_rx_len-1] << 8 ) + mb_rx_buf[mb_rx_len-2] ){
		ret = MB_FRAME_OK;
	} else {
		ret = MB_CRC_ERR;
		ret = MB_FRAME_OK;//for debug
		
		//#ifdef DEBUG
		/*usart_printf(DBGU,"CRC ERROR!CRC16:%4x,rec CRC:%4x,rec  len:%4x\r\n", \
		itemp,(mb_rx_buf[mb_rx_len-1]<<8)+mb_rx_buf[mb_rx_len-2], mb_rx_len);
		for(char i=0;i<mb_rx_len;i++){
			usart_printf(DBGU,"%4x ",mb_rx_buf[i]);
		}
		usart_printf(DBGU,"\r\n");
		*/
		//#endif
	}
	return ret;
}


void mb_send_frame(uint8_t *buf, uint8_t len)
{
	int16_t crc16;
  
	crc16 = mb_CRC16((char*)buf,len);
	buf[len++] = crc16 & 0xFF;
	buf[len++] = (crc16 >> 8) & 0xFF;
	vSerialPut( pxPort, buf, len);

//#ifdef DEBUG
	//delayms(4);
//#endif
}


short mb_cmd_process(void)
{
	short ret;
	
	ret = get_frame();
	
	if ( ret == MB_FRAME_OK ){
		ret = 0;
		switch( mb_rx_buf[MB_FN] ){
			case 0x03:
				if ( mb_rx_buf[MB_ADDR] == MB_DEV_ADDR ){		//Broadcast is not supported
					ret = read_holding_register();
					mb_event_count ++;
				}
				break;
			case 0x04:
				if ( mb_rx_buf[MB_ADDR] == MB_DEV_ADDR ){		//Broadcast is not supported
					ret = read_input_register();
					mb_event_count ++;
				}
				break;
			case 0x10:
				ret = preset_multiple_regs();
				mb_event_count ++;
				break;
			case 0x11:
				if ( mb_rx_buf[MB_ADDR] == MB_DEV_ADDR ){		//Broadcast is not supported
					ret =  report_slave_id();
					mb_event_count++;
				}
				break;
			default:
				ret = ILLEGAL_FUNCTION;
				break;
		}
        }
	if (ret){				//返回异常响应
		exception_response(ret);
		ret = MB_FRAME_OK;
	//} else {					/*异常包处理⾿	if (ret == MB_LRC_ERR) {*/
	//	MB_DBG("LRC check error!\r\n");
	} else if (ret == MB_CRC_ERR) {
		//usart_printf(DBGU,"CRC check error!\r\n");
	}
	
	//通信中断检櫿char temp = get_timeout(p_mb_to);
	//if ( temp == TM_STOP || temp == TM_TIMEOUT )
	//	ret = MB_TIMEOUT;
	//}
	return (ret);		
}

//--------------------------------------------------------------

/* function:        force_set_relay
 * argument:        short force:强制设置继电器状态便 
 * return value:    int8_t ﶿ：正常执蛿							
 * other	：异常代瘿description:     
 *					
 */
char force_set_relay(unsigned short force)
{
  return 0;
}
//--------------------------------------------------------------


/* function:        exception_response
 * argument:        none
 * return value:    int8_t 程序执行状徿 * description:     
 *					
 *					
 */
char exception_response(char excep)
{
  mb_tx_buf[0] = MB_DEV_ADDR;       		//slave addr
  mb_tx_buf[1] = mb_rx_buf[MB_FN]|0x80; 	//fun code
  mb_tx_buf[2] = excep;             		//exception code

  mb_send_frame(mb_tx_buf,3);
	
  MB_DBG("\r\nexception code: %x\r\n",excep);
  return 0;
}


/* function:        read_coil_status(fun_no:0x01)
 * argument:        none
 * return value:    int8_t 程序执行状徿 * description:     
 *					
 *					
 */
char read_coil_status()
{
#define MAX_COIL 2

  unsigned short start_addr;
  unsigned short coil_count;

  start_addr = (mb_rx_buf[2]<<8) + mb_rx_buf[3];
  coil_count = (mb_rx_buf[4]<<8) + mb_rx_buf[5];

  if (start_addr + coil_count > MAX_COIL)
    return (ILLEGAL_DATA_ADDRESS);

  return 0;
}

/*
 *  function:        read_input_status(fun_no:0x02)
 * argument:        none
 * return value:    
 * description:     
 *					
 */
char read_input_status()
{
#define MAX_INPUT 2
  unsigned short start_addr,input_count;

  start_addr  = (mb_rx_buf[2]<<8) + mb_rx_buf[3];
  input_count = (mb_rx_buf[4]<<8) + mb_rx_buf[5];

  if (start_addr + input_count > MAX_INPUT)
    return (ILLEGAL_DATA_ADDRESS);
  
  mb_tx_buf[0] = MB_DEV_ADDR;       //slave addr
  mb_tx_buf[1] = 0x02;              //fun code
  mb_tx_buf[2] = 0x01;              //byte count
  //mb_tx_buf[3] = ac_st.input>>start_addr;

  mb_send_frame(mb_tx_buf,4);
  MB_DBG("\r\nread input status: start %x ,count %x\r\n",start_addr,input_count);
  return 0;
}


/* function:        read_holding_register (fun_no:0x03)
 * argument:        none
 * return value:    int 读取寄存器个
 * description:     读取保持寄存呿address:		
 *						
 */
char read_holding_register(void)
{
  unsigned short start_addr,write_len,reg_count;

  start_addr = (mb_rx_buf[2]<<8) + mb_rx_buf[3];	//holding register address
  write_len	= (mb_rx_buf[4]<<8) + mb_rx_buf[5];	//No. of register

  mb_tx_buf[0] = MB_DEV_ADDR;				//slave address 
  mb_tx_buf[1] = mb_rx_buf[MB_FN];	//fun No.
  mb_tx_buf[2] = write_len * 2;				//byte counter
  
  if ( start_addr + write_len > (sizeof(reg40000) / sizeof(reg40000[0])) )
  	return -1;
  
  	for(reg_count=0;reg_count<write_len;reg_count++){
    	mb_tx_buf[3+2*reg_count] = reg40000[start_addr + reg_count] >> 8;
  		mb_tx_buf[3+2*reg_count+1] = reg40000[start_addr + reg_count];
	}

  mb_send_frame(mb_tx_buf,mb_tx_buf[2] + 3);
  
//  usart_wait_tx(DBGU);
//	delayms(10);  
//  usart_printf(DBGU,"read_holding_register - addr:%x,count:%x!\r\n",start_addr,reg_count);

  return 0;
}

/* function:        read_input_register (fun_no:0x04)
 * argument:        none
 * return value:    int 读取寄存器个捿description:     读取输入寄存呿address:		
 *						
 */
char read_input_register(void)
{
  unsigned short start_addr,read_len,reg_count;

  start_addr = (mb_rx_buf[2]<<8) + mb_rx_buf[3];	//holding register address
  read_len	 = (mb_rx_buf[4]<<8) + mb_rx_buf[5];	//No. of register

  mb_tx_buf[0] = MB_DEV_ADDR;			//slave address 
  mb_tx_buf[1] = mb_rx_buf[MB_FN];	//fun No.
  mb_tx_buf[2] = read_len * 2;				//byte counter
  
  if ( start_addr + read_len > (sizeof(reg30000) / sizeof(reg30000[0])) )
  	return -1;
  
  for(reg_count=0;reg_count<read_len;reg_count++){
    	mb_tx_buf[3+2*reg_count] 	= reg30000[start_addr + reg_count] >> 8;
  		mb_tx_buf[3+2*reg_count+1]= reg30000[start_addr + reg_count];
	}

  mb_send_frame(mb_tx_buf,mb_tx_buf[2]+3);
	
//  usart_wait_tx(DBGU);
//	delayms(10);  
//  usart_printf(DBGU,"read_input_register - addr:%x,count:%x!\r\n",start_addr,reg_count);
  
  return 0;
}

/* function:        fetch_comm_event_count(fun_no:0x0B)
 * argument:        none
 * return value:    
 * description:     
 *					
 *					
 */
char fetch_comm_event_count()
{
  mb_tx_buf[0] = MB_DEV_ADDR;		//slave addr
  mb_tx_buf[1] = 0x0B;              		//fun no.
  mb_tx_buf[2] = 0x00;              		//status hi
  mb_tx_buf[3] = 0x00;              		//status lo
  mb_tx_buf[4] = mb_event_count>>8; //comm_event_count hi
  mb_tx_buf[5] = mb_event_count;		//comm_event_count lo

  mb_send_frame(mb_tx_buf,6);
  MB_DBG("\r\nfetch comm event count: %x\r\n",mb_event_count);
  return 0;
}


/* function:        force_multiple_coils(fun_no:0x0F)
 * argument:        none
 * return value:    
 * description:     
 *					
 *					
 */
char force_multiple_coils(void)
{
  unsigned short start_addr,quantity_coils,force_data;
  char byte_count;
  short ret;

  start_addr     = (mb_rx_buf[2]<<8) + mb_rx_buf[3];	//starting register
  quantity_coils = (mb_rx_buf[4]<<8) + mb_rx_buf[5];	//quantity of coils
  byte_count     =  mb_rx_buf[6];								//byte count
  force_data     = ((mb_rx_buf[7])<<8) + mb_rx_buf[8];	//force data

  return ILLEGAL_DATA_ADDRESS;
}


void mb_bit_fun(unsigned short reg,unsigned int st, void(*fun)(int,int) )
{
	int i;	
	for(i=0;i<16;i++)
		if ( (reg >> i) & 0x01 )
			fun(i,st);
}

/* function:        preset_multiple_regs(fun_no:0x10)
 * argument:        none
 * return value:    
 * description:
*/
char preset_multiple_regs(void)
{
  unsigned short start_addr,write_len,reg_addr;
  unsigned char  byte_len;
  unsigned char* reg_buf;
  
	unsigned char reg_count;
  unsigned short reg_value;
  char ret;

	start_addr  = (mb_rx_buf[2]<<8) + mb_rx_buf[3];	//register address
	write_len 	= (mb_rx_buf[4]<<8) + mb_rx_buf[5];	//No. of register
	byte_len	= mb_rx_buf[6];					//byte count
	reg_buf 	= mb_rx_buf + 7;

//#ifdef DEBUG
//  usart_printf(DBGU,"\r\npreset_multiple_regs:%x,%x,%x - ",start_addr,write_len,byte_len);
//  for(char i= 0;i<write_len;i++){
//		usart_printf(DBGU,"%4x ",(mb_rx_buf[7+2*i]<<8) + mb_rx_buf[8+2*i]);
//  }
//  usart_printf(DBGU,"\r\n");
//#endif
	
  for(reg_count=0;reg_count<write_len;){
		reg_value = (reg_buf[reg_count*2 ]<<8) | reg_buf[reg_count*2 + 1];
		reg_addr = reg_count + start_addr;
		
		//usart_printf(DBGU,"debug> write register - addr:%4x=%4x.\r\n",reg_addr,reg_value);
		
		switch( reg_addr ) {
			case MB_VOL_MAX :
			case MB_VOL_SCALE:
			case MB_VOL_ERR_RATE:
			case MB_VOL_STEP:
			case MB_VOL_STEP_INTERVAL:
			case MB_VOL_STEP_TIMEOUT:
			case MB_VOL_LEVEL1:
				
			case MB_CURRRENT_MAX :
			case MB_CUR_SCALE:
			case MB_CUR_ERR_RATE:
			case MB_CUR_STEP:
			case MB_CUR_STEP_INTERVAL:
			case MB_CUR_STEP_TIMEOUT:
			case MB_CUR_CTL_START:
				
			case MB_VOL_SET_L:
			case MB_VOL_SET_R:
				
			case VMETER_START_DELAY:
			case VMETER_STOP_DELAY:
			case MB_MPUMP_PWR_OFF_FREQ:
				reg40000[reg_addr] = reg_value;
				break;
			case MB_CUR_SET_L:
				reg30000[MB_HV_ST_L] &= 0xFF;
				reg40000[reg_addr] = reg_value;
				break;
			case MB_CUR_SET_R:
				reg30000[MB_HV_ST_R] &= 0xFF;
				reg40000[reg_addr] = reg_value;
				break;
			case MB_POWERPUMP_CTL:
				powerpump_ctl(reg_value);
				reg40000[reg_addr] = reg_value;
				break;
			case MB_MPUMP_CTL:
				mpump_ctl(reg_value);
				reg40000[reg_addr] = reg_value;
				break;
			case MB_VMETER_CTL:
				vmeter_ctl( reg_value );
				reg40000[reg_addr] = reg_value;
				break;
			case MB_VMETER_ERR_RATE:
			case MB_VMETER_SET0:
			case MB_VMETER_SET1:
				//usart_printf(DBGU,"MB_VMETER_SET:%4x\n",reg_value);
			case MB_VMETER_SET2:
			case MB_VMETER_SET3:
				reg40000[reg_addr] = reg_value;
				break;
			case MB_SAMPLE_ANGLE_SET:
				//sample_moto_ctl( reg_value );
				reg40000[reg_addr] = reg_value;
				break;
			case MB_SAMPLE_CTL	:
				reg40000[reg_addr] = reg_value;
				sample_monitor_ctl(reg_value);
				break;
			case MB_SAMPLE_HOLE0:
				reg40000[reg_addr] = reg_value;
				break;
			case MB_SAMPLE_START:
				reg40000[reg_addr] = reg_value;
				//sample_monitor_start(reg_value);
				break;
			case MB_SAMPLE_INTERVAL:
				sample_monitor_interval(reg_value);
				reg40000[reg_addr] = reg_value;
				break;
			case MB_SAMPLE_LED :
				sample_led_ctl( reg_value );
				reg40000[reg_addr] = reg_value;
				break;
			case MB_BAFFLE:
				baffle_ctl( reg_value );
				reg40000[reg_addr] = reg_value;
				break;
			case MB_BLEED_VALVE:
				bleed_valve_ctl(reg_value);
				reg40000[reg_addr] = reg_value;
				break;
			case MB_BAFFLE_INTERVAL:
				sample_baffle_interval();
				reg40000[reg_addr] = reg_value;
				break;
			case MB_SYS_AUTOCTL:
				reg40000[reg_addr] = reg_value;
				break;
			case MB_MOTOR_CTRL:
				//reg40000[reg_addr] = reg_value;
				//motor_ctrl(reg_value);
				break;
			//-----------------------------------------------------------				
			/*case MB_RELAY_SET:
				mb_bit_fun ( reg_value, ON, relay_set_ch );
				reg_count ++;
				break;
			case MB_RELAY_CLR:
				mb_bit_fun ( reg_value, OFF, relay_set_ch );
				reg_count ++;
				break;
			case MB_DO_SET:
				//mb_bit_fun ( reg_value, ON, io_set_ch );
				reg_count ++;
				break;
			case MB_DO_CLR:
				//mb_bit_fun ( reg_value, OFF, io_set_ch );
				reg_count ++;
				break;*/
				
			case MB_SMS_SERVER:
			case MB_SMS_SERVER1:
			case MB_SMS_SERVER2:
			case MB_SMS_SERVER3:
			case MB_SMS_SERVER4:
			case MB_SMS_SERVER5:
			case MB_SMS_SERVER6:
			case MB_SMS_SERVER7:
			case MB_SMS_SERVER8:
			case MB_SMS_SERVER9:
				sms_center[2*(reg_addr-MB_SMS_SERVER)  ] = reg_value>>8;
				sms_center[2*(reg_addr-MB_SMS_SERVER)+1] = reg_value;
				break;
			case MB_SMS_PHONE:
			case MB_SMS_PHONE1:
			case MB_SMS_PHONE2:
			case MB_SMS_PHONE3:
			case MB_SMS_PHONE4:
			case MB_SMS_PHONE5:
			case MB_SMS_PHONE6:
			case MB_SMS_PHONE7:
			case MB_SMS_PHONE8:
			case MB_SMS_PHONE9:
				sms_phone[2*(reg_addr-MB_SMS_PHONE)  ] = reg_value>>8;
				sms_phone[2*(reg_addr-MB_SMS_PHONE)+1] = reg_value;
				break;

			case MB_SMS_TEXT:
			case MB_SMS_TEXT1:
			case MB_SMS_TEXT2:
			case MB_SMS_TEXT3:
			case MB_SMS_TEXT4:
			case MB_SMS_TEXT5:
			case MB_SMS_TEXT6:
			case MB_SMS_TEXT7:
			case MB_SMS_TEXT8:
			case MB_SMS_TEXT9:
			case MB_SMS_TEXT10:
			case MB_SMS_TEXT11:
			case MB_SMS_TEXT12:
			case MB_SMS_TEXT13:
			case MB_SMS_TEXT14:
			case MB_SMS_TEXT15:
			case MB_SMS_TEXT16:
			case MB_SMS_TEXT17:
			case MB_SMS_TEXT18:
			case MB_SMS_TEXT19:
			case MB_SMS_TEXT20:
			case MB_SMS_TEXT21:
			case MB_SMS_TEXT22:
			case MB_SMS_TEXT23:
			case MB_SMS_TEXT24:
			case MB_SMS_TEXT25:
			case MB_SMS_TEXT26:
			case MB_SMS_TEXT27:
			case MB_SMS_TEXT28:
			case MB_SMS_TEXT29:
			case MB_SMS_TEXT30:
			case MB_SMS_TEXT31:
			case MB_SMS_TEXT32:
			case MB_SMS_TEXT33:
			case MB_SMS_TEXT34:
			case MB_SMS_TEXT35:
			case MB_SMS_TEXT36:
			case MB_SMS_TEXT37:
			case MB_SMS_TEXT38:
			case MB_SMS_TEXT39:
			case MB_SMS_TEXT40:
			case MB_SMS_TEXT41:
			case MB_SMS_TEXT42:
			case MB_SMS_TEXT43:
			case MB_SMS_TEXT44:
			case MB_SMS_TEXT45:
			case MB_SMS_TEXT46:
			case MB_SMS_TEXT47:
			case MB_SMS_TEXT48:
			case MB_SMS_TEXT49:
			case MB_SMS_TEXT50:
			case MB_SMS_TEXT51:
			case MB_SMS_TEXT52:
			case MB_SMS_TEXT53:
			case MB_SMS_TEXT54:
			case MB_SMS_TEXT55:
			case MB_SMS_TEXT56:
			case MB_SMS_TEXT57:
			case MB_SMS_TEXT58:
			case MB_SMS_TEXT59:
			case MB_SMS_TEXT60:
			case MB_SMS_TEXT61:
			case MB_SMS_TEXT62:
			case MB_SMS_TEXT63:
				sms_text[2*(reg_addr-MB_SMS_TEXT)  ] = reg_value>>8;
				sms_text[2*(reg_addr-MB_SMS_TEXT)+1] = reg_value;
				break;
			case MB_SMS_CMD:
				if ( reg_value & MB_SMS_SEND ){
					//usart_printf(DBGU,"debug>sms_center:%s\r\n",sms_center);	
					//usart_printf(DBGU,"debug>sms_phone:%s\r\n",sms_phone);	
					//usart_printf(DBGU,"debug>sms_text:%s\r\n",sms_text);	
					//gsmSendMessage(&sm_param,sms_text);
					gsmSendMessage(&sm_param,"AL-900系统信息\r\n您所制备的样品已经完成，系统将进入自动停机程序。");
				}
				break;
/*			case MB_DAC0:
			case MB_DAC1:
			case MB_DAC2:
			case MB_DAC3:
			case MB_DAC4:
			case MB_DAC5:
			case MB_DAC6:
			case MB_DAC7:
				//dac_set(reg_addr - MB_DAC0, reg_value);
				//reg_count ++;
				break;
			case MB_FD110A:
				FD110A_ctl(reg_value);
				//reg_count ++;
				break;
			case MB_BEEP_ON:
				reg_count ++;
				beep_set(reg_value, (reg_buf[reg_count*2]<<8) | reg_buf[reg_count*2 + 1] );
				//reg_count ++;
				break;
			case MB_LED_ON:
			case MB_LED_OFF:
				led(reg_value);
				//reg_count ++;
				break;
			case MB_L298_CTL:
				l298n(reg_value>>8, reg_value&0x0F);
				//reg_count ++;
				break;
*/		case MB_GSM_CTL:
				uartswSendStr(reg_buf);
			default :
				//reg_count ++;
				break;
		}
		reg_count ++;
	}
	
  mb_send_frame(mb_rx_buf,6);
	delayms(10);

	hvs_update_from_modbus(&hvsl);
 	hvs_update_from_modbus(&hvsr);
  return 0;
}


/* function:        report_slave_id(fun_no:0x11)
 * argument:        none
 * return value:    
 * description:     
 */
char report_slave_id(void)
{
  mb_tx_buf[0] = MB_DEV_ADDR;		//slave addr
  mb_tx_buf[1] = 0x11;          //fun no.
  mb_tx_buf[2] = 0x02;       		//byte count
  mb_tx_buf[3] = MB_DEV_ADDR;   //slave id
  mb_tx_buf[4] = 0xFF;          //run indicator status

  mb_send_frame(mb_tx_buf,5);
  MB_DBG("\r\nreport_slave_id and reset!\r\n");
  return 0;
}
