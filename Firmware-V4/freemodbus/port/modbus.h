/*
 *	Copyright (c) 2006,
 *	All ringts reserved.
 *
 *          
 *	modbus 协议分析程序
 *         完成RTU和ASCII两种帧格式的解码，
 *
 */

#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "FreeRTOS.h"
#include "queue.h"

#include "mb.h"
#include "port.h"
#include "mbframe.h"
#include "mb_reg_map.h"





struct modbus_state {
  unsigned char timer;
  char state;
  int len;
};

extern volatile USHORT   usRegInputBuf[REG_INPUT_NREGS];
extern volatile USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];


//-----------------------------------------------------------------------
void modbus_tcp_appcall(void);
void modbus_tcp_init(void);

USHORT eMBRegHolding_Read( USHORT usAddress );
void eMBRegHolding_Write( USHORT usAddress, USHORT usRegVal );
USHORT eMBRegInput_Read( USHORT usAddress );
void eMBRegInput_Write( USHORT usAddress, USHORT usRegVal );

void vMBTCPSlaveTask( void *pvParameters );
void vMBRTUSlaveTask( void *pvParameters );

#endif
