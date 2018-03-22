
#ifndef __CAN_H__
#define __CAN_H__

#include "queue.h"
#include "stm32f10x.h"


typedef void * xCANMsgHandle;

#define MAX_PSU_CHANNEL 2

#define PDC_ADDR	0xF4
#define PDU_ADDR	0xE5
#define PSU1_ADDR	0xE3
#define PSU2_ADDR	0xE4


#define PDC_TO_PDU_ID	0x1806E5F4
#define PDU_TO_PDC_ID	0x1006F4E5

#define PDC_TO_PSU1_ID	0x1806E3F4
#define PSU1_TO_PDC_ID	0x0C06F4E3
 	
#define PDC_TO_PSU2_ID	0x1806E4F4
#define PSU2_TO_PDC_ID	0x0806F4E4


#define PDC_FUN_REQUEST	0x03
#define PDC_FUN_SET		0x10

#define PDC_FRAME_ID_CH1	0x00
#define PDC_FRAME_ID_CH2	0x01


typedef struct 
{
	u16 input_cur[2];
	u16 input_vol[2];
	u16 output_state[1];
	u16 output_set[1];
} PDU_STATE;

typedef struct
{
	u16 current;
	u16 voltage;
	u8	ctrl_state;
	u8	over_cur;
	u8	over_vol;
	u16 vol_set;
} PSU_STATE;

extern PDU_STATE pdu_state;
extern PSU_STATE psu_state[2];

extern xQueueHandle xCANTxQueue;
extern xQueueHandle xCANRxQueue;

void vStartCANTasks( unsigned portBASE_TYPE uxPriority );
signed portBASE_TYPE xCANPutMsg( CanTxMsg *tx, portTickType xBlockTime );
signed portBASE_TYPE xCANGetMsg( CanTxMsg *rx, portTickType xBlockTime );
void CAN_HardwareConfig(void);
signed portBASE_TYPE xCANSetPDU( u16 state );
signed portBASE_TYPE xCANSetPSU( u8 channle,u16 voltage );


#endif

