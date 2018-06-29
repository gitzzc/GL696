/**
  ******************************************************************************
  * @file    server.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   A sample UDP/TCP server application.
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
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/tcp.h"

#include <stdlib.h>
#include <string.h>

#include "projdefs.h"
#include "common.h"
#include "modbus.h"
#include "serial.h"
#include "timer.h"
#include "gpio.h"
#include "config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define NO_CHECK_SUM

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//MB_State mbs_tcp,mbs_rtu_t,mbs_rtu_r;
uint8_t tcp_connections=0;

struct tcp_pcb 	*pcb_buf[MEMP_NUM_TCP_PCB];
struct pbuf 	*p_buf[MEMP_NUM_TCP_PCB];

/* Private function prototypes -----------------------------------------------*/
err_t tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static void tcp_server_err(void *arg, err_t err);

/* Private functions ---------------------------------------------------------*/


psFRAME_HEAD Net_CheckFrame(uint8_t* buf,uint16_t len)
{
	psFRAME_HEAD psFrame = (psFRAME_HEAD)buf;
	uint16_t usTail,usSum;
	uint8_t* pcReserve;
	uint16_t sum,i;
	
	if ( len != psFrame->length + 6 )
		return NULL;

	pcReserve = buf + psFrame->length - 6;
	usSum 		= *(pcReserve + 8) | ((uint16_t)*(pcReserve+ 9) << 8);
	usTail		= *(pcReserve +10) | ((uint16_t)*(pcReserve+11) << 8);

	if ( psFrame->head != 0x90EB || usTail != 0x6F14 )
		return NULL;	
	for(sum=0,i=0;i<len-6;i++)
		sum += buf[i+2];
#ifndef NO_CHECK_SUM
	if ( sum != usSum )
		return NULL;
#endif	
	memcpy(LastTimeCode,psFrame+9,14);
	if ( psFrame->type != 0x01 )	//ЦёБо
		return NULL;
	return psFrame;
}

uint8_t NET_ProcessCmd(psFRAME_HEAD psFrame)
{
	int8_t  cmd_id_str[5];
	uint32_t cmd_id,cmd_para;
	uint8_t* cmd_buf,*fbuf;
	static uint8_t ld_ctrl=0;
	uint8_t i;
	float f;
	
	if ( psFrame == NULL )
		return pdFALSE;
	
	//memcpy(cmd_id_str,(uint8_t*)psFrame + sizeof(sFRAME_HEAD),4);
	memcpy(cmd_id_str,(uint8_t*)psFrame + 23,4);
	cmd_id_str[4] = 0;
	cmd_id = str2uint(cmd_id_str);
	if ( (cmd_id>>8) + (cmd_id&0xFF) != 0xFF )
		return pdFALSE;
	cmd_id = cmd_id >> 8;

	cmd_buf = (uint8_t*)psFrame + 23 + 4;
	if ( cmd_id < 0x4B ){
		cmd_para 	= cmd_buf[0] | ((uint16_t)cmd_buf[1]<<8);
		if ( cmd_para != 0x0100 )
			return pdFALSE;
	} else if ( cmd_id >= 0x4B && cmd_id < 0x70 ) {
		cmd_para 	= cmd_buf[0] | ((uint16_t)cmd_buf[1]<<8);
		fbuf = (uint8_t*)&f;
		fbuf[0] = cmd_buf[0];
		fbuf[1] = cmd_buf[1];
		fbuf[2] = cmd_buf[2];
		fbuf[3] = cmd_buf[3];
	} else if ( cmd_id >= 0x70 && cmd_id <= 0x75 ) {
		cmd_para 	= ((uint32_t)cmd_buf[0]<< 0) | ((uint32_t)cmd_buf[1]<< 8) | \
								((uint32_t)cmd_buf[2]<<16) | ((uint32_t)cmd_buf[3]<<24);
	}
		
	switch( cmd_id ){
		case 0x01:	DO_WriteDelay(DO7, 	pdLOW, 220);break;
		case 0x02:	DO_WriteDelay(DO8, 	pdLOW, 220);break;
		case 0x03:	DO_WriteDelay(DO11, pdLOW, 220);break;
		case 0x04:	DO_WriteDelay(DO9, 	pdLOW, 220);break;
		case 0x05:	DIO_Write(DO0,	DIO_ON);	break;
		case 0x06:	DIO_Write(DO0,	DIO_OFF); break;
		case 0x07:	DIO_Write(DO1,	DIO_ON);	break;
		case 0x08:	DIO_Write(DO1,	DIO_OFF);	break;
		case 0x09:	DIO_Write(DO2,	DIO_ON);	break;
		case 0x0A:	DIO_Write(DO2,	DIO_OFF);	break;
		case 0x0B:	DIO_Write(DO3,	DIO_ON);	break;
		case 0x0C:	DIO_Write(DO3,	DIO_OFF);	break;
		case 0x0D:	DIO_Write(DO4,	DIO_ON);	break;
		case 0x0E:	DIO_Write(DO4,	DIO_OFF);	break;
		case 0x0F:	DIO_Write(DO5,	DIO_ON);	break;
		case 0x10:	DIO_Write(DO5,	DIO_OFF);	break;
		case 0x11:	DO_WriteDelay(DO10,	pdLOW, 220);break;
		case 0x13:	DO_WriteDelay(DO12,	pdLOW, 220);break;
		case 0x15:	DIO_Write(DO13,	DIO_ON);	break;
		case 0x16:	DIO_Write(DO13,	DIO_OFF);	break;
		case 0x17:	DIO_Write(DO14,	DIO_ON);	break;
		case 0x18:	DIO_Write(DO14,	DIO_OFF);	break;
		case 0x19:	if( ld_ctrl == 0 ){ 
								DIO_Write(DO15,	DIO_ON); 	ld_ctrl |= (1<<0); }break;	//LD1	
		case 0x1A:	DIO_Write(DO15,	DIO_OFF); ld_ctrl &=~(1<<0);	break;
		case 0x1B:	if( ld_ctrl == 0 ){ 
								DIO_Write(DO16,	DIO_ON); 	ld_ctrl |= (1<<2); }break;	//LD3
		case 0x1C:	DIO_Write(DO16,	DIO_OFF);	ld_ctrl &=~(1<<2); 	break;
		case 0x1D:	if( ld_ctrl == 0 ){ 
								DIO_Write(DO17,	DIO_ON); 	ld_ctrl |= (1<<3); }break;	//LD4
		case 0x1E:	DIO_Write(DO17,	DIO_OFF);	ld_ctrl &=~(1<<3); 	break;
		case 0x1F:	
		case 0x21:
		case 0x23:
		case 0x25:
		case 0x27:
		case 0x29:
		case 0x2B:
		case 0x2D:
		case 0x2F:
		case 0x31:	sSys_cfg.spmu[(cmd_id-0x1F)/2].ctrl 	|= PMU_POWERON;
								sSys_cfg.spmu[(cmd_id-0x1F)/2].status |= PMU_UPDATE_CTRL;	break;
		case 0x20:	
		case 0x22:
		case 0x24:
		case 0x26:
		case 0x28:
		case 0x2A:
		case 0x2C:
		case 0x2E:
		case 0x30:
		case 0x32:	sSys_cfg.spmu[(cmd_id-0x20)/2].ctrl 	&= ~(PMU_POWERON);
								sSys_cfg.spmu[(cmd_id-0x20)/2].status |= PMU_UPDATE_CTRL;	break;
		case 0x35:		
		case 0x36:		
		case 0x37:	
		case 0x38:	
		case 0x39:	
		case 0x3A:	DO_WriteDelay(DO18, 	pdLOW, 200);break;
		case 0x3B:	break;
		case 0x3C:	break;
		case 0x3D:	break;
		case 0x3E:	break;
		case 0x3F:	break;
		case 0x40:	break;
		case 0x41:	break;
		case 0x42:	break;
		case 0x43:	sSys_cfg.spmu[cmd_id-0x3B].ctrl 	|= PMU_RESET;
								sSys_cfg.spmu[cmd_id-0x3B].status |= PMU_UPDATE_CTRL;	break;
		case 0x44:	break;
		case 0x45:	break;
		case 0x46:	break;
		case 0x47:	break;
		case 0x48:	break;
		case 0x49:	break;
		case 0x4A:	break;

		case 0x4B:
		case 0x4C:
		case 0x4D:
		case 0x4E:
		case 0x4F:
		case 0x50:
		case 0x51:
		case 0x52:
		case 0x53:
		case 0x54:	sSys_cfg.spmu[cmd_id-0x4B].vol_set = f*100;
								sSys_cfg.spmu[cmd_id-0x4B].status |= PMU_UPDATE_VOL;	break;
		
		case 0x70:	for(i=0;i<4;i++) sSys_cfg.gateway[i] = cmd_buf[3-i];ConfigWrite(&sSys_cfg,&sSys_def_cfg);break;
		case 0x71:	for(i=0;i<4;i++) sSys_cfg.ip_addr[i] = cmd_buf[3-i];ConfigWrite(&sSys_cfg,&sSys_def_cfg);break;
		case 0x72:	sSys_cfg.tcp_host_port 	= cmd_para;										ConfigWrite(&sSys_cfg,&sSys_def_cfg);break;
		case 0x73:	for(i=0;i<4;i++) sSys_cfg.udp_host[i]= cmd_buf[3-i];ConfigWrite(&sSys_cfg,&sSys_def_cfg);break;
		case 0x74:	sSys_cfg.udp_host_port 	= cmd_para;										ConfigWrite(&sSys_cfg,&sSys_def_cfg);break;
		case 0x75:	for(i=0;i<4;i++) sSys_cfg.ip_mask[i] = cmd_buf[3-i];ConfigWrite(&sSys_cfg,&sSys_def_cfg);break;
		default: 		return pdFALSE;
	}
	return pdTRUE;
}

/**
  * @brief  Initialize the server application.
  * @param  None
  * @retval None
  */
void tcp_server_init(void)
{
	struct tcp_pcb *pcb;                                 
	err_t err;	  

	/* Create a new TCP control block  */
	pcb = tcp_new();	
	if ( pcb == NULL )
		return ;

	/* Assign to the new pcb a local IP address and a port number */
	err = tcp_bind(pcb, IP_ADDR_ANY, sSys_cfg.tcp_host_port);
	  
	if(err != ERR_USE)
	{
		/* Set the connection to the LISTEN state */
		pcb = tcp_listen(pcb);
		
		/* Specify the function to be called when a connection is established */
		tcp_accept(pcb, tcp_server_accept);
  }
	else
	{
		/* We enter here if a conection to the addr IP address already exists */
		/* so we don't need to establish a new one */
		tcp_close(pcb);
	}            
}

/**
  * @brief  This funtion is called when a TCP connection has been established on the port TCP_PORT.
  * @param  arg	user supplied argument 
  * @param  pcb	the tcp_pcb which accepted the connection
  * @param  err error value
  * @retval ERR_OK
  */
err_t tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{ 
  /* Tell TCP that this is the structure we wish to be passed for our
     callbacks. */
  //tcp_arg(pcb, mbs);
  
  /* Configure LwIP to use our call back functions. */
  tcp_err(pcb, tcp_server_err);

  /* Specify the function that should be called when the TCP connection receives data */
  tcp_recv(pcb, tcp_server_recv);
 
  //tcp_poll(pcb, tcp_poll, 10);

  tcp_accepted(pcb);

  return ERR_OK;  
}

/**
  * @brief  This function is called when an error occurs on the connection 
  * @param  arg
  * @parm   err
  * @retval None 
  */
static void tcp_server_err(void *arg, err_t err)
{
}


/**
  * @brief  This function is called when a data is received over the TCP_PORT.
  *         The received data contains the number of the led to be toggled.
  * @param  arg	user supplied argument 
  * @param  pcb	the tcp_pcb which accepted the connection
  * @param  p the packet buffer that was received
  * @param  err error value
  * @retval ERR_OK
  */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	uint8_t* ucBuf;
	uint16_t usLength;
	psFRAME_HEAD psFrame;
	uint8_t ret;
	uint16_t sum,i;
	
	if ( p != NULL ){
		ucBuf	 		= (uint8_t *)(p->payload);
		usLength 	= p->len;
		psFrame = Net_CheckFrame( ucBuf,usLength );
		if ( psFrame ){
			ret = NET_ProcessCmd(psFrame);

			psFrame->type = 0x02;
				psFrame->ret = 0x00;
			if ( ret == pdFALSE )
				psFrame->ret = 0x01;
			for(sum=0,i=2;i<usLength-6;i++)
				sum += ucBuf[i];
			ucBuf[usLength-4] = sum;
			ucBuf[usLength-3] = sum>>8;
			tcp_write(pcb, ucBuf, usLength, TCP_WRITE_FLAG_COPY);
		}

		/* We call this function to tell the LwIp that we have processed the data */
		/* This lets the stack advertise a larger window, so more data can be received*/
		tcp_recved(pcb, p->tot_len);
		/* Free the p buffer */
		pbuf_free(p);
	} else if (err == ERR_OK) {

		/* When the pbuf is NULL and the err is ERR_OK, the remote end is closing the connection. */
		/* We free the allocated memory and we close the connection */
		return tcp_close(pcb);
	}
	
	return ERR_OK;
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
