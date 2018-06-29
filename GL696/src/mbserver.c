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

#include "projdefs.h"
#include "modbus.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
err_t MB_tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t MB_tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Initialize the server application.
  * @param  None
  * @retval None
  */
void MB_tcp_server_init(void)
{
	struct tcp_pcb *pcb;                                 
	err_t err;	  

	/* Create a new TCP control block  */
	pcb = tcp_new();	
	if ( pcb == NULL )
		return ;

	/* Assign to the new pcb a local IP address and a port number */
	err = tcp_bind(pcb, IP_ADDR_ANY, 502);
	  
	if(err != ERR_USE)
	{
		/* Set the connection to the LISTEN state */
		pcb = tcp_listen(pcb);
		
		/* Specify the function to be called when a connection is established */
		tcp_accept(pcb, MB_tcp_server_accept);
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
err_t MB_tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{ 
  /* Tell TCP that this is the structure we wish to be passed for our
     callbacks. */
  //tcp_arg(pcb, mbs);
  
  /* Configure LwIP to use our call back functions. */
  //tcp_err(pcb, MB_tcp_server_err);

  /* Specify the function that should be called when the TCP connection receives data */
  tcp_recv(pcb, MB_tcp_server_recv);
 
  //tcp_poll(pcb, MB_tcp_poll, 10);

  tcp_accepted(pcb);

  return ERR_OK;  
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
static err_t MB_tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	uint8_t* ucBuf;
	uint8_t* pcAdu;
	uint16_t usLength;
	uint16_t tx_len=0;
	uint8_t type;
	
	if ( p != NULL ){
		ucBuf	 		= (uint8_t *)(p->payload);
		usLength 	= p->len;
		pcAdu 		= MB_CheckFrame(ucBuf,usLength,&type); 

		if ( pcAdu != NULL ){
			tx_len = MB_Process(pcAdu);
			if ( tx_len ){
			  err = tcp_write(pcb, ucBuf, tx_len+6, 0);
				tcp_output(pcb);
			}
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
