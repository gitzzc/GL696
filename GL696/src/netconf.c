/**
  ******************************************************************************
  * @file    netconf.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    07/16/2010 
  * @brief   Network connection configuration
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
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/igmp.h"
#include "ethernetif.h"
#include "main.h"
#include "netconf.h"

#include <stdio.h>
#include <string.h>
#include "projdefs.h"

/* Private typedef -----------------------------------------------------------*/
#define LCD_DELAY             3000
#define KEY_DELAY 			  3000
#define LCD_TIMER_MSECS       250
#define MAX_DHCP_TRIES        4

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct netif netif;
uint32_t TCPTimer = 0;
uint32_t ARPTimer = 0;
uint32_t IGMPTimer= 0;
//uint32_t IPaddress=0;

uint32_t DHCPfineTimer = 0;
uint32_t DHCPcoarseTimer = 0;

//uint32_t DisplayTimer = 0;
//uint8_t LedToggle = 4;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
void LwIP_Init(void)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;
  uint8_t macaddress[6];

  /* Initializes the dynamic memory heap defined by MEM_SIZE.*/
  mem_init();

  /* Initializes the memory pools defined by MEMP_NUM_x.*/
  memp_init();

	macaddress[0] = sSys_cfg.mac[0];
	macaddress[1] = sSys_cfg.mac[0]>>8;
	macaddress[2] = sSys_cfg.mac[1];
	macaddress[3] = sSys_cfg.mac[1]>>8;
	macaddress[4] = sSys_cfg.mac[2];
	macaddress[5] = sSys_cfg.mac[2]>>8;
  
  if ( sSys_cfg.ip_mode == 1 ){	//#ifdef USE_DHCP
    ipaddr.addr 	= 0;
    netmask.addr 	= 0;
    gw.addr 			= 0;
  } else {
    IP4_ADDR(&ipaddr, sSys_cfg.ip_addr[3],sSys_cfg.ip_addr[2],sSys_cfg.ip_addr[1],sSys_cfg.ip_addr[0]);
    IP4_ADDR(&netmask,sSys_cfg.ip_mask[3],sSys_cfg.ip_mask[2],sSys_cfg.ip_mask[1],sSys_cfg.ip_mask[0]);
    IP4_ADDR(&gw, 	  sSys_cfg.gateway[3],sSys_cfg.gateway[2],sSys_cfg.gateway[1],sSys_cfg.gateway[0]);
  }

  Set_MAC_Address(macaddress);

  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
            struct ip_addr *netmask, struct ip_addr *gw,
            void *state, err_t (* init)(struct netif *netif),
            err_t (* input)(struct pbuf *p, struct netif *netif))
    
   Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.

  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/
  netif_add(&netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /*  Registers the default network interface.*/
  netif_set_default(&netif);


  if ( sSys_cfg.ip_mode == 1 ){	//#ifdef USE_DHCP
    /*  Creates a new DHCP client for this interface on the first call.
    Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
    the predefined regular intervals after starting the client.
    You can peek in the netif->dhcp struct for the actual DHCP status.*/
    dhcp_start(&netif);
  }
  /*  When the netif is fully configured this function must be called.*/
  netif_set_up(&netif);

}

/**
  * @brief  Called when a frame is received
  * @param  None
  * @retval None
  */
void LwIP_Pkt_Handle(void)
{
  /* Read a received packet from the Ethernet buffers and send it to the lwIP for handling */
  ethernetif_input(&netif);
}

/**
  * @brief  LwIP periodic tasks
  * @param  localtime the current LocalTime value
  * @retval None
  */
void LwIP_Periodic_Handle(__IO uint32_t localtime)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;

#if LWIP_TCP
  /* TCP periodic process every 250 ms */
  if (localtime - TCPTimer >= TCP_TMR_INTERVAL)
  {
    TCPTimer =  localtime;
    tcp_tmr();
  }
#endif
  
  /* ARP periodic process every 5s */
  if ((localtime - ARPTimer) >= ARP_TMR_INTERVAL)
  {
    ARPTimer =  localtime;
    etharp_tmr();
  }

  if ( sSys_cfg.ip_mode == 1 ){	//#ifdef USE_DHCP
    /* Fine DHCP periodic process every 500ms */
    if (localtime - DHCPfineTimer >= DHCP_FINE_TIMER_MSECS)
    {
      DHCPfineTimer =  localtime;
      dhcp_fine_tmr();
    }
    /* DHCP Coarse periodic process every 60s */
    if (localtime - DHCPcoarseTimer >= DHCP_COARSE_TIMER_MSECS)
    {
      DHCPcoarseTimer =  localtime;
      dhcp_coarse_tmr();
    }
    /* If no response from a DHCP server for MAX_DHCP_TRIES times */
	  /* stop the dhcp client and set a static IP address */
    if (netif.dhcp->tries > MAX_DHCP_TRIES)
    { 
      dhcp_stop(&netif);
        
			IP4_ADDR(&ipaddr, sSys_cfg.ip_addr[3],sSys_cfg.ip_addr[2],sSys_cfg.ip_addr[1],sSys_cfg.ip_addr[0]);
			IP4_ADDR(&netmask,sSys_cfg.ip_mask[3],sSys_cfg.ip_mask[2],sSys_cfg.ip_mask[1],sSys_cfg.ip_mask[0]);
			IP4_ADDR(&gw, 	  sSys_cfg.gateway[3],sSys_cfg.gateway[2],sSys_cfg.gateway[1],sSys_cfg.gateway[0]);
      netif_set_addr(&netif, &ipaddr , &netmask, &gw);
    } 
  }
	
#if LWIP_IGMP
  /* TCP periodic process every 250 ms */
  if (localtime - IGMPTimer >= IGMP_TMR_INTERVAL)
  {
    IGMPTimer =  localtime;
    igmp_tmr();
  }
#endif
}


#ifdef USE_LCD
/**
  * @brief  LCD & LEDs periodic handling
  * @param  localtime: the current LocalTime value
  * @retval None
  */
void Display_Periodic_Handle(__IO uint32_t localtime)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;
  uint8_t iptab[4];
  uint8_t iptxt[20];
  
  /* 250 ms period*/
  if (localtime - DisplayTimer >= LCD_TIMER_MSECS)
  {
    DisplayTimer = localtime;
    
    /* We have got a new IP address so update the display */
    if (IPaddress != netif.ip_addr.addr)
    {
      /* Read the new IP address */
      IPaddress = netif.ip_addr.addr;

      iptab[0] = (uint8_t)(IPaddress >> 24);
      iptab[1] = (uint8_t)(IPaddress >> 16);
      iptab[2] = (uint8_t)(IPaddress >> 8);
      iptab[3] = (uint8_t)(IPaddress);

      sprintf((char*)iptxt, "   %d.%d.%d.%d    ", iptab[3], iptab[2], iptab[1], iptab[0]);

      /* Display the new IP address */
#ifdef USE_DHCP
      if (netif.flags & NETIF_FLAG_DHCP)
      {        
        /* Display the IP address */
        LCD_DisplayStringLine(Line7, "IP address assigned ");
        LCD_DisplayStringLine(Line8, "  by a DHCP server  ");
        LCD_DisplayStringLine(Line9, iptxt);	   	
      }
      else
      {
        LCD_DisplayStringLine(Line8, "  Static IP address   ");
        LCD_DisplayStringLine(Line9, iptxt);	
      } 
    }
    else if (IPaddress == 0)
    {
      /* We still waiting for the DHCP server */
      LCD_DisplayStringLine(Line4, "     Looking for    ");
      LCD_DisplayStringLine(Line5, "     DHCP server    ");
      LCD_DisplayStringLine(Line6, "     please wait... ");
        
      LedToggle &= 3;
      STM_EVAL_LEDToggle((Led_TypeDef)(LedToggle++));

      /* If no response from a DHCP server for MAX_DHCP_TRIES times */
	    /* stop the dhcp client and set a static IP address */
      if (netif.dhcp->tries > MAX_DHCP_TRIES)
      { 
        LCD_DisplayStringLine(Line7, "    DHCP timeout    ");
        dhcp_stop(&netif);
        
        IP4_ADDR(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
        IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
        IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
        netif_set_addr(&netif, &ipaddr , &netmask, &gw);
      } 
    }
  }
#else
    LCD_DisplayStringLine(Line8, "  Static IP address   ");
    LCD_DisplayStringLine(Line9, iptxt);
    }
 }
#endif	
}
#endif

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
