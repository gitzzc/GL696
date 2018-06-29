/**
  ******************************************************************************
  * @file    stm32f107.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    07/16/2010 
  * @brief   STM32F107 hardware configuration
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
#include "stm32_eth.h"
#include "stm32f107.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ETHERNET MACMIIAR register Mask */
#define MACMIIAR_CR_MASK    ((uint32_t)0xFFFFFFE3) 

/* ETHERNET MACCR register Mask */
#define MACCR_CLEAR_MASK    ((uint32_t)0xFF20810F)  

/* ETHERNET MACFCR register Mask */
#define MACFCR_CLEAR_MASK   ((uint32_t)0x0000FF41)

/* ETHERNET DMAOMR register Mask */
#define DMAOMR_CLEAR_MASK   ((uint32_t)0xF8DE3F23)

#define PHY_STS                          0x10		//PHY Status Register (PHYSTS)
/** @defgroup PHY Status Register 
  * @{
  */ 
#define PHY_MII_INTERRUPT               ((u16)0x0040)
#define PHY_DUPLEX_STATUS               ((u16)0x0004)
#define PHY_SPEED_STATUS                ((u16)0x0002)
#define PHY_LINK_STATUS                 ((u16)0x0001)

#define PHY_MICR                         0x11       //MII Interrupt Control Register
#define PHY_INTEN                 		((u16)0x0002)
#define PHY_INT_OE                		((u16)0x0001)

#define PHY_MISR                         0x12       //MII Interrupt Status and Misc
#define PHY_LQ_INT	                 	((u16)(1<<15))
#define PHY_ED_INT	                 	((u16)(1<<14))
#define PHY_LINK_INT                 	((u16)(1<<13))
#define PHY_SPD_INT                		((u16)(1<<12))
#define PHY_DUP_INT                		((u16)(1<<11))
#define PHY_ANC_INT                		((u16)(1<<10))
#define PHY_FHF_INT                		((u16)(1<<9))
#define PHY_RHF_INT                		((u16)(1<<8))
#define PHY_LQ_INT_EN              		((u16)(1<<7))
#define PHY_ED_INT_EN              		((u16)(1<<6))
#define PHY_LINK_INT_EN            		((u16)(1<<5))
#define PHY_SPD_INT_EN              	((u16)(1<<4))
#define PHY_DUP_INT_EN              	((u16)(1<<3))
#define PHY_ANC_INT_EN              	((u16)(1<<2))
#define PHY_FHF_INT_EN              	((u16)(1<<1))
#define PHY_RHF_INT_EN              	((u16)(1<<0))

#define PHY_RBR                         0x17       //RMII and Bypass Register (RBR)
#define PHY_RX_PORT						((u16)(1<<11))
#define PHY_TX_SOURCE					((u16)(1<<9))
  

//#define MII_MODE          /* MII mode for STM3210C-EVAL Board (MB784) (check jumpers setting) */
#define RMII_MODE       /* RMII mode for STM3210C-EVAL Board (MB784) (check jumpers setting) */

/*--------------- LCD Messages ---------------*/
#define MESSAGE1   "     STM32F107      "
#define MESSAGE2   " Connectivity Line  "
#define MESSAGE3   " IAP over Ethernet  "
#define MESSAGE4   "                    "

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static ETH_InitTypeDef ETH_InitStructure;

/* Private functions ---------------------------------------------------------*/
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void ADC_Configuration(void);
void Ethernet_Configuration(void);

static void ETH_Delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
  for(index = nCount; index != 0; index--)
  {
  }
}


/*-----------------------------------------------------------*/
void IWDG_init(void)
{
	/* IWDG timeout equal to 100 ms (the timeout may varies due to LSI frequency
	dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz */
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	
	/* Set counter reload value to 625 */
	IWDG_SetReload(625);
	
	/* Reload IWDG counter */
	IWDG_ReloadCounter();
	
	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}


void ETH_MAC_Config(void)
{
  uint32_t RegValue = 0, tmpreg = 0;

  /*------------------------ ETHERNET MACCR Configuration --------------------*/
  /* Get the ETHERNET MACCR value */  
  tmpreg = ETH->MACCR;
  /* Clear WD, PCE, PS, TE and RE bits */
  tmpreg &= MACCR_CLEAR_MASK;
  /* Set the WD bit according to ETH_Watchdog value */
  /* Set the JD: bit according to ETH_Jabber value */
  /* Set the IFG bit according to ETH_InterFrameGap value */ 
  /* Set the DCRS bit according to ETH_CarrierSense value */  
  /* Set the FES bit according to ETH_Speed value */ 
  /* Set the DO bit according to ETH_ReceiveOwn value */ 
  /* Set the LM bit according to ETH_LoopbackMode value */ 
  /* Set the DM bit according to ETH_Mode value */ 
  /* Set the IPC bit according to ETH_ChecksumOffload value */                   
  /* Set the DR bit according to ETH_RetryTransmission value */ 
  /* Set the ACS bit according to ETH_AutomaticPadCRCStrip value */ 
  /* Set the BL bit according to ETH_BackOffLimit value */ 
  /* Set the DC bit according to ETH_DeferralCheck value */                          
  tmpreg |= (uint32_t)(ETH_InitStructure.ETH_Watchdog | 
                  ETH_InitStructure.ETH_Jabber | 
                  ETH_InitStructure.ETH_InterFrameGap |
                  ETH_InitStructure.ETH_CarrierSense |
                  ETH_InitStructure.ETH_Speed | 
                  ETH_InitStructure.ETH_ReceiveOwn |
                  ETH_InitStructure.ETH_LoopbackMode |
                  ETH_InitStructure.ETH_Mode | 
                  ETH_InitStructure.ETH_ChecksumOffload |    
                  ETH_InitStructure.ETH_RetryTransmission | 
                  ETH_InitStructure.ETH_AutomaticPadCRCStrip | 
                  ETH_InitStructure.ETH_BackOffLimit | 
                  ETH_InitStructure.ETH_DeferralCheck);
  /* Write to ETHERNET MACCR */
  ETH->MACCR = (uint32_t)tmpreg;
  
  /*----------------------- ETHERNET MACFFR Configuration --------------------*/ 
  /* Set the RA bit according to ETH_ReceiveAll value */
  /* Set the SAF and SAIF bits according to ETH_SourceAddrFilter value */
  /* Set the PCF bit according to ETH_PassControlFrames value */
  /* Set the DBF bit according to ETH_BroadcastFramesReception value */
  /* Set the DAIF bit according to ETH_DestinationAddrFilter value */
  /* Set the PR bit according to ETH_PromiscuousMode value */
  /* Set the PM, HMC and HPF bits according to ETH_MulticastFramesFilter value */
  /* Set the HUC and HPF bits according to ETH_UnicastFramesFilter value */
  /* Write to ETHERNET MACFFR */  
  ETH->MACFFR = (uint32_t)(ETH_InitStructure.ETH_ReceiveAll | 
                          ETH_InitStructure.ETH_SourceAddrFilter |
                          ETH_InitStructure.ETH_PassControlFrames |
                          ETH_InitStructure.ETH_BroadcastFramesReception | 
                          ETH_InitStructure.ETH_DestinationAddrFilter |
                          ETH_InitStructure.ETH_PromiscuousMode |
                          ETH_InitStructure.ETH_MulticastFramesFilter |
                          ETH_InitStructure.ETH_UnicastFramesFilter); 
  /*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
  /* Write to ETHERNET MACHTHR */
  ETH->MACHTHR = (uint32_t)ETH_InitStructure.ETH_HashTableHigh;
  /* Write to ETHERNET MACHTLR */
  ETH->MACHTLR = (uint32_t)ETH_InitStructure.ETH_HashTableLow;
  /*----------------------- ETHERNET MACFCR Configuration --------------------*/
  /* Get the ETHERNET MACFCR value */  
  tmpreg = ETH->MACFCR;
  /* Clear xx bits */
  tmpreg &= MACFCR_CLEAR_MASK;
  
  /* Set the PT bit according to ETH_PauseTime value */
  /* Set the DZPQ bit according to ETH_ZeroQuantaPause value */
  /* Set the PLT bit according to ETH_PauseLowThreshold value */
  /* Set the UP bit according to ETH_UnicastPauseFrameDetect value */
  /* Set the RFE bit according to ETH_ReceiveFlowControl value */
  /* Set the TFE bit according to ETH_TransmitFlowControl value */  
  tmpreg |= (uint32_t)((ETH_InitStructure.ETH_PauseTime << 16) | 
                   ETH_InitStructure.ETH_ZeroQuantaPause |
                   ETH_InitStructure.ETH_PauseLowThreshold |
                   ETH_InitStructure.ETH_UnicastPauseFrameDetect | 
                   ETH_InitStructure.ETH_ReceiveFlowControl |
                   ETH_InitStructure.ETH_TransmitFlowControl); 
  /* Write to ETHERNET MACFCR */
  ETH->MACFCR = (uint32_t)tmpreg;
  /*----------------------- ETHERNET MACVLANTR Configuration -----------------*/
  /* Set the ETV bit according to ETH_VLANTagComparison value */
  /* Set the VL bit according to ETH_VLANTagIdentifier value */  
  ETH->MACVLANTR = (uint32_t)(ETH_InitStructure.ETH_VLANTagComparison | 
                             ETH_InitStructure.ETH_VLANTagIdentifier); 
       
  /*-------------------------------- DMA Config ------------------------------*/
  /*----------------------- ETHERNET DMAOMR Configuration --------------------*/
  /* Get the ETHERNET DMAOMR value */  
  tmpreg = ETH->DMAOMR;
  /* Clear xx bits */
  tmpreg &= DMAOMR_CLEAR_MASK;
  
  /* Set the DT bit according to ETH_DropTCPIPChecksumErrorFrame value */
  /* Set the RSF bit according to ETH_ReceiveStoreForward value */
  /* Set the DFF bit according to ETH_FlushReceivedFrame value */
  /* Set the TSF bit according to ETH_TransmitStoreForward value */
  /* Set the TTC bit according to ETH_TransmitThresholdControl value */
  /* Set the FEF bit according to ETH_ForwardErrorFrames value */
  /* Set the FUF bit according to ETH_ForwardUndersizedGoodFrames value */
  /* Set the RTC bit according to ETH_ReceiveThresholdControl value */
  /* Set the OSF bit according to ETH_SecondFrameOperate value */
  tmpreg |= (uint32_t)(ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame | 
                  ETH_InitStructure.ETH_ReceiveStoreForward |
                  ETH_InitStructure.ETH_FlushReceivedFrame |
                  ETH_InitStructure.ETH_TransmitStoreForward | 
                  ETH_InitStructure.ETH_TransmitThresholdControl |
                  ETH_InitStructure.ETH_ForwardErrorFrames |
                  ETH_InitStructure.ETH_ForwardUndersizedGoodFrames |
                  ETH_InitStructure.ETH_ReceiveThresholdControl |                                   
                  ETH_InitStructure.ETH_SecondFrameOperate); 
  /* Write to ETHERNET DMAOMR */
  ETH->DMAOMR = (uint32_t)tmpreg;
  
  /*----------------------- ETHERNET DMABMR Configuration --------------------*/ 
  /* Set the AAL bit according to ETH_AddressAlignedBeats value */
  /* Set the FB bit according to ETH_FixedBurst value */
  /* Set the RPBL and 4*PBL bits according to ETH_RxDMABurstLength value */
  /* Set the PBL and 4*PBL bits according to ETH_TxDMABurstLength value */
  /* Set the DSL bit according to ETH_DesciptorSkipLength value */
  /* Set the PR and DA bits according to ETH_DMAArbitration value */         
  ETH->DMABMR = (uint32_t)(ETH_InitStructure.ETH_AddressAlignedBeats | 
                          ETH_InitStructure.ETH_FixedBurst |
                          ETH_InitStructure.ETH_RxDMABurstLength | /* !! if 4xPBL is selected for Tx or Rx it is applied for the other */
                          ETH_InitStructure.ETH_TxDMABurstLength | 
                         (ETH_InitStructure.ETH_DescriptorSkipLength << 2) |
                          ETH_InitStructure.ETH_DMAArbitration |
                          ETH_DMABMR_USP); /* Enable use of separate PBL for Rx and Tx */  
  
  /* Return Ethernet configuration success */
  //return ETH_SUCCESS;
}

uint32_t ETH_Init_PHY( void )
{
/* ETHERNET errors */
#define  ETH_ERROR              ((uint32_t)0)
#define  ETH_SUCCESS            ((uint32_t)1)

  uint32_t RegValue = 0, tmpreg = 0;
  __IO uint32_t i = 0;
  RCC_ClocksTypeDef  rcc_clocks;
  uint32_t hclk = 60000000;
  __IO uint32_t timeout = 0;
  /* Check the parameters */
  /* MAC --------------------------*/ 
  assert_param(IS_ETH_AUTONEGOTIATION(ETH_InitStructure.ETH_AutoNegotiation));
  assert_param(IS_ETH_WATCHDOG(ETH_InitStructure.ETH_Watchdog));
  assert_param(IS_ETH_JABBER(ETH_InitStructure.ETH_Jabber));
  assert_param(IS_ETH_INTER_FRAME_GAP(ETH_InitStructure.ETH_InterFrameGap));
  assert_param(IS_ETH_CARRIER_SENSE(ETH_InitStructure.ETH_CarrierSense));
  assert_param(IS_ETH_SPEED(ETH_InitStructure.ETH_Speed));
  assert_param(IS_ETH_RECEIVE_OWN(ETH_InitStructure.ETH_ReceiveOwn));
  assert_param(IS_ETH_LOOPBACK_MODE(ETH_InitStructure.ETH_LoopbackMode));
  assert_param(IS_ETH_DUPLEX_MODE(ETH_InitStructure.ETH_Mode));
  assert_param(IS_ETH_CHECKSUM_OFFLOAD(ETH_InitStructure.ETH_ChecksumOffload));
  assert_param(IS_ETH_RETRY_TRANSMISSION(ETH_InitStructure.ETH_RetryTransmission));
  assert_param(IS_ETH_AUTOMATIC_PADCRC_STRIP(ETH_InitStructure.ETH_AutomaticPadCRCStrip));
  assert_param(IS_ETH_BACKOFF_LIMIT(ETH_InitStructure.ETH_BackOffLimit));
  assert_param(IS_ETH_DEFERRAL_CHECK(ETH_InitStructure.ETH_DeferralCheck));
  assert_param(IS_ETH_RECEIVE_ALL(ETH_InitStructure.ETH_ReceiveAll));
  assert_param(IS_ETH_SOURCE_ADDR_FILTER(ETH_InitStructure.ETH_SourceAddrFilter));
  assert_param(IS_ETH_CONTROL_FRAMES(ETH_InitStructure.ETH_PassControlFrames));
  assert_param(IS_ETH_BROADCAST_FRAMES_RECEPTION(ETH_InitStructure.ETH_BroadcastFramesReception));
  assert_param(IS_ETH_DESTINATION_ADDR_FILTER(ETH_InitStructure.ETH_DestinationAddrFilter));
  assert_param(IS_ETH_PROMISCUOUS_MODE(ETH_InitStructure.ETH_PromiscuousMode));
  assert_param(IS_ETH_MULTICAST_FRAMES_FILTER(ETH_InitStructure.ETH_MulticastFramesFilter));  
  assert_param(IS_ETH_UNICAST_FRAMES_FILTER(ETH_InitStructure.ETH_UnicastFramesFilter));
  assert_param(IS_ETH_PAUSE_TIME(ETH_InitStructure.ETH_PauseTime));
  assert_param(IS_ETH_ZEROQUANTA_PAUSE(ETH_InitStructure.ETH_ZeroQuantaPause));
  assert_param(IS_ETH_PAUSE_LOW_THRESHOLD(ETH_InitStructure.ETH_PauseLowThreshold));
  assert_param(IS_ETH_UNICAST_PAUSE_FRAME_DETECT(ETH_InitStructure.ETH_UnicastPauseFrameDetect));
  assert_param(IS_ETH_RECEIVE_FLOWCONTROL(ETH_InitStructure.ETH_ReceiveFlowControl));
  assert_param(IS_ETH_TRANSMIT_FLOWCONTROL(ETH_InitStructure.ETH_TransmitFlowControl));
  assert_param(IS_ETH_VLAN_TAG_COMPARISON(ETH_InitStructure.ETH_VLANTagComparison));
  assert_param(IS_ETH_VLAN_TAG_IDENTIFIER(ETH_InitStructure.ETH_VLANTagIdentifier));
  /* DMA --------------------------*/
  assert_param(IS_ETH_DROP_TCPIP_CHECKSUM_FRAME(ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame));
  assert_param(IS_ETH_RECEIVE_STORE_FORWARD(ETH_InitStructure.ETH_ReceiveStoreForward));
  assert_param(IS_ETH_FLUSH_RECEIVE_FRAME(ETH_InitStructure.ETH_FlushReceivedFrame));
  assert_param(IS_ETH_TRANSMIT_STORE_FORWARD(ETH_InitStructure.ETH_TransmitStoreForward));
  assert_param(IS_ETH_TRANSMIT_THRESHOLD_CONTROL(ETH_InitStructure.ETH_TransmitThresholdControl));
  assert_param(IS_ETH_FORWARD_ERROR_FRAMES(ETH_InitStructure.ETH_ForwardErrorFrames));
  assert_param(IS_ETH_FORWARD_UNDERSIZED_GOOD_FRAMES(ETH_InitStructure.ETH_ForwardUndersizedGoodFrames));
  assert_param(IS_ETH_RECEIVE_THRESHOLD_CONTROL(ETH_InitStructure.ETH_ReceiveThresholdControl));
  assert_param(IS_ETH_SECOND_FRAME_OPERATE(ETH_InitStructure.ETH_SecondFrameOperate));
  assert_param(IS_ETH_ADDRESS_ALIGNED_BEATS(ETH_InitStructure.ETH_AddressAlignedBeats));
  assert_param(IS_ETH_FIXED_BURST(ETH_InitStructure.ETH_FixedBurst));
  assert_param(IS_ETH_RXDMA_BURST_LENGTH(ETH_InitStructure.ETH_RxDMABurstLength));
  assert_param(IS_ETH_TXDMA_BURST_LENGTH(ETH_InitStructure.ETH_TxDMABurstLength)); 
  assert_param(IS_ETH_DMA_DESC_SKIP_LENGTH(ETH_InitStructure.ETH_DescriptorSkipLength));  
  assert_param(IS_ETH_DMA_ARBITRATION_ROUNDROBIN_RXTX(ETH_InitStructure.ETH_DMAArbitration));       
  /*-------------------------------- MAC Config ------------------------------*/   
  /*---------------------- ETHERNET MACMIIAR Configuration -------------------*/
  /* Get the ETHERNET MACMIIAR value */
  tmpreg = ETH->MACMIIAR;
  /* Clear CSR Clock Range CR[2:0] bits */
  tmpreg &= MACMIIAR_CR_MASK;
  /* Get hclk frequency value */
  RCC_GetClocksFreq(&rcc_clocks);
  hclk = rcc_clocks.HCLK_Frequency;
  /* Set CR bits depending on hclk value */
  if((hclk >= 20000000)&&(hclk < 35000000))
  {
    /* CSR Clock Range between 20-35 MHz */
    tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div16;
  }
  else if((hclk >= 35000000)&&(hclk < 60000000))
  {
    /* CSR Clock Range between 35-60 MHz */ 
    tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div26;    
  }  
  else /* ((hclk >= 60000000)&&(hclk <= 72000000)) */
  {
    /* CSR Clock Range between 60-72 MHz */   
    tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div42;    
  }
  /* Write to ETHERNET MAC MIIAR: Configure the ETHERNET CSR Clock Range */
  ETH->MACMIIAR = (uint32_t)tmpreg;  
  
  /*-------------------- PHY initialization and configuration ----------------*/
  /* Put the PHY in reset mode */
  if(!(ETH_WritePHYRegister(PHY_ADDRESS, PHY_BCR, PHY_Reset))) return ETH_ERROR;
  /* Delay to assure PHY reset */
  _eth_delay_(PHY_ResetDelay);
  
  if(!(ETH_WritePHYRegister(PHY_ADDRESS, PHY_MISR, PHY_LINK_INT_EN | PHY_ANC_INT_EN ))) 	return ETH_ERROR;
  RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, PHY_RBR);
  if(!(ETH_WritePHYRegister(PHY_ADDRESS, PHY_RBR, (RegValue&~(0x000F<<9)) | (0x0000<<9) ))) return ETH_ERROR;
  if(!(ETH_WritePHYRegister(PHY_ADDRESS, PHY_MICR, PHY_INTEN | PHY_INT_OE ))) 				return ETH_ERROR;

  /* Put the PHY in reset mode */
  if(!(ETH_WritePHYRegister(PHY_ADDRESS1, PHY_BCR, PHY_Reset))) return ETH_ERROR;
  /* Delay to assure PHY reset */
  _eth_delay_(PHY_ResetDelay);

  if(!(ETH_WritePHYRegister(PHY_ADDRESS1,PHY_MISR, PHY_LINK_INT_EN | PHY_ANC_INT_EN )))		return ETH_ERROR;
  RegValue = ETH_ReadPHYRegister(PHY_ADDRESS1, PHY_RBR);
  if(!(ETH_WritePHYRegister(PHY_ADDRESS1,PHY_RBR, (RegValue&~(0x000F<<9)) | (0x0000<<9) ))) return ETH_ERROR;
  if(!(ETH_WritePHYRegister(PHY_ADDRESS1,PHY_MICR, PHY_INTEN | PHY_INT_OE ))) 				return ETH_ERROR;

  ETH_MAC_Config();
  
  return ETH_SUCCESS;

}



/**
  * @brief  Initializes the ETHERNET peripheral according to the specified
  *   parameters in the ETH_InitStructure .
  * @param ETH_InitStruct: pointer to a ETH_InitTypeDef structure that contains
  *   the configuration information for the specified ETHERNET peripheral.
  * @param PHYAddress: external PHY address                    
  * @retval ETH_ERROR: Ethernet initialization failed
  *         ETH_SUCCESS: Ethernet successfully initialized                 
  */
uint32_t ETH_Init_Interrupt(uint16_t PHYAddress)
{
  uint32_t RegValue 	= 0,MISR_Value 	= 0;
  uint32_t STSValue0 	= 0,STSValue1 	= 0;
  uint32_t RBRValue0 	= 0,RBRValue1 	= 0;
  uint32_t PHYAddr0 	= 0,PHYAddr1 	= 0;
  
  MISR_Value = ETH_ReadPHYRegister(PHYAddress, PHY_MISR);
  if ( MISR_Value & PHY_LINK_INT ){
    if ( (PHYAddress % 2) == 0 ){
      PHYAddr0 = PHYAddress;	PHYAddr1 = PHYAddress+1;
	} else {
	  PHYAddr0 = PHYAddress-1;	PHYAddr1 = PHYAddress;
	}
    STSValue0 = ETH_ReadPHYRegister(PHYAddr0, PHY_STS);
    STSValue1 = ETH_ReadPHYRegister(PHYAddr1, PHY_STS);
    if ( !(STSValue1 & PHY_LINK_STATUS) ){
	//  RBRValue0 = 0x000C;RBRValue1 = 0x0009;
	  RBRValue0 = 0x0000;RBRValue1 = 0x0000;
      RegValue = ETH_ReadPHYRegister(PHYAddr0, PHY_RBR);
      ETH_WritePHYRegister(PHYAddr0, PHY_RBR, (RegValue&~(0x000F<<9)) | (RBRValue0<<9) );
      RegValue = ETH_ReadPHYRegister(PHYAddr1, PHY_RBR);
      ETH_WritePHYRegister(PHYAddr1, PHY_RBR, (RegValue&~(0x000F<<9)) | (RBRValue1<<9) );
    } else if ( !(STSValue0 & PHY_LINK_STATUS) ){
	//  RBRValue0 = 0x000C;RBRValue1 = 0x0009;
	  RBRValue0 = 0x0005;RBRValue1 = 0x0005;
      RegValue = ETH_ReadPHYRegister(PHYAddr0, PHY_RBR);
      ETH_WritePHYRegister(PHYAddr0, PHY_RBR, (RegValue&~(0x000F<<9)) | (RBRValue0<<9) );
      RegValue = ETH_ReadPHYRegister(PHYAddr1, PHY_RBR);
      ETH_WritePHYRegister(PHYAddr1, PHY_RBR, (RegValue&~(0x000F<<9)) | (RBRValue1<<9) );
    }
	if ( STSValue0 & PHY_LINK_STATUS ){
	  if(ETH_InitStructure.ETH_AutoNegotiation != ETH_AutoNegotiation_Disable){
        if(!(ETH_WritePHYRegister(PHYAddr0, PHY_BCR, PHY_AutoNegotiation))) return ETH_ERROR;
      } else {
        if(!ETH_WritePHYRegister(PHYAddr0, PHY_BCR, ((uint16_t)(ETH_InitStructure.ETH_Mode >> 3) |
                                                    (uint16_t)(ETH_InitStructure.ETH_Speed >> 1))))
        return ETH_ERROR;
      }
	}
	if ( STSValue1 & PHY_LINK_STATUS ){
	  if(ETH_InitStructure.ETH_AutoNegotiation != ETH_AutoNegotiation_Disable){
        if(!(ETH_WritePHYRegister(PHYAddr1, PHY_BCR, PHY_AutoNegotiation))) return ETH_ERROR;
      } else {
        if(!ETH_WritePHYRegister(PHYAddr1, PHY_BCR, ((uint16_t)(ETH_InitStructure.ETH_Mode >> 3) |
                                                    (uint16_t)(ETH_InitStructure.ETH_Speed >> 1))))
        return ETH_ERROR;
      }
		}
  }
  if ( MISR_Value & PHY_ANC_INT ){
    /* Read the result of the autonegotiation */
    RegValue = ETH_ReadPHYRegister(PHYAddress, PHY_SR);
  
    /* Configure the MAC with the Duplex Mode fixed by the autonegotiation process */
    if((RegValue & PHY_Duplex_Status) != (uint32_t)RESET)
      /* Set Ethernet duplex mode to FullDuplex following the autonegotiation */
      ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
    else
      /* Set Ethernet duplex mode to HalfDuplex following the autonegotiation */
      ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;           
    /* Configure the MAC with the speed fixed by the autonegotiation process */
    if(RegValue & PHY_Speed_Status)
      /* Set Ethernet speed to 10M following the autonegotiation */    
      ETH_InitStructure.ETH_Speed = ETH_Speed_10M; 
    else
      /* Set Ethernet speed to 100M following the autonegotiation */ 
      ETH_InitStructure.ETH_Speed = ETH_Speed_100M;  
		ETH_MAC_Config();    
  }
  return ETH_SUCCESS;
}


/**
  * @brief  Configures the Ethernet Interface
  * @param  None
  * @retval None
  */
void Ethernet_Configuration(void)
{

  /* MII/RMII Media interface selection ------------------------------------------*/
#ifdef MII_MODE /* Mode MII with STM3210C-EVAL  */
  GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_MII);

  /* Get HSE clock = 25MHz on PA8 pin (MCO) */
  RCC_MCOConfig(RCC_MCO_HSE);

#elif defined RMII_MODE  /* Mode RMII with STM3210C-EVAL */
  GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);

  /* Set PLL3 clock output to 50MHz (25MHz /5 *10 =50MHz) */
  RCC_PLL3Config(RCC_PLL3Mul_10);
  /* Enable PLL3 */
  RCC_PLL3Cmd(ENABLE);
  /* Wait till PLL3 is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET)
  {}

  /* Get PLL3 clock on PA8 pin (MCO) */
  RCC_MCOConfig(RCC_MCO_PLL3CLK);
#endif

  /* Reset ETHERNET on AHB Bus */
  ETH_DeInit();

  /* Software reset */
  ETH_SoftwareReset();

  /* Wait for software reset */
  while (ETH_GetSoftwareResetStatus() == SET);

  /* ETHERNET Configuration ------------------------------------------------------*/
  /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
  ETH_StructInit(&ETH_InitStructure);

  /* Fill ETH_InitStructure parametrs */
  /*------------------------   MAC   -----------------------------------*/
  ETH_InitStructure.ETH_AutoNegotiation 		= ETH_AutoNegotiation_Enable  ;
  ETH_InitStructure.ETH_LoopbackMode 			= ETH_LoopbackMode_Disable;
  ETH_InitStructure.ETH_RetryTransmission 		= ETH_RetryTransmission_Disable;
  ETH_InitStructure.ETH_AutomaticPadCRCStrip	= ETH_AutomaticPadCRCStrip_Disable;
  ETH_InitStructure.ETH_ReceiveAll 				= ETH_ReceiveAll_Disable;
  ETH_InitStructure.ETH_BroadcastFramesReception= ETH_BroadcastFramesReception_Enable;
  ETH_InitStructure.ETH_PromiscuousMode 		= ETH_PromiscuousMode_Disable;
  ETH_InitStructure.ETH_MulticastFramesFilter 	= ETH_MulticastFramesFilter_Perfect;
  ETH_InitStructure.ETH_UnicastFramesFilter 	= ETH_UnicastFramesFilter_Perfect;
#ifdef CHECKSUM_BY_HARDWARE
  ETH_InitStructure.ETH_ChecksumOffload 		= ETH_ChecksumOffload_Enable;
#endif

  /*------------------------   DMA   -----------------------------------*/  
  
  /* When we use the Checksum offload feature, we need to enable the Store and Forward mode: 
  the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum, 
  if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
  ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable; 
  ETH_InitStructure.ETH_ReceiveStoreForward 		= ETH_ReceiveStoreForward_Enable;         
  ETH_InitStructure.ETH_TransmitStoreForward 		= ETH_TransmitStoreForward_Enable;     
 
  ETH_InitStructure.ETH_ForwardErrorFrames 			= ETH_ForwardErrorFrames_Disable;       
  ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;   
  ETH_InitStructure.ETH_SecondFrameOperate 			= ETH_SecondFrameOperate_Enable;                                                          
  ETH_InitStructure.ETH_AddressAlignedBeats 		= ETH_AddressAlignedBeats_Enable;      
  ETH_InitStructure.ETH_FixedBurst 					= ETH_FixedBurst_Enable;                
  ETH_InitStructure.ETH_RxDMABurstLength 			= ETH_RxDMABurstLength_32Beat;          
  ETH_InitStructure.ETH_TxDMABurstLength 			= ETH_TxDMABurstLength_32Beat;                                                                 
  ETH_InitStructure.ETH_DMAArbitration 				= ETH_DMAArbitration_RoundRobin_RxTx_2_1;

  /* Configure Ethernet */
  ETH_Init_PHY();

  /* Enable the Ethernet Rx Interrupt */
  ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R, ENABLE);

}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void PHY_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(	RCC_APB2Periph_AFIO  | 	\
  							RCC_APB2Periph_GPIOA | 	\
  							RCC_APB2Periph_GPIOB | 	\
  							RCC_APB2Periph_GPIOC | 	\
							RCC_APB2Periph_GPIOD, ENABLE);
							
  /* Configure PD11 as output */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_RESET);

  /* ETHERNET pins configuration */
  /* AF Output Push Pull:
  - ETH_MII_MDIO / ETH_RMII_MDIO: PA2
  - ETH_MII_MDC / ETH_RMII_MDC: PC1
  - ETH_MII_TX_EN / ETH_RMII_TX_EN: PB11
  - ETH_MII_TXD0 / ETH_RMII_TXD0: PB12
  - ETH_MII_TXD1 / ETH_RMII_TXD1: PB13*/

  /* Configure PA2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PC1 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PB11, PB12 and PB13 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /**************************************************************/
  /*               For Remapped Ethernet pins                   */
  /*************************************************************/
  /* Input (Reset Value):
  - ETH_MII_RX_CLK / ETH_RMII_REF_CLK: PA1
  - ETH_MII_RX_DV / ETH_RMII_CRS_DV: PD8
  - ETH_MII_RXD0 / ETH_RMII_RXD0: PD9
  - ETH_MII_RXD1 / ETH_RMII_RXD1: PD10*/

  /* ETHERNET pins remapp in STM3210C-EVAL board: RX_DV and RxD[3:0] */
  GPIO_PinRemapConfig(GPIO_Remap_ETH, ENABLE);

  /* Configure PA1 as input */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PD8, PD9, PD10 as input */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* MCO pin configuration------------------------------------------------- */
  /* Configure MCO (PA8) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PB14 and PB15 as IPU */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_SET);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void PHY_NVIC_Configuration(void)
{
  NVIC_InitTypeDef	NVIC_InitStructure;
  EXTI_InitTypeDef 	EXTI_InitStructure;

  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  
  /* Enable the Ethernet global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel 					= ETH_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
#if 1  
  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel 					= EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Connect EXTI Line to GPIO Pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);  

  /* Configure EXTI line */
  EXTI_InitStructure.EXTI_Line 		= EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger 	= EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Connect EXTI Line to GPIO Pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);

  /* Configure EXTI line */
  EXTI_InitStructure.EXTI_Line 		= EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger 	= EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
#endif
}


/**
  * @brief  Setup STM32 system (clocks, Ethernet, GPIO, NVIC) and STM3210C-EVAL resources.
  * @param  None
  * @retval None
  */
void System_Setup(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */ 

  /* Enable ETHERNET clock  */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx |
                        RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

  /* Configure the GPIO ports */
  PHY_GPIO_Configuration();

  /* Configure the Ethernet peripheral */
  Ethernet_Configuration();

  /* NVIC configuration */
  PHY_NVIC_Configuration();

  /* SystTick configuration: an interrupt every 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

  /* Update the SysTick IRQ priority should be higher than the Ethernet IRQ */
  /* The Localtime should be updated during the Ethernet packets processing */
  NVIC_SetPriority (SysTick_IRQn, 1);  
 
  // setup led
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);

}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
