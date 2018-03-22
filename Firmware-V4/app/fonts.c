/**
  ******************************************************************************
  * @file    fonts.c
  * @author  MCD Application Team
  * @version V4.6.1
  * @date    18-April-2011
  * @brief   This file provides text fonts for STM32xx-EVAL's LCD driver. 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "fonts.h"

#include "ascii0816.h"
#include "ascii1020.h"
#include "ascii1224.h"
#include "ascii1632.h"
#include "hzk1616.h"
#include "hzk2020.h"
#include "hzk2424.h"
#include "hzk3232.h"


/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup Common
  * @{
  */

/** @addtogroup FONTS
  * @brief      This file includes the Fonts driver of STM32-EVAL boards.
  * @{
  */  

/** @defgroup FONTS_Private_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup FONTS_Private_Defines
  * @{
  */
/**
  * @}
  */ 


/** @defgroup FONTS_Private_Macros
  * @{
  */
/**
  * @}
  */ 
  

/** @defgroup FONTS_Private_Variables
  * @{
  */

sFONT EN_Font8x16 = {
  nAsciiDot08x16,
  8, /* Width */
  16, /* Height */
};

sFONT EN_Font10x20 = {
  nAsciiDot10x20,
  10, /* Width */
  20, /* Height */
};
   
sFONT EN_Font12x24 = {
  nAsciiDot12x24,
  12, /* Width */
  24, /* Height */
};
   
sFONT EN_Font16x32 = {
  nAsciiDot16x32,
  16, /* Width */
  32, /* Height */
};   

sFONT CH_Font16x16 = {
  (const uint8_t*)GB_16,
  16, /* Width */
  16, /* Height */
};

sFONT CH_Font20x20 = {
  (const uint8_t*)GB_20,
  20, /* Width */
  20, /* Height */
};

sFONT CH_Font24x24 = {
  (const uint8_t*)GB_24,
  24, /* Width */
  24, /* Height */
};

sFONT CH_Font32x32 = {
  (const uint8_t*)GB_32,
  32, /* Width */
  32, /* Height */
};


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
