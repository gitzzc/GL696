/**
  @page STM32F1xx_IAP AN4657 STM32F1xx In-Application Programming using the USART Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    STM32F1xx_IAP/IAP_Main/readme.txt 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    8-April-2015
  * @brief   Description of implementation of the AN4657 (in-application programming
  *          using the USART (IAP)) on STM32F1xx devices.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License
  *   
  ******************************************************************************
  @endverbatim

@par Example Description

This directory contains a set of sources files and pre-configured projects that 
describes how to build an application to be loaded into Flash memory using
In-Application Programming (IAP, through USART).

@par Directory contents

 - "STM32F1xx_IAP/IAP_Main/Inc": contains the IAP firmware header files 
    - STM32F1xx_IAP/IAP_Main/Inc/main.h              The main include file of the project.
    - STM32F1xx_IAP/IAP_Main/Inc/common.h            This file provides all the headers of the common functions.
    - STM32F1xx_IAP/IAP_Main/Inc/flash_if.h          This file provides all the firmware 
                                                     function headers of the flash_if.c file.
    - STM32F1xx_IAP/IAP_Main/Inc/menu.h              This file provides all the firmware
                                                     function headers of the menu.c file.
    - STM32F1xx_IAP/IAP_Main/Inc/ymodem.h            This file provides all the firmware
                                                     function headers of the ymodem.c file.
    - STM32F1xx_IAP/IAP_Main/Inc/stm32f1xx_hal_conf.h  Library Configuration file
    - STM32F1xx_IAP/IAP_Main/Inc/stm32f1xx_it.h      Header for stm32f1xx_it.c

 - "STM32F1xx_IAP/IAP_Main/MDK-ARM": contains pre-configured project for MDK-ARM toolchain

 - "STM32F1xx_IAP/IAP_Main/EWARM": contains pre-configured project for EWARM toolchain

 - "STM32F1xx_IAP/IAP_Main/Src": contains the IAP firmware source files
    - STM32F1xx_IAP/IAP_Main/Src/main.c              Main program
    - STM32F1xx_IAP/IAP_Main/Src/stm32f1xx_it.c      Interrupt handlers
    - STM32F1xx_IAP/IAP_Main/Src/stm32f1xx_hal_msp.c Microcontroller specific packages
                                                     initialization file.
    - STM32F1xx_IAP/IAP_Main/Src/flash_if.c          The file contains write, erase and disable
                                                     write protection of the internal Flash
                                                     memory.
    - STM32F1xx_IAP/IAP_Main/Src/menu.c              This file contains the menu to select
                                                     downloading a new binary file, uploading
                                                     internal Flash memory, executing the binary
                                                     and disabling the write protection of
                                                     write-protected pages
    - STM32F1xx_IAP/IAP_Main/Src/common.c            This file provides functions related to
                                                     read/write from/to USART peripheral
    - STM32F1xx_IAP/IAP_Main/Src/ymodem.c            This file provides all the firmware functions
                                                     related to the ymodem protocol.
    - STM32F1xx_IAP/IAP_Main/Src/system_stm32f1xx.c  STM32F1xx system source file

@par Hardware and Software environment

  - This example runs on STM32F1xx Devices.
  - This example has been tested with STMicroelectronics STM3210C-EVAL RevC evaluation board
    and can be easily tailored to any other supported device and development board.
  
Table 1. IAP implementation on STM3210C-EVAL RevC
/*** Platform ***|************* Implementation **************************|***** Configuration *****\
****************************************************************************************************
|    Firmware    | The IAP program is located at 0x08000000. The Flash   |                         |
|                | routines (program/erase) are executed from the Flash  |                         |
|                | memory.                                               |                         |
|                | The size of this program is about 13 Kbytes and       |                         |
|                | programmed on:                                        | Page 0 - Page 7         |  
|                | ------------------------------------------------------|-------------------------|
|                | The user application (image to be downloaded with the |                         | 
|                | IAP) will be programmed starting from address         |                         |
|                | (uint32_t)0x08004000(1).                              | (Page 8 - Page 127)     | 
|                | The maximum size of the image to be loaded is:        | (240 Kbytes             | 
|                | ------------------------------------------------------|-------------------------|
|                | The image is uploaded with the IAP from the STM32F1xx | 12 Kbytes               | 
|                | internal Flash.                                       | (Page 8 - Page 13)      |
|                | The size of the image to be uploaded is:              |                         |
|----------------|-------------------------------------------------------|-------------------------|
|    Hardware    | Push-button (active level: high)                      | Key push-button         |                                                                     
|                |                                                       | connected to pin PB.09  |
|                | ------------------------------------------------------|-------------------------| 
|                | USART used                                            |  USART2   (CN6)         |
\**************************************************************************************************/ 
(1) User application location address is defined in the flash_if.h file as: 
#define APPLICATION_ADDRESS           ((uint32_t)0x08004000)
To modify it, change the default value to the desired one. Note that the application must be linked
relatively to the new address too.

Following picture illustrates the situation in program memory:
Figure 1. Flash memory usage

 Top Flash Memory address /-------------------------------------------\  0x0803FFFF
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |          Page 8 - Page 127                |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |          User code                        |
                          |                                           |
                          |- - - - - - - - - - - - - - - - - - - - - -|
                          |          Vector table                     |
                          |-------------------------------------------|  0x08004000
                          |          IAP code                         |
                          |- - - - - - - - - - - - - - - - - - - - - -|
                          |          Vector table                     |
                          \-------------------------------------------/	 0x08000000					  
   
  - STM3210C-EVAL RevC Set-up
    - Connect a null-modem female/female RS232 cable between the boards DB9 connector 
      and PC serial port.
      (make sure that jumper JP16 is in position 2-3).
    - Hold the Key push-button during reset to enter the IAP.    

  - Terminal configuration: 
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - BaudRate = 115200 baud
    - flow control: None 
    - Ymodem protocol is using CRC16 by default. To switch to checksum, comment #define CRC16_F
      in ymodem.c

@par How to use it? 

In order to make the program work, you must do the following:

  1. Generate a binary image for the program provided in the 
     "STM32F1xx_IAP/IAP_Binary_Template" project directory. 
  2. Program the internal Flash with the IAP (see below) 
  3. Open HyperTerminal window using the settings already defined in section
     "Hardware and Software environment" 
  4. To run the IAP driver, keep the Key push-button pressed at Reset. 
     The IAP main menu is then displayed on the HyperTerminal window.
  5. To download an application, press 1 and use the Ymodem protocol

In order to load the IAP code, you have do the following:
   - EWARM:
      - Open the Project.eww workspace
      - Rebuild all files: Project->Rebuild all
      - Load project image: Project->Debug
      - Run program: Debug->Go(F5)

   - MDK-ARM:
      - Open the Project.uvproj project
      - Rebuild all files: Project->Rebuild all target files
      - Load project image: Debug->Start/Stop Debug Session
      - Run program: Debug->Run (F5)
      

    
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
