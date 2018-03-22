#include <stdint.h>

/* Library includes. */
#include "stm32f10x.h"

#include "projdefs.h"
#include "config.h"

uint32_t ConfigRead(uint32_t address,uint8_t* cfg,uint32_t len)
{
	uint32_t i,ret,index;
	uint32_t *buf32 = (uint32_t*)cfg;
	uint32_t temp32;
	
	if ( len + 4 > CONFIG_PAGE_SIZE ) 
		return pdFALSE;

	/* Enable CRC clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	
	CRC_ResetDR();
	for(index=0;index<len/4;index++){
		buf32[index] = *(__IO uint32_t*) (address + index*4);
		CRC_CalcCRC(buf32[index]);
	}
	if ( len%4 ){
		temp32 = *(__IO uint32_t*) (address + index*4);
		CRC_CalcCRC(temp32);
		for(i=0;i<len%4;i++)
			cfg[index*4+i] = (temp32>>(i*8))&0xFF;
		index ++;
	}

	if ( CRC_CalcCRC(*(__IO uint32_t*) (address + index*4)) == 0 )
		ret = pdTRUE;
	else 
		ret = pdFALSE;

	/* Disable CRC clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, DISABLE);

	return ret;
}

uint32_t ConfigWrite(uint32_t address,uint8_t* cfg,uint32_t len)
{
	FLASH_Status FLASHStatus;
	uint32_t i,index;
	uint32_t *buf32 = (uint32_t*)cfg;
	uint32_t temp32;
	
	if ( len + 4 > CONFIG_PAGE_SIZE ) 
		return pdFALSE;
		
	/* Enable CRC clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	//----------------------------------------------------------------------
	/* Unlock the Flash Bank1 Program Erase controller */
	FLASH_UnlockBank1();
	/* Clear All pending flags */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	/* Erase the FLASH pages */
	FLASHStatus = FLASH_ErasePage(address);
	/* Program Flash PAGE1 */
	index = 0;
	CRC_ResetDR();
	while((index < len/4) && (FLASHStatus == FLASH_COMPLETE))
	{
		FLASHStatus = FLASH_ProgramWord(address + index*4, buf32[index]);
		CRC_CalcCRC(buf32[index]);
		index ++;
	}
	if ( len%4 ){
		for(temp32=0,i=0;i<len%4;i++)
			temp32 |= cfg[index*4 + i] << (i*8);
		FLASHStatus = FLASH_ProgramWord(address + index*4, temp32);
		CRC_CalcCRC(temp32);
		index ++;
	}
	FLASHStatus = FLASH_ProgramWord(address + index*4, CRC_GetCRC());
	FLASH_LockBank1();
	//----------------------------------------------------------------------

	/* Disable CRC clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, DISABLE);
	
	if ( FLASHStatus != FLASH_COMPLETE ) 
		return pdFALSE;
	return pdTRUE;
}

