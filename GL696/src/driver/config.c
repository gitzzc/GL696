#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "projdefs.h"
#include "config.h"


uint32_t ConfigReadBlock(uint32_t address,uint8_t* cfg,uint32_t len)
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

uint32_t ConfigWriteBlock(uint32_t address,uint8_t* cfg,uint32_t len)
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

void ConfigInitDate(sSYS_CFG* cfg,const sSYS_CFG* def_cfg)
{
	char *endp;
	uint8_t i;
	char* mon[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
	
	memcpy(cfg,def_cfg,(uint32_t)&def_cfg->key - (uint32_t)def_cfg);

	endp = __DATE__;
	for(i=0;i<12;i++)
		if ( strncmp(endp,mon[i],3) == 0 )
			break;
			
	cfg->date = (i+1%12)*100 + strtol(endp+4,&endp,0);
	cfg->year = strtol(endp+1,&endp,0);
	cfg->hour = strtol(__TIME__,&endp,0);	
	cfg->time = strtol(endp+1,&endp,0);	
	cfg->time = cfg->time*100 + strtol(endp+1,&endp,0);	
}

uint32_t ConfigWrite(sSYS_CFG* cfg,const sSYS_CFG* def_cfg)
{
	uint32_t ret;

	ConfigInitDate(cfg,def_cfg);
	ret = ConfigWriteBlock(CONFIG_ADDRESS1,(uint8_t*)cfg,sizeof(sSYS_CFG)) ;
	if ( (ConfigWriteBlock(CONFIG_ADDRESS2,(uint8_t*)cfg,sizeof(sSYS_CFG)) == pdFALSE) && (ret == pdFALSE) )
		ret = pdFALSE;
	return ret;
}

uint32_t ConfigRead(sSYS_CFG* cfg,const sSYS_CFG* def_cfg)
{
	uint32_t ret1,ret2;
	
	ret1 = ConfigReadBlock(CONFIG_ADDRESS1,(uint8_t*)cfg,sizeof(sSYS_CFG));
	ret2 = ConfigReadBlock(CONFIG_ADDRESS2,(uint8_t*)cfg,sizeof(sSYS_CFG));
	ConfigInitDate(cfg,def_cfg);
	if ( ret1 == pdFALSE && ret2 == pdFALSE ){
		memcpy(cfg,def_cfg,sizeof(sSYS_CFG));
		ret1 = ConfigWrite(cfg,def_cfg);
	} else if ( ret1 == pdFALSE ){
		ret1 = ConfigWriteBlock(CONFIG_ADDRESS1,(uint8_t*)cfg,sizeof(sSYS_CFG));
	} else if ( ret2 == pdFALSE ){
		ret1 = ConfigReadBlock(CONFIG_ADDRESS1,(uint8_t*)cfg,sizeof(sSYS_CFG));
		ConfigInitDate(cfg,def_cfg);
		if ( ret1 == pdTRUE )
			ret1 = ConfigWriteBlock(CONFIG_ADDRESS2,(uint8_t*)cfg,sizeof(sSYS_CFG));
	}
	return ret1;
}
