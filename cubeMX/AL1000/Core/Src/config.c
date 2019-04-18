#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "main.h"
#include "config.h"
#include "flash_if.h"

uint32_t ConfigReadBlock(uint32_t address,uint32_t* cfg,uint32_t len)
{
	CRC_HandleTypeDef   CrcHandle;
	uint32_t i,ret;
	
	if ( len > CONFIG_SIZE )
		return pdFALSE;

	CrcHandle.Instance = CRC;
	HAL_CRC_DeInit(&CrcHandle);
	if (HAL_CRC_Init(&CrcHandle) != HAL_OK) Error_Handler();
	
	for(i=0;i<len;i++){
		cfg[i] = *(__IO uint32_t*) (address + i*4);
	}

	if ( HAL_CRC_Calculate(&CrcHandle, (uint32_t *)cfg, len) == 0 )
		ret = pdTRUE;
	else 
		ret = pdFALSE;

	HAL_CRC_DeInit(&CrcHandle);
	return ret;
}

uint32_t ConfigWriteBlock(uint32_t address,uint32_t* cfg,uint32_t len)
{
	CRC_HandleTypeDef   CrcHandle;

	if ( len > CONFIG_SIZE )
		return pdFALSE;

	FLASH_If_Init();
	if ( FLASH_If_Erase(address,CONFIG_SIZE) != FLASHIF_OK ) Error_Handler();

	CrcHandle.Instance = CRC;
	HAL_CRC_DeInit(&CrcHandle);
	if (HAL_CRC_Init(&CrcHandle) != HAL_OK) Error_Handler();
	cfg[len-1] = HAL_CRC_Calculate(&CrcHandle, cfg, len-1);
	HAL_CRC_DeInit(&CrcHandle);

	if ( FLASH_If_Write(address,cfg,len) != FLASHIF_OK ) Error_Handler();

	return pdTRUE;
}

uint8_t ConfigWrite(uint32_t* config,uint32_t len,const uint32_t* def_cfg)
{
	uint8_t ret;

	if ( len > CONFIG_SIZE ) return pdFALSE;

	ret = ConfigWriteBlock(CONFIG_ADDRESS1,config,len);
	if ( (ConfigWriteBlock(CONFIG_ADDRESS2,config,len) == pdFALSE) && (ret == pdFALSE) )
		ret = pdFALSE;
	return ret;
}

uint8_t ConfigRead(uint32_t* config,uint32_t len,const uint32_t* def_cfg)
{
	uint8_t ret1,ret2;
	
	ret1 = ConfigReadBlock(CONFIG_ADDRESS1,config,len);
	ret2 = ConfigReadBlock(CONFIG_ADDRESS2,config,len);
	if ( ret1 == pdFALSE && ret2 == pdFALSE ){
		memcpy(config,def_cfg,len*4);
		ret1 = ConfigWrite(config,len,def_cfg);
	} else if ( ret1 == pdFALSE ){
		ret1 = ConfigWriteBlock(CONFIG_ADDRESS1,config,len);
	} else if ( ret2 == pdFALSE ){
		ret1 = ConfigReadBlock (CONFIG_ADDRESS1,config,len);
		if ( ret1 == pdTRUE )
			ret1 = ConfigWriteBlock(CONFIG_ADDRESS2,config,len);
	}
	return ret1;
}
