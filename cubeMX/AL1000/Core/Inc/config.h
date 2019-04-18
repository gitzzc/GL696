#ifndef __CONFIG_H__
#define __CONFIG_H__


//---------------------------------------------------------------
#define CONFIG_SIZE 		FLASH_PAGE_SIZE
#define CONFIG_PAGE1		62
#define CONFIG_PAGE2		(CONFIG_PAGE1+CONFIG_SIZE/FLASH_PAGE_SIZE)
#define CONFIG_ADDRESS1		((uint32_t)0x08000000 + FLASH_PAGE_SIZE*CONFIG_PAGE1)
#define CONFIG_ADDRESS2		((uint32_t)0x08000000 + FLASH_PAGE_SIZE*CONFIG_PAGE2)

//---------------------------------------------------------------
uint32_t ConfigReadBlock (uint32_t address,uint32_t* cfg,uint32_t len);
uint32_t ConfigWriteBlock(uint32_t address,uint32_t* cfg,uint32_t len);
uint8_t ConfigWrite(uint32_t* config,uint32_t len,const uint32_t* def_cfg);
uint8_t ConfigRead(uint32_t* config,uint32_t len,const uint32_t* def_cfg);



#endif
