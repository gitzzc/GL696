#ifndef __CONFIG_H__
#define __CONFIG_H__


//---------------------------------------------------------------
#define CONFIG_PAGE_SIZE 	2048 
#define CONFIG_PAGE1		126
#define CONFIG_PAGE2		(CONFIG_PAGE1+1)
#define CONFIG_ADDRESS1		((uint32_t)0x08000000 + CONFIG_PAGE_SIZE*CONFIG_PAGE1)
#define CONFIG_ADDRESS2		((uint32_t)0x08000000 + CONFIG_PAGE_SIZE*CONFIG_PAGE2)

//---------------------------------------------------------------
uint32_t ConfigReadBlock (uint32_t address,uint8_t* cfg,uint32_t len);
uint32_t ConfigWriteBlock(uint32_t address,uint8_t* cfg,uint32_t len);
uint32_t ConfigWrite(sSYS_CFG* cfg,const sSYS_CFG* def_cfg);
uint32_t ConfigRead (sSYS_CFG* cfg,const sSYS_CFG* def_cfg);



#endif
