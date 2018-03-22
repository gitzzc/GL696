#ifndef __CONFIG_H__
#define __CONFIG_H__


//---------------------------------------------------------------
#define CONFIG_PAGE_SIZE 	1024
#define CONFIG_PAGE1		126
#define CONFIG_PAGE2		(CONFIG_PAGE1+1)
#define CONFIG_ADDRESS1		((uint32_t)0x08000000 + CONFIG_PAGE_SIZE*CONFIG_PAGE1)
#define CONFIG_ADDRESS2		((uint32_t)0x08000000 + CONFIG_PAGE_SIZE*CONFIG_PAGE2)

//---------------------------------------------------------------
uint32_t ConfigRead (uint32_t address,uint8_t* cfg,uint32_t len);
uint32_t ConfigWrite(uint32_t address,uint8_t* cfg,uint32_t len);



#endif
