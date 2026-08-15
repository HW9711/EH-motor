#ifndef __MCFLASH_H__
#define __MCFLASH_H__
#include "userpara.h"
#include "at32f4xx.h"

void F4Flash_SMCPara_Read(u32 WriteAddr,__IO u16 *pBuffer,u16 NumToWrite,__IO u16 *pSBuffer);   
void FLASH_Write(u32 WriteAddr,__IO u16 *pBuffer,u16 NumToWrite);
void F4Flash_SMCPara_Erase(uint32_t adrr);


#define TEST_FLASH_ADDRESS_START       	(0x8000000 + 2048*99)
#define TEST_FLASH_ADDRESS_START_HALL  	(0x8000000 + 2048*100)

#define TEST_FLASH_ADDRESS_START2       (0x8000000 + 2048*101)
#define TEST_FLASH_ADDRESS_START_HALL2  (0x8000000 + 2048*102)
#define TEST_FLASH_ADDRESS_START_YS1    (0x8000000 + 2048*103)
#define TEST_FLASH_ADDRESS_START_YS2    (0x8000000 + 2048*104)

#endif /* __STMFLASH_H__ */
