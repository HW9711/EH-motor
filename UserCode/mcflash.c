#include  "userpara.h"
#include "at32f4xx.h"
#include "mcflash.h"
#include "mcctl.h"
#include "at32_board.h"

//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
u16 FLASH_ReadHalfWord(u32 faddr){
	return *(vu16*)faddr; 
}


//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数   
void FLASH_Write_NoCheck(u32 WriteAddr,__IO u16 *pBuffer,u16 NumToWrite){
	u16 i;
	for(i = 0;i < NumToWrite;i ++){
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
		WriteAddr+=2;//地址增加2.
	}  
} 

//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
void FLASH_Write(u32 WriteAddr,__IO u16 *pBuffer,u16 NumToWrite){
	uint8_t i = 0,j = 0;
	u32 addr = 0;
	uint8_t savesta = 0;
	
	__disable_irq();					//关闭所有中断
	FLASH_Unlock();						//解锁
	
	for(j = 0;j < 3;j ++){			//最多尝试储存3次，如果超过3次储存还是错误的，就擦除扇区，报错。
		
		addr = WriteAddr;         //保存地址
		FLASH_ErasePage(addr);		//擦除这个扇区
		
		FLASH_ProgramHalfWord(addr,0xAADD);//首地址写入0xAADD
		
		if(FLASH_ReadHalfWord(addr) != 0xAADD){
			savesta = 1;//储存错误
		}else{//首地址储存成功
			addr += 2;//地址增加2
			for(i = 0;i < NumToWrite;i ++){		 //写入需要保存的数据 
				FLASH_ProgramHalfWord(addr,pBuffer[i]);
				if(FLASH_ReadHalfWord(addr) != pBuffer[i]){
					savesta = 1;//储存错误
					break;//退出本次Foc循环
				}
				addr += 2;//地址增加2
			}
		}
		
		if(savesta == 0){//保存成功
			break;//退出for循环
		}else if(savesta == 1 && j >= 2){
			App.FB.Err = E_SAVE;//储存错误
//			App.BitErr |= ERR_XT_Save_MCPara;//保存错误
		}
	}
	
	//如果储存错误，就擦除扇区
	if(App.FB.Err == E_SAVE){
		FLASH_ErasePage(WriteAddr);//擦除扇区
	}
	
	FLASH_Lock();//上锁
	
	__enable_irq();//打开所有中断
}

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void FLASH_Read(u32 ReadAddr,__IO uint16_t *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i = 0;i < NumToRead;i ++){
		pBuffer[i] = FLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr += 2;//偏移2个字节.	
	}
}

//读FLAS函数
void F4Flash_SMCPara_Read(u32 WriteAddr,__IO u16 *pBuffer,u16 NumToWrite,__IO u16 *pSBuffer){//初始化读
	uint16_t i;
	if(FLASH_ReadHalfWord(WriteAddr) == 0xAADD){
		FLASH_Read(WriteAddr + 2,pBuffer,NumToWrite);
  }else{
		for(i = 0;i < ParaNum;i ++){
			pBuffer[i] = pSBuffer[i];
		}
  }
}


//FLASH擦除扇区函数
void F4Flash_SMCPara_Erase(uint32_t adrr){//擦除参数
	FLASH_Unlock();						//解锁
	FLASH_ErasePage(adrr);//擦除这个扇区
	FLASH_Lock();//上锁
}


