#include "at32f4xx.h"
#include "gpio.h"
#include "at32_board.h"
#include "key.h"
#include "stdio.h"
#include "mcfoc.h"
#include "mclib.h"
#include "math.h"
#include "mcctl.h"
#include "mcuart.h"
#include "protect.h"
#include "logic.h"
#include "fangbo.h"
#include "youshua.h"
#include "YSstatemachine.h"
#include "YSlogic.h"
#include "YSmcctl.h"
#include "YSprotect.h"



__IO uint32_t LsiFreq = 1000;

#if use_HSI == 1//内部时钟
#include "PLLconfig.h"
#endif

int main(void){
	
	#if use_HSI == 1 //内部时钟
	NewPLLconfig();
	SystemCoreClockUpdate();
	#endif
	Hard_Init();		//硬件初始化
	Soft_Init();		//软件初始化
	
	IWDG_KeyRegWrite(IWDG_KeyRegWrite_Enable);//解锁看门狗
	IWDG_SetPrescaler(IWDG_Psc_32);						//配置看门狗预分频值
	IWDG_SetReload(LsiFreq);									//配置看门狗重装载值
	IWDG_ReloadCounter();											//重新加载重装载值
	IWDG_Enable();														//使能看门狗

	while(1){
		SystemProtect();				//系统保护函数：统一的过压、欠压、过温
		if(YSorWSFlag == 1 ){
			Calculate();					//无刷计算函数
			Motor_Protect();			//电机保护函数
		}else if(YSorWSFlag == 2){
			YSMotor_Protect();		//有刷保护函数
		}
		UartDealResponse();			//串口指令接收及解析
		IWDG_ReloadCounter();		//看门狗喂狗	
	}
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif
