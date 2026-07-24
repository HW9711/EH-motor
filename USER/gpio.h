#ifndef  __GPIO_H 
#define  __GPIO_H
#include "at32f4xx_gpio.h"
#include  "userpara.h"
/*位带操作,实现51类似的GPIO控制功能IO口操作宏定义*/ 
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入



//状态指示灯 status led
#define	_STA_LED									1
#define STA_LED_PIN               GPIO_Pins_9
#define STA_LED_GPIO              GPIOB
#define STA_LED                   PBout(9)

#define STA_LED_INIT_STA          Bit_RESET
#define STALED_OFF                STA_LED = 1
#define STALED_ON                 STA_LED = 0
#define STALED_ON_OFF             STA_LED = !STA_LED


////开电容
//#define EN_CAP_ON                 0//1-开电容 0-关电容
//#if 		EN_CAP_ON == 1
//#define EN_CAP_PIN                GPIO_Pins_4
//#define EN_CAP_GPIO               GPIOC
//#define EN_CAP_INIT_STA           Bit_RESET
//#define EN_CAP                    PCout(4)	
//#endif


//按键初始化
//使能
#define key_run										0
#define KEYRUN_PIN                GPIO_Pins_5
#define KEYRUN_GPIO               GPIOB
#define KAYRUN                    PBin(5)

//方向
#define key_dir										0
#define KEYDIR_PIN                GPIO_Pins_4
#define KEYDIR_GPIO               GPIOB
#define KAYDIR                    PBin(4)

//刹车
#define key_brake									0
#define KEYBRA_PIN                GPIO_Pins_13
#define KEYBRA_GPIO               GPIOC
#define KAYBRA                    PCin(13)

//学习按键
#define key_HallSdy								0
#define KEYStady_PIN              GPIO_Pins_0
#define KEYStady_GPIO             GPIOD
#define KAYStady                  PDin(0)

//SW1
#define Key_SW1										0
#define KEYSW1_PIN                GPIO_Pins_3
#define KEYSW1_GPIO               GPIOC
#define KAYSW1                    PCin(3)

//SW2
#define Key_SW2										0
#define KEYSW2_PIN                GPIO_Pins_2
#define KEYSW2_GPIO               GPIOC
#define KAYSW2                    PCin(2)

//SW3
#define Key_SW3										0
#define KEYSW3_PIN                GPIO_Pins_14
#define KEYSW3_GPIO               GPIOC
#define KAYSW3                    PCin(14)

//FG输出信号
#define _FG												0
#define FG_PIN                		GPIO_Pins_15
#define FG_GPIO               		GPIOA
#define FGOUT                 		PAout(15)

//异常输出信号
#define _ALARM										0
#define ALARM_PIN                	GPIO_Pins_3
#define ALARM_GPIO               	GPIOB
#define ALARM                    	PBout(3)


//风扇
#define _FAN											0
#define FAN_PIN                		GPIO_Pins_1
#define FAN_GPIO               		GPIOC
#define FAN                    		PCout(1)



//hall
#define _HALL											1
//#define HALLA_PIN                	GPIO_Pins_6
//#define HALLA_GPIO               	GPIOB
//#define HALLB_PIN                	GPIO_Pins_7
//#define HALLB_GPIO               	GPIOB
//#define HALLC_PIN                	GPIO_Pins_8
//#define HALLC_GPIO               	GPIOB
//#define gethall                  	((GPIOB->IPTDT) >> 6 ) & 0x07

//#define HALLA_PIN                GPIO_Pins_0
//#define HALLA_GPIO               GPIOA
//#define HALLB_PIN                GPIO_Pins_1
//#define HALLB_GPIO               GPIOA
//#define HALLC_PIN                GPIO_Pins_2
//#define HALLC_GPIO               GPIOA
//#define gethall                  ((GPIOA->IPTDT) >> 0 ) & 0x07

#define HALLA_PIN                	GPIO_Pins_10
#define HALLA_GPIO               	GPIOC
#define HALLB_PIN                	GPIO_Pins_11
#define HALLB_GPIO               	GPIOC
#define HALLC_PIN                	GPIO_Pins_12
#define HALLC_GPIO               	GPIOC

#define gethall                  	(PCin(10) + PCin(11) * 2 + PCin(12) * 4)//((GPIOB->IPTDT) >> 6 ) & 0x07




//hall
#define _HALL2											1

#define HALLA2_PIN                	GPIO_Pins_12
#define HALLA2_GPIO               	GPIOA

#define HALLB2_PIN                	GPIO_Pins_11
#define HALLB2_GPIO               	GPIOA

#define HALLC2_PIN                	GPIO_Pins_2
#define HALLC2_GPIO               	GPIOC




//更改了PCB，飞线！
#define gethall2                  	(PAin(12) + PAin(11) * 2 + PCin(2) * 4)//((GPIOB->IPTDT) >> 6 ) & 0x07

//无感方波过零检测
#define EN1A_PIN					GPIO_Pins_4
#define	EN1A_GPIO					GPIOB

#define EN1B_PIN					GPIO_Pins_5
#define EN1B_GPIO					GPIOB

#define EN1Z_PIN					GPIO_Pins_3
#define EN1Z_GPIO					GPIOB			

#define EN2A_PIN					GPIO_Pins_6
#define EN2A_GPIO					GPIOB

#define EN2B_PIN					GPIO_Pins_7
#define EN2B_GPIO					GPIOB

#define EN2Z_PIN					GPIO_Pins_8
#define EN2Z_GPIO					GPIOB


#define get_Efhall1					( PBin(4) + PBin(5) * 2 + PBin(3)*4 )

#define get_Efhall2					( PBin(6) + PBin(7) * 2 + PBin(8)*4 )

#define get_EN1A						PBin(4)
#define get_EN1B						PBin(5)
#define get_EN1Z						PBin(3)

#define get_EN2A						PBin(6)
#define get_EN2B						PBin(7)
#define get_EN2Z						PBin(8)


#define Hall_1_EXTI_				1
				





#define _IPM												0
#define IPM_PIN											GPIO_Pins_9
#define IPM_GPIO										GPIOC

#define IPM_GPIOSource							GPIO_PortSourceGPIOC
#define IPM_PINSource								GPIO_PinsSource9

#define IPM_Exti_Line 							EXTI_Line9
#define IPM_NVIC_Channel						EXTI9_5_IRQn 
#define IPM_EXTI_IRQHandler					EXTI9_5_IRQHandler


#define _ENZ												0
#define ENZ_PIN											GPIO_Pins_3
#define ENZ_GPIO										GPIOB

#define ENZ_GPIOSource							GPIO_PortSourceGPIOB
#define ENZ_PINSource								GPIO_PinsSource3

#define ENZ_Exti_Line 							EXTI_Line3
#define ENZ_NVIC_Channel						EXTI3_IRQn 
#define ENZ_EXTI_IRQHandler					EXTI3_IRQHandler


#define _ENZ2												0
#define ENZ2_PIN										GPIO_Pins_8
#define ENZ2_GPIO										GPIOB

#define ENZ2_GPIOSource							GPIO_PortSourceGPIOB
#define ENZ2_PINSource							GPIO_PinsSource8

#define ENZ2_Exti_Line 							EXTI_Line8
#define ENZ2_NVIC_Channel						EXTI9_5_IRQn 
#define ENZ2_EXTI_IRQHandler				EXTI9_5_IRQHandler



//接近开关
#define _PrsIN												0
#define PrsIN_PIN											GPIO_Pins_13
#define PrsIN_GPIO										GPIOC
#define Read_PrsIN										PCin(13)

#define PrsIN_GPIOSource							GPIO_PortSourceGPIOC
#define PrsIN_PINSource								GPIO_PinsSource13

#define PrsIN_Exti_Line 							EXTI_Line13
#define PrsIN_NVIC_Channel						EXTI15_10_IRQn 
#define PrsIN_EXTI_IRQHandler					EXTI15_10_IRQHandler


#define _Relay 												0
#define Relay_PIN               			GPIO_Pins_9
#define Relay_GPIO              			GPIOB
#define Relay                   			PBout(9)

#define Relay_INIT_STA          			Bit_RESET
#define Relay_OFF                			Relay = 0
#define Relay_ON                 			Relay = 1


#define _PVCC_PWM_EN						1
#define PVCC_PWM_EN_PIN						GPIO_Pins_15
#define PVCC_PWM_EN_GPIO					GPIOA
#define	PVCC_PWM							PAout(15)

#define PVCC_PWM_INIT_STA					Bit_RESET		//初始化状态先关了把
#define PVCC_PWM_ON							PVCC_PWM = 1	//高电平开启
#define PVCC_PWM_OFF						PVCC_PWM = 0 	//低电平关闭


void HALL1_GPIO_INIT(void);
void HALL2_GPIO_INIT(void);
void HALL1_GPIO_INIT_Disable(void);
void HALL2_GPIO_INIT_Disable(void);


void GPIO_Init_App(void);
void GPIO_Noninductive1_Init(void);
void GPIO_Noninductive2_Init(void);
void GPIO_Brake_Init(void);

#endif 
