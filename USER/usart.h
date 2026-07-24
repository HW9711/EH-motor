#ifndef  __USART_H 
#define  __USART_H
#include "at32f4xx_usart.h"
#include "gpio.h"
#include  "userpara.h"


#define _AllUart 1//0-屏蔽所有Uart 1-开启Uart

	#if _AllUart == 1
		#define MAX_RXBUF  70
		typedef struct{
			unsigned int  RxCRCH;
			unsigned int  RxCRCL;
			unsigned int  TempH;
			unsigned int  TempL;
			unsigned char SendVa;
			unsigned char SendLen;
			unsigned int  ReAddrL;
			unsigned int  ReAddrH;
			unsigned char Re3num;   //需要返回的数量
			
		uint8_t    R_DATA[MAX_RXBUF];   
		uint8_t    T_DATA[MAX_RXBUF];
		uint8_t    MT_DATA[MAX_RXBUF];
			uint8_t    RxEnd;
			uint16_t   RxLen;
			uint16_t   RxCRC;
			uint16_t   TxCRC;
			uint16_t   CalcCRC;
			uint8_t    Addr;
			uint8_t    FunCode;
			uint8_t    TxLen;
			uint16_t   UpdateRegH;
			uint16_t   UpdateRegL;
			int16_t    UpdatePara;
			uint16_t	 ReadReg;
			
			//主模式下状态管理
			uint8_t MasterSta;
			uint16_t IdleTimCnt;
			uint8_t OverTimFg;
			uint16_t OverTimCnt;
			uint8_t MasterSendOK;
			uint8_t SendErrCnt;
			
			float	tempcurrent;
			uint16_t  tempdragstep;				//拖动步数
		}MCUART_Type;
		
		
		//使能串口1
		#define _Uart1 																0
		#if _Uart1 == 	1
			//0-PA9/PA10 1-PB6/PB7
			#define Uart1_GPIO 													0
			//有485控制脚
			#define _Uart1_RS485_RD 										0
			#if _Uart1_RS485_RD == 1
				#define Uart1_RS485_RD_PIN                GPIO_Pins_12 
				#define Uart1_RS485_RD_GPIO               GPIOC
				#define Uart1_RS485_RD_INIT_STA           Bit_RESET
				#define Uart1_RS485_RD                    PCout(12)	
				#define Uart1_RS485_TX                    Uart1_RS485_RD = 1
				#define Uart1_RS485_RX                    Uart1_RS485_RD = 0
			#endif
			#define Uart1_Rate													115200
			#define Uart1_NVIC_Priority									5
			#define Uart1_DMA_CH												DMA1_Channel5//cal里的DMA弹性表格
			#define Uart1_DMA_Priority									DMA_PRIORITY_HIGH
			#define Uart1_DMA_ADDR											(0x40013800 + 0x04)//寄存器手册48页
			
			extern MCUART_Type SerUart1;
		#endif
		
		
		//使能串口3
		#define _Uart3 																1
		#if _Uart3 == 1
			//0-PB10/PB11 1-PC10/PC11 2-PD8/PD9
			#define Uart3_GPIO 													0
			//有485控制脚
			#define _Uart3_RS485_RD 										0
			#if _Uart3_RS485_RD == 1
				#define Uart3_RS485_RD_PIN                GPIO_Pins_8
				#define Uart3_RS485_RD_GPIO               GPIOA
				#define Uart3_RS485_RD_INIT_STA           Bit_RESET
				#define Uart3_RS485_RD                    PAout(8)	
				#define Uart3_RS485_TX                    Uart3_RS485_RD = 1
				#define Uart3_RS485_RX                    Uart3_RS485_RD = 0
			#endif
//			#define Uart3_Rate													9600
			#define Uart3_Rate													115200
			#define Uart3_NVIC_Priority									5
			#define Uart3_DMA_CH												DMA1_Channel3//cal里的DMA弹性表格
			#define Uart3_DMA_Priority									DMA_PRIORITY_HIGH
			#define Uart3_DMA_ADDR											(0x40004800 + 0x04)//寄存器手册48页
			
			extern MCUART_Type SerUart3;
		#endif
		
		
		//使能串口5
		#define _Uart5 																0
		#if _Uart5 == 1
			//0-PD2/PC12 1-PB8/PB9	
			#define Uart5_GPIO 													1
			//有485控制脚
			#define _Uart5_RS485_RD 										1
			#if _Uart5_RS485_RD == 1
				#define Uart5_RS485_RD_PIN                GPIO_Pins_8
				#define Uart5_RS485_RD_GPIO               GPIOA
				#define Uart5_RS485_RD_INIT_STA           Bit_RESET
				#define Uart5_RS485_RD                    PAout(8)	
				#define Uart5_RS485_TX                    Uart5_RS485_RD = 1
				#define Uart5_RS485_RX                    Uart5_RS485_RD = 0
			#endif
			#define Uart5_Rate													115200
			#define Uart5_NVIC_Priority									5
			#define Uart5_DMA_CH												DMA2_Channel1//cal里的DMA弹性表格
			#define Uart5_DMA_Priority									DMA_PRIORITY_HIGH
			#define Uart5_DMA_ADDR											(0x40005000 + 0x04)//寄存器手册48页
			
			extern MCUART_Type SerUart5;
		#endif
		
		
		//使能串口6
		#define _Uart6 																0
		#if _Uart6 == 1
			//0-PC6/PC7 1-PA4/PA5	
			#define Uart6_GPIO 													1
			//有485控制脚
			#define _Uart6_RS485_RD 										1
			#if _Uart6_RS485_RD == 1
				#define Uart6_RS485_RD_PIN                GPIO_Pins_1
				#define Uart6_RS485_RD_GPIO               GPIOB
				#define Uart6_RS485_RD_INIT_STA           Bit_RESET
				#define Uart6_RS485_RD                    PBout(1)	
				#define Uart6_RS485_TX                    Uart6_RS485_RD = 1
				#define Uart6_RS485_RX                    Uart6_RS485_RD = 0
			#endif
			#define Uart6_Rate													115200
			#define Uart6_NVIC_Priority									5
			#define Uart6_DMA_CH												DMA2_Channel6//cal里的DMA弹性表格
			#define Uart6_DMA_Priority									DMA_PRIORITY_HIGH
			#define Uart6_DMA_ADDR											(0x40016000 + 0x04)//寄存器手册48页
			
			extern MCUART_Type SerUart6;
		#endif
		


	#endif


extern void UART_Init(void);
extern void Uart_IRQHandler(USART_Type* USARTx,DMA_Channel_Type* DMAy_Channelx,MCUART_Type * seruart);
extern void Modbus_Send(USART_Type* USARTx,uint8_t *pData,uint16_t Len);
#endif 

