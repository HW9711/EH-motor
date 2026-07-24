#include "usart.h"
#include "stdio.h"
#include "mcuart.h"




#if _AllUart == 1

	#if _Uart1 == 1
		MCUART_Type SerUart1;
	#endif
	#if _Uart2 == 1
		MCUART_Type SerUart2;
	#endif
	#if _Uart3 == 1
		MCUART_Type SerUart3;
	#endif
	#if _Uart4 == 1
		MCUART_Type SerUart4;
	#endif
	#if _Uart5 == 1
		MCUART_Type SerUart5;
	#endif
	#if _Uart6 == 1
		MCUART_Type SerUart6;
	#endif
	#if _Uart7 == 1
		MCUART_Type SerUart7;
	#endif
	#if _Uart8 == 1
		MCUART_Type SerUart8;
	#endif


//波特率配置
void USART_Base_Confi(USART_Type* USARTx,uint32_t rate){
	USART_InitType USART_InitStructure;
	
  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USARTx, &USART_InitStructure); 
}

//中断配置
void USART_NVIC_Confi(USART_Type* USARTx,uint8_t IRQChannel,uint8_t Priority){
	
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Priority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearITPendingBit(USARTx, USART_INT_IDLEF);
	USART_INTConfig(USARTx, USART_INT_IDLEF, ENABLE);//空闲中断
	USART_ClearITPendingBit(USARTx, USART_INT_IDLEF);
}


//DMA 配置
void USART_DMA_Confi(USART_Type* USARTx,DMA_Channel_Type* DMAy_Channelx,uint32_t PeripheralBaseAddr,uint8_t* MemoryBaseAddr,uint32_t DMA_priority){
	DMA_InitType DMA_InitStructure;
	DMA_Reset(DMAy_Channelx);                                               		
	DMA_DefaultInitParaConfig(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr    = PeripheralBaseAddr;          	//指定Channelx的外设基地址  DMA_CPBAx  传输源或目标
	DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)(MemoryBaseAddr);		//(uint32_t)&SerUart1.R_DATA;   //数据储存地址    DMA_CMBAx     写入这个地址
	DMA_InitStructure.DMA_Direction             = DMA_DIR_PERIPHERALSRC;        //从外设读
	DMA_InitStructure.DMA_BufferSize            = MAX_RXBUF;                    //传输个数  对应DMA传输通道个数
	DMA_InitStructure.DMA_PeripheralInc         = DMA_PERIPHERALINC_DISABLE;    //指定外设地址寄存器是否递增  由于都是从&ADC1->RDOR中读取，所以不需要递增
	DMA_InitStructure.DMA_MemoryInc             = DMA_MEMORYINC_ENABLE;         //指定内存地址是否递增        
	DMA_InitStructure.DMA_PeripheralDataWidth   = DMA_PERIPHERALDATAWIDTH_BYTE;	//数据宽度     外设
	DMA_InitStructure.DMA_MemoryDataWidth       = DMA_MEMORYDATAWIDTH_BYTE;   	//数据宽度      内存
	DMA_InitStructure.DMA_Mode                  = DMA_MODE_NORMAL;              //CIRM 循环模式 不循环
	DMA_InitStructure.DMA_Priority              = DMA_priority;          				//优先级
	DMA_InitStructure.DMA_MTOM                  = DMA_MEMTOMEM_DISABLE;         //是否内存到内存传输  否
	DMA_Init(DMAy_Channelx, &DMA_InitStructure);                                //将上面的配置赋给通道x
	DMA_ChannelEnable(DMAy_Channelx, ENABLE);                                   //使能
	USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);                              //使能UARTx的DMA传输
}

//接收中断配置
void Uart_IRQHandler(USART_Type* USARTx,DMA_Channel_Type* DMAy_Channelx,MCUART_Type * seruart){
	uint32_t Temp = 0;
	Temp = USARTx->STS;//必须将这两个寄存器的数值读出来
	Temp = USARTx->DT;//必须将这两个寄存器的数值读出来
	Temp = DMA_GetCurrDataCounter(DMAy_Channelx);//获取剩余数量
	seruart->RxLen = MAX_RXBUF - Temp; //长度
	seruart->RxEnd = 1;//接收完成
	//为下一次接收做准备
	DMAy_Channelx->CHCTRL &= 0xFFFFFFFE;
	DMAy_Channelx->TCNT = MAX_RXBUF;
	DMAy_Channelx->CHCTRL |= 0x00000001;
	
}



void Modbus_Send(USART_Type* USARTx, uint8_t *pData,uint16_t Len){
	uint8_t i = 0;
	uint32_t j = 0;
	
	if(USARTx == USART1){
		#ifdef Uart1_RS485_TX
		Uart1_RS485_TX;
		#endif
	}else if(USARTx == USART2){
		#ifdef Uart2_RS485_TX
		Uart2_RS485_TX;
		#endif
	}else if(USARTx == USART3){
		#ifdef Uart3_RS485_TX
		Uart3_RS485_TX;
		#endif
	}else if(USARTx == UART4){
		#ifdef Uart4_RS485_TX
		Uart4_RS485_TX;
		#endif
	}else if(USARTx == UART5){
		#ifdef Uart5_RS485_TX
		Uart5_RS485_TX;
		#endif
	}else if(USARTx == USART6){
		#ifdef Uart6_RS485_TX
		Uart6_RS485_TX;
		#endif
	}else if(USARTx == UART7){
		#ifdef Uart7_RS485_TX
		Uart7_RS485_TX;
		#endif
	}else if(USARTx == UART8){
		#ifdef Uart8_RS485_TX
		Uart8_RS485_TX;
		#endif
	}
	
	
	
	for(i = 0;i < Len;i ++){
		USART_SendData(USARTx,*(pData+i));
		j = 0;
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TRAC) == RESET && j < 30000){
			j ++;
		}
	}
	
	if(USARTx == USART1){
		#ifdef Uart1_RS485_RX
		Uart1_RS485_RX;
		#endif
	}else if(USARTx == USART2){
		#ifdef Uart2_RS485_RX
		Uart2_RS485_RX;
		#endif
	}else if(USARTx == USART3){
		#ifdef Uart3_RS485_RX
		Uart3_RS485_RX;
		#endif
	}else if(USARTx == UART4){
		#ifdef Uart4_RS485_RX
		Uart4_RS485_RX;
		#endif
	}else if(USARTx == UART5){
		#ifdef Uart5_RS485_RX
		Uart5_RS485_RX;
		#endif
	}else if(USARTx == USART6){
		#ifdef Uart6_RS485_RX
		Uart6_RS485_RX;
		#endif
	}else if(USARTx == UART7){
		#ifdef Uart7_RS485_RX
		Uart7_RS485_RX;
		#endif
	}else if(USARTx == UART8){
		#ifdef Uart8_RS485_RX
		Uart8_RS485_RX;
		#endif
	}
	
	
}




#endif



void UART_Init(void){
	
	#if _AllUart == 1
	GPIO_InitType GPIO_InitStructure;
	
		#if _Uart1 == 1
			//GPIO
			#if Uart1_GPIO == 0
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_9; //PA9 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOA, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_10;//PA10 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
			#elif Uart1_GPIO == 1
				GPIO_PinsRemapConfig(GPIO_Remap_USART1, ENABLE);
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6; //PB6 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOB, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_7;//PB7 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOB, &GPIO_InitStructure);
			#else
				错误
			#endif
			
			//RS485_RD
			#if _Uart1_RS485_RD == 1
				GPIO_StructInit(&GPIO_InitStructure); 
				GPIO_WriteBit(Uart1_RS485_RD_GPIO,Uart1_RS485_RD_PIN,Uart1_RS485_RD_INIT_STA);
				GPIO_InitStructure.GPIO_Pins = Uart1_RS485_RD_PIN;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
				GPIO_Init(Uart1_RS485_RD_GPIO, &GPIO_InitStructure);
				Uart1_RS485_RX;
			#endif
			//时钟
			RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART1, ENABLE);
			//波特率
			USART_Base_Confi(USART1,Uart1_Rate);
			//中断
			USART_NVIC_Confi(USART1,USART1_IRQn,Uart1_NVIC_Priority);
			USART_DMA_Confi(USART1,Uart1_DMA_CH,Uart1_DMA_ADDR,&SerUart1.R_DATA[0],Uart1_DMA_Priority);
			//使能串口
			USART_Cmd(USART1, ENABLE);
		#endif
		
		
		
		#if _Uart3 == 1
			//GPIO
			#if Uart3_GPIO == 0
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_10; //PB10 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOB, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_11;//PB11 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOB, &GPIO_InitStructure);
			#elif Uart3_GPIO == 1
				GPIO_PinsRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_10; //PC10 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOC, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_11;//PC11 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOC, &GPIO_InitStructure);
			#elif Uart3_GPIO == 2
				GPIO_PinsRemapConfig(GPIO_FullRemap_USART3, ENABLE);
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_8; //PD8 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOD, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_9;//PD9 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOD, &GPIO_InitStructure);	
			#else
				错误
			#endif
			
			//RS485_RD
			#if _Uart3_RS485_RD == 1
				GPIO_StructInit(&GPIO_InitStructure); 
				GPIO_WriteBit(Uart3_RS485_RD_GPIO,Uart3_RS485_RD_PIN,Uart3_RS485_RD_INIT_STA);
				GPIO_InitStructure.GPIO_Pins = Uart3_RS485_RD_PIN;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
				GPIO_Init(Uart3_RS485_RD_GPIO, &GPIO_InitStructure);
				Uart3_RS485_RX;
			#endif
			//时钟
			RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_USART3, ENABLE);
			//波特率
			USART_Base_Confi(USART3,Uart3_Rate);
			//中断
			USART_NVIC_Confi(USART3,USART3_IRQn,Uart3_NVIC_Priority);
			USART_DMA_Confi(USART3,Uart3_DMA_CH,Uart3_DMA_ADDR,&SerUart3.R_DATA[0],Uart3_DMA_Priority);
			//使能串口
			USART_Cmd(USART3, ENABLE);
		#endif
		
		
		
		
		#if _Uart5 == 1
			//GPIO
			#if Uart5_GPIO == 0
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_12; //PC12 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOC, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_2;//PD2 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOD, &GPIO_InitStructure);
			#elif Uart5_GPIO == 1
				GPIO_PinsRemapConfig(AFIO_MAP5_USART5_0001, ENABLE);
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_9; //PB8 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOB, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_8;//PB9 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOB, &GPIO_InitStructure);
			#else
				错误
			#endif
			
			//RS485_RD
			#if _Uart5_RS485_RD == 1
				GPIO_StructInit(&GPIO_InitStructure); 
				GPIO_WriteBit(Uart5_RS485_RD_GPIO,Uart5_RS485_RD_PIN,Uart5_RS485_RD_INIT_STA);
				GPIO_InitStructure.GPIO_Pins = Uart5_RS485_RD_PIN;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
				GPIO_Init(Uart5_RS485_RD_GPIO, &GPIO_InitStructure);
				Uart5_RS485_RX;
			#endif
			//时钟
			RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_UART5, ENABLE);
			//波特率
			USART_Base_Confi(UART5,Uart5_Rate);
			//中断
			USART_NVIC_Confi(UART5,UART5_IRQn,Uart5_NVIC_Priority);
			USART_DMA_Confi(UART5,Uart5_DMA_CH,Uart5_DMA_ADDR,&SerUart5.R_DATA[0],Uart5_DMA_Priority);
			//使能串口
			USART_Cmd(UART5, ENABLE);
		#endif
		
		
		#if _Uart6 == 1
			//GPIO
			#if Uart6_GPIO == 0
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6; //PC6 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOC, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_7;//PC7 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOC, &GPIO_InitStructure);
			#elif Uart6_GPIO == 1
				GPIO_PinsRemapConfig(AFIO_MAP8_USART6_0001, ENABLE);
				GPIO_StructInit(&GPIO_InitStructure);
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_4; //PA4 T
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
				GPIO_Init(GPIOA, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_5;//PA5 R
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
			#else
				错误
			#endif
			
			//RS485_RD
			#if _Uart6_RS485_RD == 1
				GPIO_StructInit(&GPIO_InitStructure); 
				GPIO_WriteBit(Uart6_RS485_RD_GPIO,Uart6_RS485_RD_PIN,Uart6_RS485_RD_INIT_STA);
				GPIO_InitStructure.GPIO_Pins = Uart6_RS485_RD_PIN;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
				GPIO_Init(Uart6_RS485_RD_GPIO, &GPIO_InitStructure);
				Uart6_RS485_RX;
			#endif
			//时钟
			RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART6, ENABLE);
			//波特率
			USART_Base_Confi(USART6,Uart6_Rate);
			//中断
			USART_NVIC_Confi(USART6,USART6_IRQn,Uart6_NVIC_Priority);
			USART_DMA_Confi(USART6,Uart6_DMA_CH,Uart6_DMA_ADDR,&SerUart6.R_DATA[0],Uart6_DMA_Priority);
			//使能串口
			USART_Cmd(USART6, ENABLE);
		#endif
		
	#endif
	
}




