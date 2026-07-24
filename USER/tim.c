#include "tim.h"
#include "mcctl.h"



//0 上桥 高 下桥 高 
//1 上桥 低 下桥 低
#define PWM_Polarity    0


void TIM_500us_Confi(void);
void TIM_Motor_PWM(TMR_Type* TMRx);
void PWM_Input(TMR_Type* TMRx);
void Hall_Tim_Init(TMR_Type* TMRx);
void PWM_Output(TMR_Type* TMRx,uint8_t ch,uint8_t fuyong);
void TIM_Encoder_Confi(TMR_Type* TMRx,uint8_t ch,uint8_t fuyong,uint8_t change);
void TIM3_BUCK_Init(void);

//所有定时器初始化
void TIM_Init_App(void){
	TIM_500us_Confi();		//500us基本定时器		//TMR6
	TIM_Motor_PWM(TMR1);	//电机1					//TMR1
	TIM_Motor_PWM(TMR8);	//电机2					//TMR8
	PWM_Output(TMR2,4,0);							//TMR2
	//保证两个定时器同时开，因为ADC采样需要在两个定时器下桥处理。
	TMR1->CTRL1 |= TMR_CTRL1_CNTEN;	
	TMR8->CTRL1 |= TMR_CTRL1_CNTEN;
	//HALL定时器5计时
	TIM_HALL_Confi();		//HALL定时器5			//TMR5
	//无感霍尔定时器判定
	TIM_ENABC_Confi();	//无HALL定时器3		//TMR3

}


void PWM_Output(TMR_Type* TMRx,uint8_t ch,uint8_t fuyong){
	GPIO_InitType GPIO_InitStructure;
	TMR_OCInitType  TMR_OCInitStructure;
	TMR_TimerBaseInitType  TMR_TMReBaseStructure;
	
	if(TMRx == TMR2){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR2, ENABLE);
		//PA3
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
//		GPIO_PinsRemapConfig(GPIO_FullRemap_TMR2, ENABLE);    

	}else if(TMRx == TMR3){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
		GPIO_PinsRemapConfig(GPIO_PartialRemap_TMR3, ENABLE);    
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
		GPIO_PinsRemapConfig(GPIO_PartialRemap_TMR3, ENABLE); 

	}else if(TMRx == TMR4){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR4, ENABLE);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

	}else if(TMRx == TMR11){
		RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_TMR11, ENABLE);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
		
	}
	
	

	TMR_TimeBaseStructInit(&TMR_TMReBaseStructure);
	TMR_TMReBaseStructure.TMR_Period = (PBCLK_PWM/PWM_FREQUENCY)/2-1;
	TMR_TMReBaseStructure.TMR_DIV = 1 - 1;
	TMR_TMReBaseStructure.TMR_ClockDivision = 0;
	TMR_TMReBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;
	TMR_TimeBaseInit(TMRx, &TMR_TMReBaseStructure);
	TMR_ARPreloadConfig(TMRx, ENABLE);
	
	TMR_OCStructInit(&TMR_OCInitStructure);
	TMR_OCInitStructure.TMR_OCMode = TMR_OCMode_PWM1;
	TMR_OCInitStructure.TMR_OutputState = TMR_OutputState_Enable;
	TMR_OCInitStructure.TMR_OutputNState =TMR_OutputNState_Enable;
	TMR_OCInitStructure.TMR_OCPolarity = TMR_OCPolarity_High;
	TMR_OCInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_High;
	TMR_OCInitStructure.TMR_OCIdleState = TMR_OCIdleState_Reset;
	TMR_OCInitStructure.TMR_OCIdleState = TMR_OCNIdleState_Reset;
	
	
	
	
	
	
	if(ch == 1){
		TMR_OC1Init(TMRx, &TMR_OCInitStructure);
		TMR_OC1PreloadConfig(TMRx, TMR_OCPreload_Enable);
	}else if(ch == 2){
		TMR_OC2Init(TMRx, &TMR_OCInitStructure);
		TMR_OC2PreloadConfig(TMRx, TMR_OCPreload_Enable);
	}else if(ch == 3){
		TMR_OC3Init(TMRx, &TMR_OCInitStructure);
		TMR_OC3PreloadConfig(TMRx, TMR_OCPreload_Enable);
	}else if(ch == 4){
		TMR_OC1PreloadConfig(TMRx, TMR_OCPreload_Disable);
		TMR_OC2PreloadConfig(TMRx, TMR_OCPreload_Disable);
		TMR_OC3PreloadConfig(TMRx, TMR_OCPreload_Disable);
		TMR_OCInitStructure.TMR_Pulse = (PBCLK_PWM / PWM_FREQUENCY / 2) - 6;
		TMR_OC4Init(TMRx, &TMR_OCInitStructure);
		TMR_OC4PreloadConfig(TMRx, TMR_OCPreload_Disable);

	}
	
	TMR_Cmd(TMRx, ENABLE);
}




void Hall_Tim_Init(TMR_Type* TMRx){
	NVIC_InitType NVIC_InitStructure;
	GPIO_InitType GPIO_InitStructure = {0};
	TMR_TimerBaseInitType  TMR_TimerBaseInitStructure = {0};
	TMR_ICInitType  TIM_ICInitStructure = {0};
	
	
	if(TMRx == TMR2){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR2, ENABLE);
		/* Enable the TMR3 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TMR2_GLOBAL_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//霍尔中断
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		/* TMR3 channel1 ,channel2 and channel3 configuration */
		GPIO_PinsRemapConfig(GPIO_FullRemap_TMR2, ENABLE);//TIM3----PC6 7 8 完全重映射
		GPIO_StructInit(&GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_3 | GPIO_Pins_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	}else if(TMRx == TMR3){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE);
		/* Enable the TMR3 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TMR3_GLOBAL_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//霍尔中断
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		/* TMR3 channel1 ,channel2 and channel3 configuration */
		GPIO_PinsRemapConfig(GPIO_FullRemap_TMR3, ENABLE);//TIM3----PC6 7 8 完全重映射
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6 | GPIO_Pins_7 | GPIO_Pins_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	}else if(TMRx == TMR4){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR4, ENABLE);
		
		NVIC_InitStructure.NVIC_IRQChannel = TMR4_GLOBAL_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//霍尔中断
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6 | GPIO_Pins_7 | GPIO_Pins_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
	
  TMR_SelectPlusMode(TMRx ,TMR_Plus_Mode_Enable);
  TMR_TimeBaseStructInit(&TMR_TimerBaseInitStructure);
  TMR_TimerBaseInitStructure.TMR_Period = 65535;
  TMR_TimerBaseInitStructure.TMR_ClockDivision = TMR_CKD_DIV1;
  TMR_TimerBaseInitStructure.TMR_CounterMode = TMR_CounterDIR_Up;
  TMR_TimerBaseInitStructure.TMR_DIV = 240 - 1;
  TMR_TimeBaseInit(TMRx, &TMR_TimerBaseInitStructure);

  
	
	TMR_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TMR_Channel = TMR_Channel_2;
  TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_TRC;
	TIM_ICInitStructure.TMR_ICFilter = 0xf;
  TMR_ICInit(TMRx, &TIM_ICInitStructure);
	
	TMR_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TMR_Channel = TMR_Channel_3;
  TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_TRC;
	TIM_ICInitStructure.TMR_ICFilter = 0xf;
  TMR_ICInit(TMRx, &TIM_ICInitStructure);
	
	TMR_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TMR_Channel = TMR_Channel_1;
  TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_TRC;
	TIM_ICInitStructure.TMR_ICFilter = 0xf;
  TMR_ICInit(TMRx, &TIM_ICInitStructure);

  /* XOR funtion enable */
  TMR_SelectHallSensor(TMRx, ENABLE);

  /* Select XOR signal as trigger */
  TMR_SelectInputTrigger(TMRx, TMR_TRGSEL_TI1F_ED);

  /* TMR3 counter reset each trigger */
  TMR_SelectSlaveMode(TMRx, TMR_SlaveMode_Reset);

  /* Trigger interrupt enable */
  TMR_INTConfig(TMRx, TMR_INT_Trigger , ENABLE);
	TMR_INTConfig(TMRx, TMR_INT_Overflow , ENABLE);

  /* TMR enable counter */
  TMR_Cmd(TMRx, ENABLE);
}


//PWM输入检测
void PWM_Input(TMR_Type* TMRx){
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	TMR_ICInitType  TMR_ICInitStructure;
	TMR_TimerBaseInitType  TMR_TMReBaseStructure;
	
	if(TMRx == TMR2){//TIM2_PA15
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_PinsRemapConfig(GPIO_FullRemap_TMR2, ENABLE);    

		NVIC_InitStructure.NVIC_IRQChannel = TMR2_GLOBAL_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//PWMIN输入检测
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);		
	}else if(TMRx == TMR3){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
		GPIO_PinsRemapConfig(GPIO_PartialRemap_TMR3, ENABLE);    

		NVIC_InitStructure.NVIC_IRQChannel = TMR3_GLOBAL_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//PWMIN输入检测
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
	}

	TMR_TimeBaseStructInit(&TMR_TMReBaseStructure);
  TMR_TMReBaseStructure.TMR_Period = 0xffff;
  TMR_TMReBaseStructure.TMR_DIV = 50;
  TMR_TMReBaseStructure.TMR_ClockDivision = 0;
	TMR_TMReBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;
  TMR_TimeBaseInit(TMRx, &TMR_TMReBaseStructure);
	TMR_ARPreloadConfig(TMRx, ENABLE);
  
  
	TMR_ICStructInit(&TMR_ICInitStructure);
  TMR_ICInitStructure.TMR_Channel = TMR_Channel_1;
  TMR_ICInitStructure.TMR_ICPolarity = TMR_ICPolarity_Falling ;
  TMR_ICInitStructure.TMR_ICSelection = TMR_ICSelection_DirectTI;
  TMR_ICInitStructure.TMR_ICDIV = TMR_ICDIV_DIV1;
  TMR_ICInitStructure.TMR_ICFilter = 0x06;
  TMR_ICInit(TMRx, &TMR_ICInitStructure);
	
	TMR_ICStructInit(&TMR_ICInitStructure);
  TMR_ICInitStructure.TMR_Channel = TMR_Channel_2;
  TMR_ICInitStructure.TMR_ICPolarity = TMR_ICPolarity_Rising;
  TMR_ICInitStructure.TMR_ICSelection = TMR_ICSelection_IndirectTI;
  TMR_ICInitStructure.TMR_ICDIV = TMR_ICDIV_DIV1;
  TMR_ICInitStructure.TMR_ICFilter = 0x06;
  TMR_ICInit(TMRx, &TMR_ICInitStructure);

  TMR_Cmd(TMRx, ENABLE);

  TMR_INTConfig(TMRx, TMR_INT_CC1, ENABLE);
	TMR_INTConfig(TMRx, TMR_INT_Overflow, ENABLE);
}




//定时器-500us
void TIM_500us_Confi(void){
	TMR_TimerBaseInitType  TMR_TMReBaseStructure;
	NVIC_InitType 				 NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR6, ENABLE);
	TMR_TimeBaseStructInit(&TMR_TMReBaseStructure);
  TMR_TMReBaseStructure.TMR_Period = 500 - 1;//500us
  TMR_TMReBaseStructure.TMR_DIV = 240 - 1;
  TMR_TMReBaseStructure.TMR_ClockDivision = 0;
	TMR_TMReBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;
	TMR_TimeBaseInit(TMR6, &TMR_TMReBaseStructure);
	TMR_ARPreloadConfig(TMR6, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TMR6_GLOBAL_IRQn; //定时器6中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//定时中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	TMR_INTConfig(TMR6, TMR_INT_Overflow, ENABLE);//溢出中断
	TMR_Cmd(TMR6, ENABLE);
}

//计数器，计算HALL变化直接的间隔时间		
void TIM_HALL_Confi(void){
	TMR_TimerBaseInitType  TMR_TMReBaseStructure;
	NVIC_InitType 				 NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR5, ENABLE);		//TMR5
	TMR_TimeBaseStructInit(&TMR_TMReBaseStructure);
	TMR_TMReBaseStructure.TMR_Period = 50000-1;			//2ms 产生一次中断
	TMR_TMReBaseStructure.TMR_DIV = 24 - 1;			//1,0000,000HZ
	TMR_TMReBaseStructure.TMR_ClockDivision = 0;
	TMR_TMReBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;	//向上计数
	TMR_TimeBaseInit(TMR5, &TMR_TMReBaseStructure);
	TMR_ARPreloadConfig(TMR5, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TMR5_GLOBAL_IRQn; //定时器6中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;//定时中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	TMR_INTConfig(TMR5, TMR_INT_Overflow, ENABLE);//溢出中断
	TMR_Cmd(TMR5, ENABLE);
}


//计数器，计算HALL变化直接的间隔时间		
void TIM_HALL_Disable(void){
	TMR_TimerBaseInitType  TMR_TMReBaseStructure;
	NVIC_InitType 				 NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR5, DISABLE);		//TMR5
	TMR_TimeBaseStructInit(&TMR_TMReBaseStructure);
	TMR_TMReBaseStructure.TMR_Period = 50000-1;			//2ms 产生一次中断
	TMR_TMReBaseStructure.TMR_DIV = 24 - 1;			//1,0000,000HZ
	TMR_TMReBaseStructure.TMR_ClockDivision = 0;
	TMR_TMReBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;	//向上计数
	TMR_TimeBaseInit(TMR5, &TMR_TMReBaseStructure);
	TMR_ARPreloadConfig(TMR5, DISABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TMR5_GLOBAL_IRQn; //定时器6中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;//定时中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
	TMR_INTConfig(TMR5, TMR_INT_Overflow, DISABLE);//溢出中断
	TMR_Cmd(TMR5, DISABLE);
}



//计数器，用于切闭环校验，并且实现消抖换相
void TIM_ENABC_Confi(void){
	TMR_TimerBaseInitType  TMR_TMReBaseStructure;
	NVIC_InitType 				 NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE);		//TMR3
	TMR_TimeBaseStructInit(&TMR_TMReBaseStructure);
	TMR_TMReBaseStructure.TMR_Period = 50-1;			//产生一次中断 	5us
	TMR_TMReBaseStructure.TMR_DIV = 24 - 1;			//1,0000,000HZ
	TMR_TMReBaseStructure.TMR_ClockDivision = 0;
	TMR_TMReBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;	//向上计数
	TMR_TimeBaseInit(TMR3, &TMR_TMReBaseStructure);
	TMR_ARPreloadConfig(TMR3, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TMR3_GLOBAL_IRQn; //定时器3中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//定时中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	TMR_INTConfig(TMR3, TMR_INT_Overflow, ENABLE);//溢出中断
	TMR_Cmd(TMR3, ENABLE);
}


void TIM3_DISABLE(void){
	TMR_TimerBaseInitType  TMR_TMReBaseStructure;
	NVIC_InitType 				 NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, DISABLE);		//TMR3
	TMR_TimeBaseStructInit(&TMR_TMReBaseStructure);
	TMR_TMReBaseStructure.TMR_Period = 50-1;			//产生一次中断 	5us
	TMR_TMReBaseStructure.TMR_DIV = 24 - 1;			//1,0000,000HZ
	TMR_TMReBaseStructure.TMR_ClockDivision = 0;
	TMR_TMReBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;	//向上计数
	TMR_TimeBaseInit(TMR3, &TMR_TMReBaseStructure);
	TMR_ARPreloadConfig(TMR3, DISABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TMR3_GLOBAL_IRQn; //定时器6中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//定时中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
	TMR_INTConfig(TMR3, TMR_INT_Overflow, DISABLE);//溢出中断
	TMR_Cmd(TMR3, DISABLE);
}



//高级定时器配置
void TIM_Motor_PWM(TMR_Type* TMRx){
	GPIO_InitType 				 GPIO_InitStructure;
	TMR_TimerBaseInitType  TMR_TMReBaseStructure;
	NVIC_InitType 				 NVIC_InitStructure;
	TMR_OCInitType  			 TMR_OCInitStructure;
	TMR_BRKDTInitType 		 TMR_BDTRInitStruct;
	
	//IO 配置
	if(TMRx == TMR1){
		//高级定时器1
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_14;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//刹车
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		//时基
		RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_TMR1, ENABLE);
	}else if(TMRx == TMR8){
		//高级定时器8
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
//		//BUCK  不用CH4！！！！！
//		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_9;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
//		GPIO_Init(GPIOC, &GPIO_InitStructure);


		//刹车
		GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//时基
		RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_TMR8, ENABLE);
	}
	
	
	TMR_TimeBaseStructInit(&TMR_TMReBaseStructure);
	TMR_TMReBaseStructure.TMR_Period = (PBCLK_PWM / PWM_FREQUENCY / 2) - 1;
	TMR_TMReBaseStructure.TMR_DIV = 1 - 1;
	TMR_TMReBaseStructure.TMR_ClockDivision = 0;
	TMR_TMReBaseStructure.TMR_CounterMode = TMR_CounterDIR_CenterAligned1;
	
	TMR_TimeBaseInit(TMRx, &TMR_TMReBaseStructure);
	TMR_ARPreloadConfig(TMRx, ENABLE);
	//PWMOUT配置
	#if PWM_Polarity == 0
		TMR_OCStructInit(&TMR_OCInitStructure);
		TMR_OCInitStructure.TMR_OCMode = TMR_OCMode_PWM1;
		TMR_OCInitStructure.TMR_OutputState = TMR_OutputState_Enable;
		TMR_OCInitStructure.TMR_OutputNState = TMR_OutputNState_Enable;
		TMR_OCInitStructure.TMR_OCPolarity = TMR_OCPolarity_High;
		TMR_OCInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_High;
		TMR_OCInitStructure.TMR_OCIdleState = TMR_OCIdleState_Reset;
		TMR_OCInitStructure.TMR_OCNIdleState = TMR_OCNIdleState_Reset;
		
		//CH1
		TMR_OCInitStructure.TMR_Pulse = 0;
		TMR_OC1Init(TMRx, &TMR_OCInitStructure);
		TMR_OC1PreloadConfig(TMRx, TMR_OCPreload_Enable);
		//CH2
		TMR_OCInitStructure.TMR_Pulse = 0;
		TMR_OC2Init(TMRx, &TMR_OCInitStructure);
		TMR_OC2PreloadConfig(TMRx, TMR_OCPreload_Enable);
		//CH3
		TMR_OCInitStructure.TMR_Pulse = 0;
		TMR_OC3Init(TMRx, &TMR_OCInitStructure);
		TMR_OC3PreloadConfig(TMRx, TMR_OCPreload_Enable);
		//CH4
		TMR_OCInitStructure.TMR_Pulse = (PBCLK_PWM / PWM_FREQUENCY / 2) - 5;
		TMR_OC4Init(TMRx, &TMR_OCInitStructure);
		TMR_OC4PreloadConfig(TMRx, TMR_OCPreload_Disable);
//		TMR_OC4PreloadConfig(TMRx, TMR_OCPreload_Enable);
		
		//死区
		TMR_BDTRInitStruct.TMR_AutomaticOutput = TMR_AutomaticOutput_Enable;
		TMR_BDTRInitStruct.TMR_Break = TMR_Break_Enable;											//不允许刹车去关断TIM
		TMR_BDTRInitStruct.TMR_BreakPolarity = TMR_BreakPolarity_High;
		TMR_BDTRInitStruct.TMR_DeadTime = 0xB0;//0xED 3us E0 -> 2.0us C8->1.2 CE 1.8
		TMR_BDTRInitStruct.TMR_LOCKgrade = TMR_LOCKgrade_OFF;
		TMR_BDTRInitStruct.TMR_OSIMIState = TMR_OSIMIState_Disable;
		TMR_BDTRInitStruct.TMR_OSIMRState = TMR_OSIMRState_Disable;
		TMR_BRKDTConfig(TMRx,&TMR_BDTRInitStruct);
	#elif PWM_Polarity == 1
		TMR_OCStructInit(&TMR_OCInitStructure);
		TMR_OCInitStructure.TMR_OCMode = TMR_OCMode_PWM1;
		TMR_OCInitStructure.TMR_OutputState = TMR_OutputState_Enable;
		TMR_OCInitStructure.TMR_OutputNState = TMR_OutputNState_Enable;
		TMR_OCInitStructure.TMR_OCPolarity = TMR_OCPolarity_Low;
		TMR_OCInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_Low;
		TMR_OCInitStructure.TMR_OCIdleState = TMR_OCIdleState_Reset;
		TMR_OCInitStructure.TMR_OCNIdleState = TMR_OCNIdleState_Reset;
		
		//CH1
		TMR_OCInitStructure.TMR_Pulse = 0;
		TMR_OC1Init(TMRx, &TMR_OCInitStructure);
		TMR_OC1PreloadConfig(TMRx, TMR_OCPreload_Disable);
		//CH2
		TMR_OCInitStructure.TMR_Pulse = 0;
		TMR_OC2Init(TMRx, &TMR_OCInitStructure);
		TMR_OC2PreloadConfig(TMRx, TMR_OCPreload_Disable);
		//CH3
		TMR_OCInitStructure.TMR_Pulse = 0;
		TMR_OC3Init(TMRx, &TMR_OCInitStructure);
		TMR_OC3PreloadConfig(TMRx, TMR_OCPreload_Disable);
		//CH4
		TMR_OCInitStructure.TMR_Pulse = (PBCLK_PWM / PWM_FREQUENCY / 2) - 6;
		TMR_OC4Init(TMRx, &TMR_OCInitStructure);
		TMR_OC4PreloadConfig(TMRx, TMR_OCPreload_Disable);
		//死区
		TMR_BDTRInitStruct.TMR_AutomaticOutput = TMR_AutomaticOutput_Enable;
		TMR_BDTRInitStruct.TMR_Break = TMR_Break_Enable;
		TMR_BDTRInitStruct.TMR_BreakPolarity = TMR_BreakPolarity_High;
		TMR_BDTRInitStruct.TMR_DeadTime = 0xE0;
		TMR_BDTRInitStruct.TMR_LOCKgrade = TMR_LOCKgrade_OFF;
		TMR_BDTRInitStruct.TMR_OSIMIState = TMR_OSIMIState_Enable;
		TMR_BDTRInitStruct.TMR_OSIMRState = TMR_OSIMRState_Enable;
		TMR_BRKDTConfig(TMRx,&TMR_BDTRInitStruct);
	#endif
	
	
	TMR_CCxCmd(TMRx, TMR_Channel_1, TMR_CCx_Disable);  
	TMR_CCxCmd(TMRx, TMR_Channel_2, TMR_CCx_Disable);  
	TMR_CCxCmd(TMRx, TMR_Channel_3, TMR_CCx_Disable);  
	TMR_CCxNCmd(TMRx, TMR_Channel_1, TMR_CCxN_Disable);  
	TMR_CCxNCmd(TMRx, TMR_Channel_2, TMR_CCxN_Disable);  
	TMR_CCxNCmd(TMRx, TMR_Channel_3, TMR_CCxN_Disable);
	
//	TMR_CCxCmd(TMRx, TMR_Channel_4, TMR_CCx_Disable);
	TMR_CCxCmd(TMRx, TMR_Channel_4, TMR_CCx_Enable);
	
	
	if(TMRx == TMR1){
		NVIC_InitStructure.NVIC_IRQChannel = TMR1_BRK_TMR9_IRQn; //定时器1刹车中断
	}
	else if(TMRx == TMR8){
		NVIC_InitStructure.NVIC_IRQChannel = TMR8_BRK_TMR12_IRQn; //定时器1刹车中断
	}
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//刹车中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TMR_INTConfig(TMRx, TMR_INT_Break, ENABLE);//刹车中断
	TMR_CtrlPWMOutputs(TMRx, ENABLE);//高级定时器才需要
	
	TMR_Cmd(TMRx, DISABLE);
	//TMR_Cmd(TMRx, ENABLE);
}


void TIM_Encoder_Confi(TMR_Type* TMRx,uint8_t ch,uint8_t fuyong,uint8_t change){
	TMR_TimerBaseInitType  	TMR_TimerBaseInitStructure = {0};
	//TMR_ICInitType 					TIM_ICInitStructure;
	NVIC_InitType 					NVIC_InitStructure;
	
	if(TMRx == TMR2){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR2, ENABLE);
		//12通道
		if(ch == 12){
			//IO 复用1
//			if(fuyong == 1){
//				GPIO_StructInit(&GPIO_InitStructure);
//				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_15;
//				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
//				GPIO_Init(GPIOA, &GPIO_InitStructure);
//				
//				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_3;
//				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
//				GPIO_Init(GPIOB, &GPIO_InitStructure);
//				GPIO_PinsRemapConfig(GPIO_PartialRemap1_TMR2, ENABLE);
//			}
			
		}
		//34通道
		else if(ch == 34){
			
		}
	}else if(TMRx == TMR3){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE);
		//12通道
		if(ch == 12){
//			//IO 复用1
//			if(fuyong == 1){//部分重映射
//				GPIO_StructInit(&GPIO_InitStructure);
//				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_4;
//				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
//				GPIO_Init(GPIOB, &GPIO_InitStructure);
//				
//				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_5;
//				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
//				GPIO_Init(GPIOB, &GPIO_InitStructure);
//				GPIO_PinsRemapConfig(GPIO_PartialRemap_TMR3, ENABLE);
//			}
			
		}
		//34通道
		else if(ch == 34){
			
		}
	}else if(TMRx == TMR4){
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR4, ENABLE);
		//12通道
		if(ch == 12){
			//无复用
//			if(fuyong == 0){
//				GPIO_StructInit(&GPIO_InitStructure);
//				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6;
//				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
//				GPIO_Init(GPIOB, &GPIO_InitStructure);
//				
//				GPIO_InitStructure.GPIO_Pins = GPIO_Pins_7;
//				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//				GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
//				GPIO_Init(GPIOB, &GPIO_InitStructure);
////			}
//			//IO 复用1
//			else if(fuyong == 1){//部分重映射
//				
//			}
			
		}
		//34通道
		else if(ch == 34){
			
		}
		
	}
	
	TMR_SelectPlusMode(TMRx ,TMR_Plus_Mode_Enable);
	
  TMR_TimeBaseStructInit(&TMR_TimerBaseInitStructure);
  TMR_TimerBaseInitStructure.TMR_Period = MCPara[83] - 1;
  TMR_TimerBaseInitStructure.TMR_ClockDivision = TMR_CKD_DIV1;
  TMR_TimerBaseInitStructure.TMR_CounterMode = TMR_CounterDIR_Down;
  TMR_TimerBaseInitStructure.TMR_DIV = 1 - 1;//4-1
  TMR_TimeBaseInit(TMRx, &TMR_TimerBaseInitStructure);
	
	//这个函数默认配置了12通道，只有12通道才适合做编码器
	if(change == 0){
		TMR_EncoderInterfaceConfig(TMRx, TMR_EncoderMode_TI12, TMR_ICPolarity_Falling, TMR_ICPolarity_Rising);
	}else {
		TMR_EncoderInterfaceConfig(TMRx, TMR_EncoderMode_TI12, TMR_ICPolarity_Rising, TMR_ICPolarity_Rising);
	}
	
	//TMR_EncoderInterfaceConfig(TMRx, TMR_EncoderMode_TI12, TMR_ICPolarity_Falling, TMR_ICPolarity_Rising);
//	TMR_EncoderInterfaceConfig(TMRx, TMR_EncoderMode_TI12, TMR_ICPolarity_Falling, TMR_ICPolarity_Falling);
  
//	if(ch == 12){
//		
//		
////		TIM_ICInitStructure.TMR_Channel = TMR_Channel_1;
////		TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_DirectTI; //映射到TI1上
////		TIM_ICInitStructure.TMR_ICDIV = 0;         //配置输入分频,不分频 
////		TIM_ICInitStructure.TMR_ICFilter = 15;//滤波器值
////		TMR_ICInit(TMRx, &TIM_ICInitStructure);
////		TIM_ICInitStructure.TMR_Channel = TMR_Channel_2;
////		TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_DirectTI; //映射到TI1上
////		TIM_ICInitStructure.TMR_ICDIV = 0;         //配置输入分频,不分频 
////		TIM_ICInitStructure.TMR_ICFilter = 15;//滤波器值
////		TMR_ICInit(TMRx, &TIM_ICInitStructure); 
//		
//		TIM_ICInitStructure.TMR_Channel = TMR_Channel_1;
//		TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_IndirectTI; //映射到TI1上
//		TIM_ICInitStructure.TMR_ICDIV = 0;         //配置输入分频,不分频 
//		TIM_ICInitStructure.TMR_ICFilter = 15;//滤波器值
//		TMR_ICInit(TMRx, &TIM_ICInitStructure);
//		TIM_ICInitStructure.TMR_Channel = TMR_Channel_2;
//		TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_IndirectTI; //映射到TI1上
//		TIM_ICInitStructure.TMR_ICDIV = 0;         //配置输入分频,不分频 
//		TIM_ICInitStructure.TMR_ICFilter = 15;//滤波器值
//		TMR_ICInit(TMRx, &TIM_ICInitStructure); 
//		
//	}else if(ch == 34){
//		TIM_ICInitStructure.TMR_Channel = TMR_Channel_3;
//		TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_DirectTI; //映射到TI1上
//		TIM_ICInitStructure.TMR_ICDIV = 0;         //配置输入分频,不分频 
//		TIM_ICInitStructure.TMR_ICFilter = 15;//滤波器值
//		TMR_ICInit(TMRx, &TIM_ICInitStructure);
//		TIM_ICInitStructure.TMR_Channel = TMR_Channel_4;
//		TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_DirectTI; //映射到TI1上
//		TIM_ICInitStructure.TMR_ICDIV = 0;         //配置输入分频,不分频 
//		TIM_ICInitStructure.TMR_ICFilter = 15;//滤波器值
//		TMR_ICInit(TMRx, &TIM_ICInitStructure); 
//	}
		
	
	
	
	TMR_SelectOutputTrigger(TMRx,TMR_TRGOSource_Reset);
	
	TMR_SelectMasterSlaveMode(TMRx,TMR_MasterSlaveMode_Disable);
	
	if(TMRx == TMR3){
		NVIC_InitStructure.NVIC_IRQChannel = TMR3_GLOBAL_IRQn; //定时器1刹车中断
	}else if(TMRx == TMR4){
		NVIC_InitStructure.NVIC_IRQChannel = TMR4_GLOBAL_IRQn; //定时器1刹车中断
	}
	
	
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//编码器溢出中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TMR_ClearITPendingBit(TMRx, TMR_INT_Overflow);
	TMR_INTConfig(TMRx, TMR_INT_Overflow, ENABLE);//刹车中断
	TMR_ClearITPendingBit(TMRx, TMR_INT_Overflow);
  /* TMR enable counter */
  TMR_Cmd(TMRx, ENABLE);
}






























