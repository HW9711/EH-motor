#include "gpio.h"

void GPIO_Init_App(void){
  GPIO_InitType GPIO_InitStructure;
//  NVIC_InitType NVIC_InitStructure = {0};	
//	EXTI_InitType EXTI_InitStructure = {0};
	#if _IPM == 1 || _ENZ == 1
	EXTI_InitType EXTI_InitStructure;
	NVIC_InitType NVIC_InitStructure = {0};
	#endif
	
	#if use_HSI == 1
	GPIO_PinsRemapConfig(GPIO_Remap_PD01, ENABLE);
	#endif
	
	//状态指示灯 
	#if _STA_LED == 1
  GPIO_StructInit(&GPIO_InitStructure); 
	GPIO_WriteBit(STA_LED_GPIO,STA_LED_PIN,STA_LED_INIT_STA);
  GPIO_InitStructure.GPIO_Pins = STA_LED_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(STA_LED_GPIO, &GPIO_InitStructure);
	#endif
	
	//开电容
	#if EN_CAP_ON == 1
  GPIO_StructInit(&GPIO_InitStructure);//填充默认值     
	GPIO_WriteBit(EN_CAP_GPIO,EN_CAP_PIN,EN_CAP_INIT_STA); //初始化       //改1-2
  GPIO_InitStructure.GPIO_Pins = EN_CAP_PIN;		                   //改2-1		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(EN_CAP_GPIO, &GPIO_InitStructure);                     //改3-1
	#endif
	
	//使能
	#if key_run == 1
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pins = KEYRUN_PIN;	 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(KEYRUN_GPIO, &GPIO_InitStructure); 
	#endif
	
	//方向
	#if key_dir == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = KEYDIR_PIN;	 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(KEYDIR_GPIO, &GPIO_InitStructure);
	#endif
	
	//刹车
	#if key_brake == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = KEYBRA_PIN;	 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(KEYBRA_GPIO, &GPIO_InitStructure);
	#endif

	//FG
	#if _FG == 1
	GPIO_StructInit(&GPIO_InitStructure);//填充默认值     
	GPIO_WriteBit(FG_GPIO,FG_PIN,Bit_RESET); //初始化       //改1-2
  GPIO_InitStructure.GPIO_Pins = FG_PIN;		                   //改2-1		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(FG_GPIO, &GPIO_InitStructure);                     //改3-1
	FGOUT = 1;
	#endif
	
	//hall 学习按键
	#if key_HallSdy == 1
	GPIO_StructInit(&GPIO_InitStructure);//填充默认值     
  GPIO_InitStructure.GPIO_Pins = KEYStady_PIN;		                   //改1-1		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(KEYStady_GPIO, &GPIO_InitStructure);                     //改2-1
	#endif

	//SW1
	#if Key_SW1 == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = KEYSW1_PIN;	 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(KEYSW1_GPIO, &GPIO_InitStructure);
	#endif
	
	//SW2
	#if Key_SW2 == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = KEYSW2_PIN;	 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(KEYSW2_GPIO, &GPIO_InitStructure);
	#endif
	
	//SW3
	#if Key_SW3 == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = KEYSW3_PIN;	 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(KEYSW3_GPIO, &GPIO_InitStructure);
	#endif
	
	//FAN
	#if _FAN == 1
	GPIO_StructInit(&GPIO_InitStructure);//填充默认值     
	GPIO_WriteBit(FAN_GPIO,FAN_PIN,Bit_RESET); //初始化       //改1-2
  GPIO_InitStructure.GPIO_Pins = FAN_PIN;		                   //改2-1		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(FAN_GPIO, &GPIO_InitStructure);                     //改3-1
	FAN = 0;//光风扇
	#endif
	
	
	//IPM 错误输出接口
	#if _IPM == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = IPM_PIN;		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(IPM_GPIO, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(IPM_GPIOSource, IPM_PINSource); 
	
	EXTI_InitStructure.EXTI_Line = IPM_Exti_Line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = IPM_NVIC_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//刹车中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	#endif
	
	
	//编码器Z轴错误
	#if _ENZ == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = ENZ_PIN;		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(ENZ_GPIO, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(ENZ_GPIOSource, ENZ_PINSource); 
	
	EXTI_InitStructure.EXTI_Line = ENZ_Exti_Line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = ENZ_NVIC_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//编码器Z轴中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	#endif
	
	
	//编码器Z轴错误
	#if _ENZ2 == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = ENZ2_PIN;		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(ENZ2_GPIO, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(ENZ2_GPIOSource, ENZ2_PINSource); 
	
	EXTI_InitStructure.EXTI_Line = ENZ2_Exti_Line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = ENZ2_NVIC_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//编码器Z轴中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	#endif
	
	
	
	
	//接近开关
	#if _PrsIN == 1
	GPIO_StructInit(&GPIO_InitStructure);   
  GPIO_InitStructure.GPIO_Pins = PrsIN_PIN;		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
  GPIO_Init(PrsIN_GPIO, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(PrsIN_GPIOSource, PrsIN_PINSource); 
	
	EXTI_InitStructure.EXTI_Line = PrsIN_Exti_Line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = PrsIN_NVIC_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//接近开关中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	#endif
	
	#if _Relay_SW == 0
	#if _Relay == 1
	GPIO_StructInit(&GPIO_InitStructure); 
	GPIO_WriteBit(Relay_GPIO,Relay_PIN,Relay_INIT_STA);
	GPIO_InitStructure.GPIO_Pins = Relay_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
	GPIO_Init(Relay_GPIO, &GPIO_InitStructure);

	#endif

	#endif
	
	//BUCK电路调压
	#if PVCC_PWM_EN ==1
	#if _PVCC_PWM_EN ==1
	
	GPIO_StructInit(&GPIO_InitStructure); 
	GPIO_WriteBit(PVCC_PWM_EN_GPIO,PVCC_PWM_EN_PIN,PVCC_PWM_INIT_STA);
	GPIO_InitStructure.GPIO_Pins = PVCC_PWM_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
	GPIO_Init(PVCC_PWM_EN_GPIO, &GPIO_InitStructure);	
	
	
	#endif
	#endif
	
}


void HALL1_GPIO_INIT(void){

	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure = {0};	
	EXTI_InitType EXTI_InitStructure = {0};
	
		//hall
	#if _HALL == 1
	GPIO_StructInit(&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pins = HALLA_PIN;		                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
	GPIO_Init(HALLA_GPIO, &GPIO_InitStructure);  
	//中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource10); 

    EXTI_InitStructure.EXTI_Line = EXTI_Line10;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 

	GPIO_InitStructure.GPIO_Pins = HALLB_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(HALLB_GPIO, &GPIO_InitStructure); 
// 	中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource11); 
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 

  
	GPIO_InitStructure.GPIO_Pins = HALLC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(HALLC_GPIO, &GPIO_InitStructure); 
  // 	中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource12);
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 
	#endif
}



void HALL1_GPIO_INIT_Disable(void){

	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure = {0};	
	EXTI_InitType EXTI_InitStructure = {0};
	
		//hall
	#if _HALL == 1
	GPIO_StructInit(&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pins = HALLA_PIN;		                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
	GPIO_Init(HALLA_GPIO, &GPIO_InitStructure);  
	//中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource10); 

  EXTI_InitStructure.EXTI_Line = DISABLE;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = DISABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 

	GPIO_InitStructure.GPIO_Pins = HALLB_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(HALLB_GPIO, &GPIO_InitStructure); 
// 	中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource11); 
    EXTI_InitStructure.EXTI_Line = DISABLE;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = DISABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 

  
	GPIO_InitStructure.GPIO_Pins = HALLC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(HALLC_GPIO, &GPIO_InitStructure); 
  // 	中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource12);
    EXTI_InitStructure.EXTI_Line = DISABLE;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = DISABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 
	#endif
}

void HALL2_GPIO_INIT(void){
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure = {0};	
	EXTI_InitType EXTI_InitStructure = {0};	
	
	
	//hall
	#if _HALL2 == 1
	GPIO_StructInit(&GPIO_InitStructure);
	//HAll2A
	GPIO_InitStructure.GPIO_Pins = HALLA2_PIN;		                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
	GPIO_Init(HALLA2_GPIO, &GPIO_InitStructure);  
	  //中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinsSource12); 

    EXTI_InitStructure.EXTI_Line = EXTI_Line12;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 


	//HAll2B
	GPIO_InitStructure.GPIO_Pins = HALLB2_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(HALLB2_GPIO, &GPIO_InitStructure);  
	//中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinsSource11); 

  EXTI_InitStructure.EXTI_Line = EXTI_Line11;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 
	
	////HAll2C
	GPIO_InitStructure.GPIO_Pins = HALLC2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(HALLC2_GPIO, &GPIO_InitStructure);  
	//中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource2); 

  EXTI_InitStructure.EXTI_Line = EXTI_Line2;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//9-5的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 
  
	#endif	
}


void HALL2_GPIO_INIT_Disable(void){
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure = {0};	
	EXTI_InitType EXTI_InitStructure = {0};	
	
	//hall
	#if _HALL2 == 1
	GPIO_StructInit(&GPIO_InitStructure);
	//HAll2A
	GPIO_InitStructure.GPIO_Pins = HALLA2_PIN;		                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
	GPIO_Init(HALLA2_GPIO, &GPIO_InitStructure);  
	  //中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinsSource12); 

    EXTI_InitStructure.EXTI_Line = DISABLE;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = DISABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 


	//HAll2B
	GPIO_InitStructure.GPIO_Pins = HALLB2_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(HALLB2_GPIO, &GPIO_InitStructure);  
	//中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinsSource11); 

    EXTI_InitStructure.EXTI_Line = DISABLE;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = DISABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 
	
	////HAll2C
	GPIO_InitStructure.GPIO_Pins = HALLC2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(HALLC2_GPIO, &GPIO_InitStructure);  
	//中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource2); 

    EXTI_InitStructure.EXTI_Line = DISABLE;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发！
	EXTI_InitStructure.EXTI_LineEnable = DISABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//9-5的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 
  
	#endif	
}



void GPIO_Noninductive1_Init(void){
//无感方波GPIO初始化
	GPIO_InitType GPIO_InitStructure;
	//无感过零点检测EN1A
	GPIO_InitStructure.GPIO_Pins = EN1A_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PD; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(EN1A_GPIO, &GPIO_InitStructure);  
	//无感过零点检测EN1B
	GPIO_InitStructure.GPIO_Pins = EN1B_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PD; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(EN1B_GPIO, &GPIO_InitStructure);
	//无感过零点检测EN1Z
	GPIO_InitStructure.GPIO_Pins = EN1Z_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PD; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(EN1Z_GPIO, &GPIO_InitStructure);  
}


void GPIO_Brake_Init(void){
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure = {0};	
	EXTI_InitType EXTI_InitStructure = {0};	
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pins = GPIO_Pins_6;		                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	  //中断!
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinsSource12); 

  EXTI_InitStructure.EXTI_Line = EXTI_Line6;						//中断线10
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;				//中断模式，不是事件触发模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//上升沿触发！
	EXTI_InitStructure.EXTI_LineEnable = ENABLE;					//使能中断线
	EXTI_Init(&EXTI_InitStructure);                              ///<Configure EXTI0 line
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;		//10-15的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		//优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断通道		
	NVIC_Init(&NVIC_InitStructure); 
	
}



void GPIO_Noninductive2_Init(void){
//无感方波GPIO初始化
	GPIO_InitType GPIO_InitStructure;
	//无感过零点检测EN2A
	GPIO_InitStructure.GPIO_Pins = EN2A_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PD; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(EN2A_GPIO, &GPIO_InitStructure);  
	//无感过零点检测EN2B
	GPIO_InitStructure.GPIO_Pins = EN2B_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PD; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(EN2B_GPIO, &GPIO_InitStructure);  
	//无感过零点检测EN1Z
	GPIO_InitStructure.GPIO_Pins = EN2Z_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PD; 		 
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		
	GPIO_Init(EN2Z_GPIO, &GPIO_InitStructure);  
}


