#include "mcLib.h"
#include "tim.h"
#include "mcctl.h"
#include "stdint.h"
#include "mcflash.h"
#include "userpara.h"
#include "adc.h"
#include "usart.h"
#include "math.h"
#include "gpio.h"
#include "mcuart.h"
#include "key.h"
#include "dac.h"
#include "logic.h"
#include "at32_board.h"
#include "protect.h"
#include "canctl.h"
#include "pospid.h"
#include "fangbo.h"
#include "fangbo_nohall.h"
#include "youshua.h"
#include "YSprotect.h"
#include "driver_param.h"


APP_TYPEDEF App;										//无刷全部变量	
__IO uint16_t ADC1Value[3];					//ADC采样数组
uint16_t SPStatus[SpeNum] = {0};		//状态数组

//状态参数
int32_t MCStatus[StatusNum]={
	0,              //0,电机当前状态
	0,              //1,电机当前转速
	0,              //2,控制器输出电流
	0,              //3,控制器母线电压值
	0,              //4,控制器错误值
	0,              //5,电机当前温度
	0,              //6,控制器当前温度
	0,              //7,电机当前转矩
	0,              //8,电机当前功率
	0,              //9,在485模式下是否检测速度
	0,              //10,检查速度的最大值
};

//无刷通道1参数	//参数寄存器地址0x10xx
unsigned short MCPara[ParaNum];
uint16_t SMCPara[ParaNum] = {
	1,              //0-地址

	1,              //1-该通道电机极对数				系统保护参数以01地址设置为准
	80,           	//2-欠压保护值 /10V					系统保护参数以01地址设置为准					
	3000,          	//3-欠压保护时间阈值 ms			系统保护参数以01地址设置为准
	350,           	//4-过压保护值 /10V					系统保护参数以01地址设置为准
	1000,        	 	//5-过压保护时间阈值 ms			系统保护参数以01地址设置为准

	85,          	  //6-驱动器过温保护温度度		系统保护参数以01地址设置为准
	3000,           //7-驱动器过温保护时间ms		系统保护参数以01地址设置为准

	800,            //8-往复运动最高转速			--倍率100	

	90,            	//9-	该电流采样滤波值（0~100），越大滤波越深			系统保护参数以01地址设置为准
	20,             //10- 通讯保护时长，多久没有实现通讯就报错/10 	S		系统保护参数以01地址设置为准
	850,          	//11- 硬件保护值			//700
			                  
	                      
	50,              //12- 电机最低允许转速			转速倍率10！
	8000,            	//13- 电机最高允许转速			转速倍率10!
	100,            	//14- 堵转保护值rpm                     
	3000,            	//15- 堵转保护时间ms
	
	5,              	//16-	往复运动堵转保护低于保护值时计数衰减值。越小越敏感

	450,              //17-	长时间大电流堵转保护判定电流 /100A		
	30000,						//18- 长时间大电流堵转保护判定时间 ms
	
	190,         			//19-有Hall单向运动过流保护值/100A
	300,          		//20-有Hall单向运动过流保护时间阈值 ms
	210,         			//21-有Hall往复运动过流保护值/100A
	500,         			//22-有Hall往复运动过流保护时间阈值 ms		 
	200,							//23-无Hall单向运动过流保护值/100A
	30,							//24-无Hall单向运动过流保护时间阈值 ms
	
	//无感启动参数		新板子加电感
	2500,	             //25- 启动拖动最终转速		3000
	60,	               //26- 启动拖动步进        40
	0,	             //27- 启动拖动起始PWM			1100				
	500,	             //28- 启动拖动最大的PWM  	2000
	40,	             	 //29- 启动拖动PWM爬升			1
	100,		           //30- 开环拖动最少时间  500
	//PID参数
	800,	          //31- BUCK电路 PI参数P		200			有Hall单向运动			
	5,	            //32- BUCK电路 PI参数I	   5    	有Hall单向运动
	60,	            //33- BUCK电路 PI参数D     2			有Hall单向运动
	
//	//无感启动参数		旧板子无电感
//	2000,	           //25- 启动拖动最终转速		
//	40,	             //26- 启动拖动步进
//	0,	             //27- 启动拖动起始PWM							
//	900,	           //28- 启动拖动最大的PWM
//	5,	             //29- 启动拖动PWM爬升			
//	100,	           //30- 开环拖动最少时间
//	//PID参数
//	300,	            //31- BUCK电路 PI参数P							
//	5,	           	  //32- BUCK电路 PI参数I	       
//	25,	          	  //33- BUCK电路 PI参数D
	
	7000,	            //34- PWM输出的最大值			0-7500 
	0,	        			//35- PWM输出的最小值			0-7500
	300,	       		  //36-	同步整流闭环爬升									
	300,	            //37- 同步整流闭环下降	
	//刹车参数
	8,	        		//38-切换电机刹车PWM下降-------------有关:有转速切换电机的时候的刹车力度--15
	15,	        		//39-切换霍尔刹车PWM下降-------------有关:有转速切换霍尔的时候的刹车力度
	200,						//40-有Hall的往复运动刹车PWM下降-----有关:往复运动时切换方向时的刹车;								
	15,							//41-切换电机转向的PWM下降------有关：有转速切换电机转向的时候的刹车力度		
	1,							//42-切换霍尔之后需要等待的时间ms--电机1和电机2统一
	1,							//43-切换电机之后需要等待的时间ms--电机1和电机2统一
	15,							//44-停止指令刹车PWM下降-------------有关:停止指令刹车的力度
	10,							//45-判定刹车完成PWM值
	
	1000,							//46-BUCK电路PI参数P						--往复运动的PI参数P	 500	
	6,								//47-BUCK电路PI参数I						--往复运动的PI参数I	  6
	20,								//48-BUCK电路PI参数D						--往复运动的PI参数D	  50
	
	//位置控制功能参数
	0,							//49-	减速机的型号  -0 ：4.957647 ; -1 :1.96 ; -2 :2.0;
	100,						//50-	刹车完成后延时多少ms等待转子完全停止后，再拖动	ms
	10,							//51-	停止时，步进的时间间隔	ms
	10,							//52-	停止时，位置到位之后，保持位置的时间 ms
	800,						//53-	停止时，拖动到指定位置的力量
	
	10,							//54-	模式4/5，步进间隔时间
	10,							//55-	模式4/5，步数运行玩之后，保持位置时间ms
	1000,						//56-	模式4/5，拖动力量
	
	400,						//57-	发送停止指令后再次允许启动的时间	-- 电机1和电机2统一
	
	500,						//58- 恒流启动值/100		0.2A		//电机1和电机2统一
	400,						//59- 恒流PID P值
	40,							//60- 恒流PID I值
	
	500,						//61- BUCK电路PI参数P				 无Hall单向运动PID参数 新增!
	10,							//62- BUCK电路PI参数I         无Hall单向运动PID参数 新增！
	0,							//63- BUCK电路PI参数D         无Hall单向运动PID参数 新增！
	
	5,	       			//64-	同步整流闭环爬升，加速度			无Hall		新增!						
	5,	          	//65- 同步整流闭环下降，减速度			无Hall     新增！ 
	
	100,						//66- 有刷刀具回位最长时间		ms
	
};


//无刷通道2参数	//参数寄存器地址0x11xx
unsigned short MCPara2[ParaNum];
uint16_t SMCPara2[ParaNum] = {
	2,              //0-地址

	1,              //1-该通道电机极对数				系统保护参数以01地址设置为准
	80,           	//2-欠压保护值 /10V					系统保护参数以01地址设置为准					
	3000,          	//3-欠压保护时间阈值 ms			系统保护参数以01地址设置为准
	350,           	//4-过压保护值 /10V					系统保护参数以01地址设置为准
	1000,        	 	//5-过压保护时间阈值 ms			系统保护参数以01地址设置为准
	85,          	  //6-驱动器过温保护温度度		系统保护参数以01地址设置为准
	3000,           //7-驱动器过温保护时间ms		系统保护参数以01地址设置为准

	800,           	 //8-往复运动最高转速			--倍率100	
	90,            	 //9-	NC
	0,             	 //10- 接下来参数是该通道独立的参数，
	150,          	 //11- 硬件保护值
			                  
	50,             //12- 电机最低允许转速			转速倍率10！
	8000,            //13- 电机最高允许转速			转速倍率10!
	100,             //14- 堵转保护值rpm                     
	1000,            //15- 堵转保护时间ms
	5,             	 //16-	往复运动堵转保护低于保护值时计数衰减值。越小越敏感

	450,              //17-	长时间大电流堵转保护判定电流 /100A		
	30000,						//18- 长时间大电流堵转保护判定时间 ms
	
	190,         			//19-有Hall单向运动过流保护值/100A
	500,          		//20-有Hall单向运动过流保护时间阈值 ms
	210,         			//21-有Hall往复运动过流保护值/100A
	500,         			//22-有Hall往复运动过流保护时间阈值 ms		 
	200,							//23-无Hall单向运动过流保护值/100A
	500,							//24-无Hall单向运动过流保护时间阈值 ms
	
	//无感启动参数		新板子加电感
	2500,	             //25- 启动拖动最终转速		3000
	60,	               //26- 启动拖动步进        40
	0,	             //27- 启动拖动起始PWM			1100				
	500,	             //28- 启动拖动最大的PWM  	2000
	40,	             	 //29- 启动拖动PWM爬升			1
	100,		           //30- 开环拖动最少时间  500
	
	//PID参数
	800,	          //31- BUCK电路 PI参数P		200					
	5,	            //32- BUCK电路 PI参数I	   5    
	60,	            //33- BUCK电路 PI参数D     2
	
//	//无感启动参数		旧板子无电感
//	2000,	           //25- 启动拖动最终转速		
//	40,	             //26- 启动拖动步进
//	0,	             //27- 启动拖动起始PWM							
//	900,	           //28- 启动拖动最大的PWM
//	5,	             //29- 启动拖动PWM爬升			
//	100,	           //30- 开环拖动最少时间
//	//PID参数
//	300,	            //31- BUCK电路 PI参数P							
//	5,	            	//32- BUCK电路 PI参数I	       
//	25,	            	//33- BUCK电路 PI参数D
	
	7000,	            //34- PWM输出的最大值			0-7500 
	0,	        			//35- PWM输出的最小值			0-7500
	300,	       		  //36-	同步整流闭环爬升，加速度			有Hall								
	300,	            //37- 同步整流闭环下降，减速度			有Hall
	//刹车参数
	8,	        		//38-切换电机刹车PWM下降-------------有关:有转速切换电机的时候的刹车力度
	15,	        		//39-切换霍尔刹车PWM下降-------------有关:有转速切换霍尔的时候的刹车力度
	200,						//40-有Hall的往复运动刹车PWM下降-----有关:往复运动时切换方向时的刹车;								
	15,							//41-切换电机转向的PWM下降-----------有关：有转速切换电机转向的时候的刹车力度		
	1,							//42-切换霍尔之后需要等待的时间ms----电机1和电机2统一
	100,						//43-切换电机之后需要等待的时间ms----电机1和电机2统一
	15,							//44-停止指令刹车PWM下降-------------有关:停止指令刹车的力度
	10,							//45-判定刹车完成PWM值

	1000,							//46-BUCK电路PI参数P						--往复运动的PI参数P	 500	
	6,								//47-BUCK电路PI参数I						--往复运动的PI参数I	  6
	20,								//48-BUCK电路PI参数D						--往复运动的PI参数D	  50
	
	//位置控制功能参数
	0,							//49-	减速机的型号  -0 ：4.957647 ; -1 :1.96 ; -2 :2.0;
	100,						//50-	刹车完成后延时多少ms等待转子完全停止后，再拖动	ms
	10,							//51-	停止时，步进的时间间隔	ms
	10,							//52-	停止时，位置到位之后，保持位置的时间 ms
	800,						//53-	停止时，拖动到指定位置的力量
	
	10,							//54-	模式4/5，步进间隔时间
	10,							//55-	模式4/5，步数运行玩之后，保持位置时间ms
	1000,						//56-	模式4/5，拖动力量
	400,						//57-	发送停止指令后再次允许启动的时间	-- 电机1和电机2统一以电机1参数为准
	200,						//58- 恒流启动值/100		0.2A						-- 电机1和电机2统一以电机1参数为准
	400,						//59- 恒流PID P值												-- 电机1和电机2统一以电机1参数为准
	40,							//60- 恒流PID I值                       -- 电机1和电机2统一以电机1参数为准
	

	500,						//61- BUCK电路PI参数P				 无Hall单向运动PID参数 新增!
	10,							//62- BUCK电路PI参数I         无Hall单向运动PID参数 新增！
	0,							//63- BUCK电路PI参数D         无Hall单向运动PID参数 新增！
	
	5,	       			//64-	同步整流闭环爬升，加速度			无Hall		新增!						
	5,	          	//65- 同步整流闭环下降，减速度			无Hall     新增！ 
	
	
	100,						//66- 有刷刀具回位最长时间		ms
		
	
};                                                          


//软件初始化函数
void Soft_Init(void){
	uint16_t i;
	//上电先读取n次电压和温度，
	for(i = 0; i < 100;i ++){
		//母线电压
		App.FB.Prot.BusVol = (App.FB.Prot.BusVol + (float)get_vbus_adc_value * VOLTAGE_ADC_TO_PHY_RATIO) / 2.0f;
		//温度
		App.FB.Prot.Temper = App.FB.Prot.Temper * 0.9f + 0.1f * temp_data(ADC1Value[0],10000);
		//Buck调压电压
		App.FB.Prot.BusVolP = (App.FB.Prot.BusVolP  + (float)get_Pvcc_adc_value * VOLTAGEP_ADC_PVCC_PHY_RATIO) / 2.0f;
		Delay_ms(2);
	}
	//保护相关初始化
	Protect_Init();			//无刷保护
	YSProtect_Init();		//有刷保护
	//软件初始化
	Logic_Sotf_Init();
	//方波初始化
	Fangbo_Init();
	//电机1和电机2速度环PI初始化
	VoltageP1_Init();
	VoltageP2_Init();
	//位置初始化
	Pos_Init();
}

//硬件初始化函数
void Hard_Init(void){
	/**************************RCC时钟*************************************************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);          //配置中断优先组模式：只有主优先级，没有从优先级
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);     //复用AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);    //GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);		 //GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOC, ENABLE);		 //GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOD, ENABLE);		 //GPIOD时钟
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOE, ENABLE);		 //GPIOE时钟
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOF, ENABLE);		 //GPIOF时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);       //DMA1时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA2, ENABLE);			 //DMA2时钟

	/**************************System***************************************************/
	//调试口选择
	GPIO_PinsRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	//滴答时钟初始化  
	Delay_init();       
	//系统GPIO初始化	
	GPIO_Init_App();			
	/****************************M1***************************************************/
	//FLASH地址
	App.Logic.MotorFlashAddr1 = TEST_FLASH_ADDRESS_START;
	//读取模式参数
	F4Flash_SMCPara_Read(App.Logic.MotorFlashAddr1,MCPara,ParaNum,SMCPara);
	/****************************M2***************************************************/
	//FLASH地址
	App.Logic.MotorFlashAddr2 = TEST_FLASH_ADDRESS_START2;
	//读取模式参数
	F4Flash_SMCPara_Read(App.Logic.MotorFlashAddr2,MCPara2,ParaNum,SMCPara2);
	DriverParam_Init();
	/**************************硬件初始化***************************************************/
	//DAC初始化
	DAC_Init_App();
	//所以TIM初始化
	TIM_Init_App();
	//ADC初始化
	ADC_Init_App();
	//串口初始化
	UART_Init();
	//无感方波电机1过零点检测初始化
	GPIO_Noninductive1_Init();
	//无感方波电机2过零点检测初始化
	GPIO_Noninductive2_Init();
	//BUCK从0启动
	TMR_SetCompare4(TMR2,0);
	//使能BUCK
	PVCC_PWM_ON;
	//等待10ms初始化完成
	Delay_ms(10);
}


//位置控制初始化函数
void Pos_Init(void){
	App.Pos.DragTim = MCPara[54];			//拖动时间
	App.Pos.LocatedTim = MCPara[55];	//定位时间
	App.Pos.PosNow1 = 0x1F;						//未判定状态
	App.Pos.PosNow2 = 0x1F;						//未判定状态

	//通道1根据不同的刀具选择不同的减速比
	if(MCPara[49] == 0){
		App.Pos.ReduceRatio1 =	4.957647f;				
	}else if(MCPara[49] == 1) {
		App.Pos.ReduceRatio1 = 1.960f;
	}else if(MCPara[49] == 2){
		App.Pos.ReduceRatio1 = 2.00f;
	}
	//通道2根据不同的刀具选择不同的减速比
	if(MCPara2[49] == 0){
		App.Pos.ReduceRatio2 = 4.957647f;
	}else if(MCPara2[49] == 1){
		App.Pos.ReduceRatio2 = 1.960f;
	}else if(MCPara2[49] == 2){
		App.Pos.ReduceRatio1 = 2.00f;
	}
	//计算他每一个Hall对应外圈的角度，以外围一圈为1作为基准
	App.Pos.OutSideAngle1 = 1/(App.Pos.ReduceRatio1*6);			
	App.Pos.OutSideAngle2 = 1/(App.Pos.ReduceRatio2*6);			
	
	App.Pos.WaitStopTim1 = MCPara[50];			//刹车后需要等待多久才能位置回拉
	App.Pos.WaitStopTim2 = MCPara2[50];			//刹车后需要等待多久才能位置回拉
	
	//记录这个位置，后续每次转动都需要停止在这个位置,上电默认此刻位置：
	App.Pos.OutSidePosition1 = 0;		
	App.Pos.LockFlag1 = 1;
	
	//记录这个位置，后续每次转动都需要停止在这个位置，上电默认此刻位置：
	App.Pos.OutSidePosition2 = 0;		
	App.Pos.LockFlag2 = 1;	
	
}



//位置控制初始化函数
void Pos_Uart_Init(void){
	App.Pos.DragTim = MCPara[54];			//拖动时间
	App.Pos.LocatedTim = MCPara[55];	//定位时间
	App.Pos.PosNow1 = 0x1F;						//未判定状态
	App.Pos.PosNow2 = 0x1F;						//未判定状态

	//通道1根据不同的刀具选择不同的减速比
	if(MCPara[49] == 0){
		App.Pos.ReduceRatio1 =	4.957647f;				
	}else if(MCPara[49] == 1) {
		App.Pos.ReduceRatio1 = 1.960f;
	}else if(MCPara[49] == 2){
		App.Pos.ReduceRatio1 = 2.00f;
	}
	//通道2根据不同的刀具选择不同的减速比
	if(MCPara2[49] == 0){
		App.Pos.ReduceRatio2 = 4.957647f;
	}else if(MCPara2[49] == 1){
		App.Pos.ReduceRatio2 = 1.960f;
	}else if(MCPara2[49] == 2){
		App.Pos.ReduceRatio1 = 2.00f;
	}
	//计算他每一个Hall对应外圈的角度，以外围一圈为1作为基准
	App.Pos.OutSideAngle1 = 1/(App.Pos.ReduceRatio1*6);			
	App.Pos.OutSideAngle2 = 1/(App.Pos.ReduceRatio2*6);			
	
	App.Pos.WaitStopTim1 = MCPara[50];			//刹车后需要等待多久才能位置回拉
	App.Pos.WaitStopTim2 = MCPara2[50];			//刹车后需要等待多久才能位置回拉
	
	//不更新默认位置
}



