#include "interrupt.h"
#include "mcctl.h"
#include "tim.h"
#include "usart.h"
#include "mcuart.h"
#include "adc.h"
#include "logic.h"
#include "key.h"
#include "fangbo.h"
#include "fangbo_nohall.h"
#include "at32_board.h"
#include "youshua.h"
#include "YSstatemachine.h"
#include "YSmcctl.h"
#include "YSlogic.h"

//外部变量声明
extern uint8_t NextCCWhall[7];
extern uint8_t NextCWhall[7];


//500us中断函数
void TIM_500us_Int(void){

	if(YSorWSFlag == 1 ){		//无刷计时
		
		Control_State_Scan();	//控制状态检测
		Logic_Fangbo();				//方波逻辑
		Logic_Fangbo2();			//方波逻辑2
		Ctl_1ms();						//控制500us		-->BUCK电路闭环计算
		Ctl_1ms2();						//控制500us		-->BUCK电路闭环计算
		
		
		
		
		
		App.FB.Cyc1msCnt ++;	
		if(App.FB.Cyc1msCnt >= 2){
			App.FB.Cyc1msCnt = 0;
			App.FB.Cyc1msFg = 1;
			App.Logic.Calculate_1msFlag = 1;			//1ms去计算一次电压电流
			if(App.FB.Cyc1msFg == 1){
				App.FB.Cyc1msFg = 0;
				//1ms 时间
				FangBo_1ms();	//方波1ms控制函数
				Logic_1ms();	//保护周期和ADC采样触发函数
				Pos_1ms();		//位置拖动参数1ms控制函数
				//500ms时间
				App.FB.Cyc500msCnt ++; //500ms
				if(App.FB.Cyc500msCnt >= 500){
					App.FB.Cyc500msCnt = 0;
					Logic_500ms();		//500ms控制函数
				}
			}
		}
		//方波公用，往复运动计时
		if((App.Logic.CtlMode==3)&&(App.FB.AllRun ==1||App.FB2.AllRun ==1)){
			App.Logic.Logic1msCnt++;
			if(App.Logic.Logic1msCnt>=2 ){
				App.Logic.Logic1msCnt=0;
				App.Logic.Mode31msFg = 1;				
			}
		}else{
			App.Logic.Logic1msCnt=0;
			App.Logic.Mode31msFg = 0;
		}
		
		//方波电机1充电时间计时
		if(App.FB.Status == HS_CHARGE){			//状态机位于充电状态
			App.FB.FB1ms_Cnt++;
			if(App.FB.FB1ms_Cnt>=2){
				App.FB.FB1ms_Cnt =0;
				App.FB.ChaTim1msFg =1;					//充电1ms标志
			}
		}else{
			App.FB.FB1ms_Cnt 	 = 0;
			App.FB.FB1ms_Cnt 	 = 0;
			App.FB.ChaTim1msFg = 0;						//不是充电则不进行计数
		}
		
		//方波电机1刹车计时
		if(App.FB.Status == HS_BRA){				//刹车状态
			//方波刹车时间计时
			App.FB.Bra1msCnt++;
			if(App.FB.Bra1msCnt >=2){
				App.FB.Bra1msCnt = 0;
				App.FB.Bra1msFg =1;
			}
		}else{
			App.FB.Bra1msCnt = 0 ;
			App.FB.Bra1msCnt = 0 ;
			App.FB.Bra1msFg  = 0 ;
			App.FB.BraTimCnt = 0 ;
		}
		
		//方波电机2充电时间计时
		if(App.FB2.Status == HS_CHARGE){			//状态机位于充电状态
			App.FB2.FB1ms_Cnt++;
			if(App.FB2.FB1ms_Cnt>=2){
				App.FB2.FB1ms_Cnt =0;
				App.FB2.ChaTim1msFg =1;					//充电1ms标志
			}
		}else{
			App.FB2.FB1ms_Cnt = 0;
			App.FB2.FB1ms_Cnt = 0;
			App.FB2.ChaTim1msFg =0;					//不是充电则不进行计数
		}
		
			//方波电机2刹车时间计时	
		if(App.FB2.Status == HS_BRA){				//刹车状态
			App.FB2.Bra1msCnt++;
			if(App.FB2.Bra1msCnt >=2){
				App.FB2.Bra1msCnt = 0;
				App.FB2.Bra1msFg =1;
			}
		}else{
			App.FB2.Bra1msCnt = 0 ;
			App.FB2.Bra1msCnt = 0 ;
			App.FB2.Bra1msFg  = 0 ;
			App.FB2.BraTimCnt = 0 ;
		}
		//电机种类发生变化，需要计数停止等待时间
		if(App.Logic.Brake_Kind == 2){
			App.Logic.Motor_Kind_Change_Tim_Cnt++;
			if(App.Logic.Motor_Kind_Change_Tim_Cnt >= 2){
				App.Logic.Motor_Kind_Change_Tim_Cnt  = 0;
				App.Logic.Motor_Kind_Change_Tim++;
			}
		}else{
			App.Logic.Motor_Kind_Change_Tim_Cnt = 0;
			App.Logic.Motor_Kind_Change_Tim = 0 ;
		}	
		//电机霍尔发生变化，选哟计数停止等待时间
		if(App.Logic.Brake_Kind == 4){
			App.Logic.Motor_Hall_Change_Tim_Cnt++;
			if(App.Logic.Motor_Hall_Change_Tim_Cnt>=2){
				App.Logic.Motor_Hall_Change_Tim_Cnt =0;
				App.Logic.Motor_Hall_Change_Tim++;
			}
		}else{
			App.Logic.Motor_Hall_Change_Tim = 0;
			App.Logic.Motor_Hall_Change_Tim_Cnt = 0;
		}
		
		
		
	}else if(YSorWSFlag == 2){	//有刷计时
		YSLog_Ctl();				//逻辑
		//有刷状态机
		YSMotor_Ctl1();			//通道1状态机
		YSMotor_Ctl2();			//通道2状态机
		App2.SysCtl.YS1msCnt++;
		if(App2.SysCtl.YS1msCnt >= 2){
			App2.SysCtl.YS1msCnt = 0;
			YS1msCtl();													//有刷1ms控制函数
			App2.SysCtl.YS500msCnt++;
			if(App2.SysCtl.YS500msCnt >=500){
				App2.SysCtl.YS500msCnt = 0;
				YS500msCtl();											//有刷500ms控制函数
			}
		}
	}else{
		
	}
	App2.SysPort.System1msCnt ++;					//系统计时
	if(App2.SysPort.System1msCnt >=2){
		App2.SysPort.System1msCnt = 0;			//1ms
		App2.SysCtl.CalAdcFg = 1;						//系统计算1ms
		App2.SysPort.SystemProtectFlag = 1;	//系统保护1ms
	}
}



void FangBo_1ms(void){

	if(App.Logic.Motor_Stop_StartFlag == 1){
		App.Logic.Motor_Stop_Start_Tim_Cnt ++;
		if(App.Logic.Motor_Stop_Start_Tim_Cnt >= MCPara[57]){
			App.Logic.Motor_Stop_Start_Tim_Cnt = 0;
			App.Logic.Motor_Stop_StartFlag = 0;
		}
	}
	
	if(App.Logic.Brake_Kind == 5){		//正在往返运动
		App.Logic.Brake_TimeCnt++;
	}else{
		App.Logic.Brake_TimeCnt =0;
	}
	
	App.FB.OpenLoop.OpenStart1msFg = 1;		//开环1ms标志 
	if(App.FB.Status == HS_START){
		if(App.FB.OpenLoop.OpenLoopTime < 30000){
			App.FB.OpenLoop.OpenLoopTime++;
		}
	}
	
	App.FB2.OpenLoop.OpenStart1msFg = 1;		//开环1ms标志 
	if(App.FB2.Status == HS_START){
		if(App.FB2.OpenLoop.OpenLoopTime < 30000){
			App.FB2.OpenLoop.OpenLoopTime++;
		}
	}
	
}

//无刷通道1——1ms控制
void Ctl_1ms(void){	
	HallSqu_Ctl();				//方波状态机	1ms一次
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 || App.Logic.CtlMode == 3){			//模式1/2/3的PWM控制在这里
		if(App.FB.Status == HS_BRA){		//在刹车状态，根据不同的刹车种类去进行不同力度的刹车
			if(App.Logic.VBusNowPWM >0){	//通过BUCK去刹车更加平稳
				switch (App.Logic.Brake_Kind){
					case  0:
						App.FB.Status = HS_STOP;
						break;
					case	1:
						App.Logic.VBusNowPWM -=MCPara[44];	//停机指令刹车PWM衰减速度	
						break;	
					case  2:
						App.Logic.VBusNowPWM -=MCPara[38];	//切换电机指令刹车PWM衰减速度	
						break;
					case	3:
						App.Logic.VBusNowPWM -=MCPara[41];	//切换方向指令刹车PWM衰减速度
						break;
					case	4:
						App.Logic.VBusNowPWM -=MCPara[39];	//切换Hall指令刹车PWM衰减速度
						break;
					case	5:
						App.Logic.VBusNowPWM -=MCPara[40];	//有Hall模式3指令刹车PWM衰减速度
						break;
					default:
						break;
				}
				
			}
			//限幅：以免超出PWM值范围
			if(App.Logic.VBusNowPWM >mcApp_VoltageP_PIParam.qOutMax){
				App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam.qOutMax ;
			}else if(App.Logic.VBusNowPWM < mcApp_VoltageP_PIParam.qOutMin){
				App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam.qOutMin ;
			}
//			PWMNow_text = App.Logic.VBusNowPWM;
			//直接输出到BUCK
			TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);
			
		}else{		//正常闭环
			if(App.FB.AllRun == 1){
				if(App.FB.Status == HS_RUN){
					Hall_Vbus_PI_Out();		//有Hall的电压环计算
				}else{					
				}
			}
		}
	}

}


//无刷通道2——1ms控制
void Ctl_1ms2(void){
	HallSqu_Ctl2();				//方波状态机	1ms一次
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 || App.Logic.CtlMode == 3){			//模式1/2/3的PWM控制在这里
		if(App.FB2.Status == HS_BRA){		//在刹车状态，根据不同的刹车种类去进行不同力度的刹车
			if(App.Logic.VBusNowPWM >0){  //通过BUCK去刹车更加平稳
				switch (App.Logic.Brake_Kind){
					case  0:
						App.FB.Status = HS_STOP;
						break;
					case	1:
						App.Logic.VBusNowPWM -=MCPara2[44];  	//停机指令刹车PWM衰减速度	
						break;	                             
					case  2:                               
						App.Logic.VBusNowPWM -=MCPara2[38];		//切换电机指令刹车PWM衰减速度	
						break;                               
					case	3:                               
						App.Logic.VBusNowPWM -=MCPara2[41];  	//切换方向指令刹车PWM衰减速度
						break;                               
					case	4:                               
						App.Logic.VBusNowPWM -=MCPara2[39];  	//切换Hall指令刹车PWM衰减速度
						break;
					case	5:
						App.Logic.VBusNowPWM -=MCPara2[40];	//切换Hall指令刹车PWM衰减速度
						break;				
					default:
						break;
				}
			}
			//限幅：以免超出PWM值范围
			if(App.Logic.VBusNowPWM >mcApp_VoltageP_PIParam2.qOutMax){
				App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam2.qOutMax ;
			}else if(App.Logic.VBusNowPWM < mcApp_VoltageP_PIParam2.qOutMin){
				App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam2.qOutMin ;
			}
//			PWMNow_text = App.Logic.VBusNowPWM  ;
			//直接输出到BUCK
			TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);
			
		}else{		//正常闭环
			if(App.FB2.AllRun == 1){
				if(App.FB2.Status == HS_RUN ){
					Hall_Vbus_PI_Out2();		//有Hall的电压环计算
				}			
			}else{
			}
		}	
	}else if(App.Logic.CtlMode == 4||App.Logic.CtlMode == 5){									//模式4/5的PWM控制和步数控制在这里

	}
}


void EXTI2_IRQHandler(void){
	if(EXTI_GetIntStatus(EXTI_Line2) != RESET){
		App.FB2.Cyc_Hall ++;		//周期
		App.FB2.Hall_Timer_Cnt = TMR_GetCounter(TMR5);
		TMR5->CNT = 0;

		if(App.FB2.Cyc_Hall >=6){
					App.FB2.Hall_Timer_Sum =App.FB2.Hall_Timer_Cnt +App.FB2.Hall_Timer_Sum;
					App.FB2.Hall_Timer_Sum_Now = App.FB2.Hall_Timer_Sum ;		//周期满传递值
					App.FB2.Hall_Timer_Sum = 0;
					App.FB2.Cyc_Hall = 0;
			}else{
			App.FB2.Hall_Timer_Sum =App.FB2.Hall_Timer_Cnt +App.FB2.Hall_Timer_Sum;
			}
		EXTI_ClearIntPendingBit(EXTI_Line2);  //清除中断标志
		}
}

void EXTI9_5_IRQHandler(void){
	if(EXTI_GetIntStatus(EXTI_Line6) != RESET){
		TIM_Break_Int();
		TIM_Break_Int2();
		TMR_SetCompare4(TMR2,0);
		EXTI_ClearIntPendingBit(EXTI_Line6);  ///<Clear the  EXTI line 0 pending bit
	}
	
}


//中断
void EXTI15_10_IRQHandler(void){

		if(EXTI_GetIntStatus(EXTI_Line10) != RESET){
			App.FB.Cyc_Hall ++;		//周期
			App.FB.Hall_Timer_Cnt = TMR_GetCounter(TMR5);
				TMR_SetCounter(TMR5,0);
				if(App.FB.Cyc_Hall >=6){
					App.FB.Hall_Timer_Sum =App.FB.Hall_Timer_Cnt +App.FB.Hall_Timer_Sum;
					App.FB.Hall_Timer_Sum_Now = App.FB.Hall_Timer_Sum ;
					App.FB.Hall_Timer_Sum = 0;
					App.FB.Cyc_Hall = 0;
				}else{
				App.FB.Hall_Timer_Sum =App.FB.Hall_Timer_Cnt +App.FB.Hall_Timer_Sum;
				}
			
			EXTI_ClearIntPendingBit(EXTI_Line10);  ///<Clear the  EXTI line 0 pending bit
		}
		if(EXTI_GetIntStatus(EXTI_Line11) != RESET){
			if(App.Logic.Motor_Kind == 1){
				App.FB.Cyc_Hall ++;
				App.FB.Hall_Timer_Cnt = TMR_GetCounter(TMR5);
				TMR_SetCounter(TMR5,0);
				if(App.FB.Cyc_Hall >=6){
					App.FB.Hall_Timer_Sum =App.FB.Hall_Timer_Cnt +App.FB.Hall_Timer_Sum;
					App.FB.Hall_Timer_Sum_Now = App.FB.Hall_Timer_Sum ;
					App.FB.Hall_Timer_Sum = 0;
					App.FB.Cyc_Hall = 0;
				}else{
				App.FB.Hall_Timer_Sum =App.FB.Hall_Timer_Cnt +App.FB.Hall_Timer_Sum;
				}
			}else if(App.Logic.Motor_Kind == 2){
				App.FB2.Cyc_Hall ++;		//周期
				App.FB2.Hall_Timer_Cnt = TMR_GetCounter(TMR5);
				TMR_SetCounter(TMR5,0);

				if(App.FB2.Cyc_Hall >=6){
					App.FB2.Hall_Timer_Sum =App.FB2.Hall_Timer_Cnt +App.FB2.Hall_Timer_Sum;
					App.FB2.Hall_Timer_Sum_Now = App.FB2.Hall_Timer_Sum ;		//周期满传递值
					App.FB2.Hall_Timer_Sum = 0;
					App.FB2.Cyc_Hall = 0;
				}else{
				App.FB2.Hall_Timer_Sum =App.FB2.Hall_Timer_Cnt +App.FB2.Hall_Timer_Sum;
				}
				
			}
			EXTI_ClearIntPendingBit(EXTI_Line11);  ///<Clear the  EXTI line 0 pending bit
		}
		if(EXTI_GetIntStatus(EXTI_Line12) != RESET){
			if(App.Logic.Motor_Kind == 1){
				App.FB.Cyc_Hall ++;
				App.FB.Hall_Timer_Cnt = TMR_GetCounter(TMR5);
				TMR_SetCounter(TMR5,0);
				if(App.FB.Cyc_Hall >=6){
					App.FB.Hall_Timer_Sum =App.FB.Hall_Timer_Cnt +App.FB.Hall_Timer_Sum;
					App.FB.Hall_Timer_Sum_Now = App.FB.Hall_Timer_Sum ;
					App.FB.Hall_Timer_Sum = 0;
					App.FB.Cyc_Hall = 0;
				}else{
				App.FB.Hall_Timer_Sum =App.FB.Hall_Timer_Cnt +App.FB.Hall_Timer_Sum;
				}
			}else if(App.Logic.Motor_Kind == 2){
				App.FB2.Cyc_Hall ++;		//周期
				App.FB2.Hall_Timer_Cnt = TMR_GetCounter(TMR5);
				TMR_SetCounter(TMR5,0);

				if(App.FB2.Cyc_Hall >=6){
					App.FB2.Hall_Timer_Sum =App.FB2.Hall_Timer_Cnt +App.FB2.Hall_Timer_Sum;
					App.FB2.Hall_Timer_Sum_Now = App.FB2.Hall_Timer_Sum ;		//周期满传递值
					App.FB2.Hall_Timer_Sum = 0;
					App.FB2.Cyc_Hall = 0;
				}else{
				App.FB2.Hall_Timer_Sum =App.FB2.Hall_Timer_Cnt +App.FB2.Hall_Timer_Sum;
				}
			}
			EXTI_ClearIntPendingBit(EXTI_Line12);  ///<Clear the  EXTI line 0 pending bit
		}
	}
	
	

//刹车中断
void TIM_Break_Int(void){
	PWM_3l_Stop();
	App.FB.AllRun = 0;
	App.FB.Err = E_FAIL;
}

void TIM_Break_Int2(void){
	PWM_3l_Stop2();
	App.FB2.AllRun = 0;
	App.FB2.Err = E_FAIL;
}

//ADC1中断 注入通道转换完成
void ADC1_2_IRQHandler(void){
	if(ADC_GetINTStatus(ADC1,ADC_INT_JEC)!=RESET){
		if(YSorWSFlag == 1){
			HallSqu_ADCISRTasks();	//无刷方波的ADC中断
		}else if(YSorWSFlag == 2){
			Youshua_ADCISRTasks();	//有刷的ADC中断
		}
		
		ADC_ClearINTPendingBit(ADC1, ADC_INT_JEC);
	}
}

//500us中断
void TMR6_GLOBAL_IRQHandler(void){ 
  if(TMR_GetINTStatus(TMR6,TMR_INT_Overflow) == SET){
		TIM_500us_Int();
		TMR_ClearITPendingBit(TMR6, TMR_INT_Overflow);
  }
}


//方波有霍尔，换相测速计时
void TMR5_GLOBAL_IRQHandler(void){
	 if(TMR_GetINTStatus(TMR5,TMR_INT_Overflow) == SET){
		 if(App.Logic.Motor_Kind == 1){
			 App.FB.Spd_Enable = 1;
		 }
		 if(App.Logic.Motor_Kind == 2){
			 App.FB2.Spd_Enable = 1;
		 }
		 //太久没有换相应该报错
		TMR_ClearITPendingBit(TMR5, TMR_INT_Overflow);
  }
}


void TMR3_GLOBAL_IRQHandler(void){
	//无Hall换相
	if(TMR_GetINTStatus(TMR3,TMR_INT_Overflow) == SET){
			if(App.FB.Status == HS_START||App.FB.Status == HS_RUN||App.FB.Status == HS_BRA){
				if(App.FB.Dir.Now == CW){
					OpenLoop_check_changeCW(App.FB.OpenLoop.HallVal);		//无霍尔CW启动检测
				}else{
					OpenLoop_check_changeCCW(App.FB.OpenLoop.HallVal);	//无霍尔CCW启动检测
				}
				App.FB.OpenLoop.Spd_NotHallCnt ++;										//5us计时
				if(App.FB.OpenLoop.Spd_NotHallCnt > 80000){
					App.FB.OpenLoop.Spd_NotHallCnt = 0;
					App.FB.Err = E_RUNSTALL;		//运行中被堵故障，启动不好会触发
				}
			}
			
			if(App.FB2.Status == HS_START||App.FB2.Status == HS_RUN||App.FB2.Status == HS_BRA){
				if(App.FB2.Dir.Now == CW){
					OpenLoop_check_changeCW2(App.FB2.OpenLoop.HallVal);			//无霍尔CW启动检测
				}else{                                                    
					OpenLoop_check_changeCCW2(App.FB2.OpenLoop.HallVal);    //无霍尔CCW启动检测
				}                                                         
				App.FB2.OpenLoop.Spd_NotHallCnt ++;						            //5us计时 
				if(App.FB2.OpenLoop.Spd_NotHallCnt > 80000){
					App.FB2.OpenLoop.Spd_NotHallCnt = 0;
					App.FB2.Err = E_RUNSTALL;		//运行中被堵故障，启动不好会触发
				}
			}			
			TMR_ClearITPendingBit(TMR3, TMR_INT_Overflow);
	}
}


//刹车中断
void TMR1_BRK_TMR9_IRQHandler(void){
	if(TMR_GetINTStatus(TMR1, TMR_INT_Break) == SET){
		TMR_SetCompare4(TMR2,0);
		TIM_Break_Int();
		TIM_Break_Int2();
    TMR_ClearITPendingBit(TMR1, TMR_INT_Break);
  }
}

//刹车中断
void TMR8_BRK_TMR12_IRQHandler(void){
	if(TMR_GetINTStatus(TMR8, TMR_INT_Break) == SET){
		TMR_SetCompare4(TMR2,0);
		TIM_Break_Int();
		TIM_Break_Int2();

    TMR_ClearITPendingBit(TMR8, TMR_INT_Break);
  }
}




#if _AllUart == 1
	#if _Uart1 == 1
	//串口1中断
	void USART1_IRQHandler(void){
		if(USART_GetITStatus(USART1, USART_INT_IDLEF) != RESET){
			USART_ClearITPendingBit(USART1, USART_INT_IDLEF);
			
			Uart_IRQHandler(USART1,Uart1_DMA_CH,&SerUart1);
		}	
	}
	#endif

	#if _Uart2 == 1
	//串口2中断
	void USART2_IRQHandler(void){
		if(USART_GetITStatus(USART2, USART_INT_IDLEF) != RESET){
			USART_ClearITPendingBit(USART2, USART_INT_IDLEF);
			
			Uart_IRQHandler(USART2,Uart2_DMA_CH,&SerUart2);
		}	
	}
	#endif

	#if _Uart3 == 1
	//串口3中断
	void USART3_IRQHandler(void){
		if(USART_GetITStatus(USART3, USART_INT_IDLEF) != RESET){	//如果是空闲中断，就去读取数据，并且到解析函数去解析
			USART_ClearITPendingBit(USART3, USART_INT_IDLEF);
			Uart_IRQHandler(USART3,Uart3_DMA_CH,&SerUart3);
		}	
		
		if(USART_GetITStatus(USART3, USART_INT_ERR) != RESET){		//识别到错误中断，那么这一帧就不做解析
			USART_ClearITPendingBit(USART3, USART_INT_ERR);					//清除错误中断标志位，此处也可以加入串口错误保护程序，
			//为下一次接收做准备																			//此处不做保护处理，直接清除错误并且重置DMA接收
			Uart3_DMA_CH->CHCTRL &= 0xFFFFFFFE;											//重新打开DMA接收，让下个数据从接收数组第一个开始接收，不在原本数据后面累加。
			Uart3_DMA_CH->TCNT = MAX_RXBUF;
			Uart3_DMA_CH->CHCTRL |= 0x00000001;
		}
			
		if(USART_GetITStatus(USART3, USART_INT_ORERR) != RESET){		//识别到溢出中断，
			USART_ClearITPendingBit(USART3, USART_INT_ORERR);					
			//为下一次接收做准备
			Uart3_DMA_CH->CHCTRL &= 0xFFFFFFFE;
			Uart3_DMA_CH->TCNT = MAX_RXBUF;
			Uart3_DMA_CH->CHCTRL |= 0x00000001;
		}
		
		if(USART_GetITStatus(USART3, USART_INT_NERR) != RESET){		//识别到噪声干扰错误中断，
			USART_ClearITPendingBit(USART3, USART_INT_NERR);					
			//为下一次接收做准备
			Uart3_DMA_CH->CHCTRL &= 0xFFFFFFFE;
			Uart3_DMA_CH->TCNT = MAX_RXBUF;
			Uart3_DMA_CH->CHCTRL |= 0x00000001;
		}
		
		if(USART_GetITStatus(USART3, USART_INT_FERR) != RESET){		//识别到帧错误中断，
			USART_ClearITPendingBit(USART3, USART_INT_FERR);					
			//为下一次接收做准备
			Uart3_DMA_CH->CHCTRL &= 0xFFFFFFFE;
			Uart3_DMA_CH->TCNT = MAX_RXBUF;
			Uart3_DMA_CH->CHCTRL |= 0x00000001;
		}
	}
	#endif
	
	#if _Uart4 == 1
	//串口4中断
	void UART4_IRQHandler(void){
		if(USART_GetITStatus(UART4, USART_INT_IDLEF) != RESET){
			USART_ClearITPendingBit(UART4, USART_INT_IDLEF);
			
			Uart_IRQHandler(UART4,Uart4_DMA_CH,&SerUart4);
		}	
	}
	#endif

	
	#if _Uart5 == 1
	//串口5中断
	void UART5_IRQHandler(void){
		if(USART_GetITStatus(UART5, USART_INT_IDLEF) != RESET){
			USART_ClearITPendingBit(UART5, USART_INT_IDLEF);
			
			Uart_IRQHandler(UART5,Uart5_DMA_CH,&SerUart5);
		}	
	}
	#endif
	
	#if _Uart6 == 1
	//串口6中断
	void USART6_IRQHandler(void){
		if(USART_GetITStatus(USART6, USART_INT_IDLEF) != RESET){
			USART_ClearITPendingBit(USART6, USART_INT_IDLEF);
			
			Uart_IRQHandler(USART6,Uart6_DMA_CH,&SerUart6);
		}	
	}
	#endif
	
#endif







//void TMR2_GLOBAL_IRQHandler(void){
//	
//  TMR_ClearITPendingBit(TMR2, TMR_INT_CC1);
//	TMR_ClearITPendingBit(TMR2, TMR_INT_Overflow);
//	
//	TMR2 -> CNT = 0;
//}


//IPM 错误
#if _IPM == 1
void IPM_EXTI_IRQHandler(void){
	if(EXTI_GetIntStatus(IPM_Exti_Line) != RESET){
			TIME_PWM_Stop_3Channel();
			App.FB.Err = E_IPM;
//		}
		
		EXTI_ClearIntPendingBit(IPM_Exti_Line);  ///<Clear the  EXTI line 0 pending bit
	}
}
#endif






//编码器 Z 轴
#if _ENZ == 1
void ENZ_EXTI_IRQHandler(void){
	if(EXTI_GetIntStatus(ENZ_Exti_Line) != RESET){
		
		Enoc_Z_Offset();
		
		EXTI_ClearIntPendingBit(ENZ_Exti_Line);  ///<Clear the  EXTI line 0 pending bit
	}
}
#endif



//编码器 Z2 轴
#if _ENZ2 == 1
void ENZ2_EXTI_IRQHandler(void){
	if(EXTI_GetIntStatus(ENZ2_Exti_Line) != RESET){
		
		Enoc_Z2_Offset();
		
		EXTI_ClearIntPendingBit(ENZ2_Exti_Line);  ///<Clear the  EXTI line 0 pending bit
	}
}
#endif



//接近开关
#if _PrsIN == 1
void PrsIN_EXTI_IRQHandler(void){
	if(EXTI_GetIntStatus(PrsIN_Exti_Line) != RESET){
		EXTI_ClearIntPendingBit(PrsIN_Exti_Line);  ///<Clear the  EXTI line 0 pending bit
	}
}
#endif














