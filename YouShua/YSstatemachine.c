#include "YSstatemachine.h"
#include "youshua.h"
#include "YSmcctl.h"
#include "mcuart.h"

//通道1等待状态
void Ctl_Wait(void){
	if((App2.Ch1.Start == 1)&&(App2.Log.BreakSta == 0)){
		App2.Ch1.Status = CTLS_INIT;	//允许启动，进入初始化
	}
	if(App2.Err > 0){
		App2.Ch1.Status = CTLS_ERROR;	//错误，进入错误处理
	}
}

//通道2等待状态
void Ctl_Wait2(void){
	if((App2.Ch2.Start == 1)&&(App2.Log.BreakSta == 0)){
		App2.Ch2.Status = CTLS_INIT;	//允许启动，进入初始化
	}
	if(App2.Err > 0){
		App2.Ch2.Status = CTLS_ERROR;	//错误，进入错误处理
	}
}


//通道1初始化状态
void Ctl_Init(void){

	//通道1有刷刀头电阻电感初始化
	if(App2.Log.Drill == 1){		//通道1刀头1
		//刀头1电阻电感和转换系数和保护电流
		App2.Ch1.Emf.qRs = (float)YSPara1[15] / 1000.0f;
		App2.Ch1.Emf.qLsDtq = (float)YSPara1[16] * PWM_FREQUENCY / 1000000.0f;
		App2.Ch1.Emf.qSpdxisu = (float)YSPara1[17];
		App2.Ch1.Emf.qRsxisu = (float)YSPara1[18] / 100.0f;
		//刀头1过流保护值
		if(App2.Log.CtlMode == 3){				//往复运动的过流保护值
			App2.Ch1.Prot.OverCurrent = App2.Ch1.Prot.OverCurrent2;
			App2.Ch1.Prot.CurCnt			= App2.Ch1.Prot.CurCnt2;
		}else{														//单向运动的过流保护值
			App2.Ch1.Prot.OverCurrent = App2.Ch1.Prot.OverCurrent1;
			App2.Ch1.Prot.CurCnt			= App2.Ch1.Prot.CurCnt1;
		}
	}
	else if(App2.Log.Drill == 2){//通道1刀头2
		//刀头2电阻电感和转换系数
		App2.Ch1.Emf.qRs = (float)YSPara1[27] / 1000.0f;
		App2.Ch1.Emf.qLsDtq = (float)YSPara1[28] * PWM_FREQUENCY / 1000000.0f;
		App2.Ch1.Emf.qSpdxisu = (float)YSPara1[29];
		App2.Ch1.Emf.qRsxisu = (float)YSPara1[30] / 100.0f;
		//刀头2过流保护值
		if(App2.Log.CtlMode == 3){			//往复运动的过流保护值
			App2.Ch1.Prot.OverCurrent = App2.Ch1.Prot.OverCurrent4;
			App2.Ch1.Prot.CurCnt			= App2.Ch1.Prot.CurCnt4;
		}else{													//单向运动的过流保护值
			App2.Ch1.Prot.OverCurrent = App2.Ch1.Prot.OverCurrent3;
			App2.Ch1.Prot.CurCnt			= App2.Ch1.Prot.CurCnt3;
		}		
	}
	//Log/PWM参数初始化
	App2.SysCtl.MaxSpdSet = (float)( PWM_PERIOD_COUNT * YSPara1[12] /1000.0f);	//最大PWM
	App2.SysCtl.MinSpdSet = 0;
	App2.Ch1.Dir.Now = App2.Ch1.Dir.Set;		//确认启动方向
	App2.Log.VbusRef = 0;										//速度环计算后的输出值，给电压环的参考值
	App2.Log.BreakSta = 0;									//初始化刹车为正常运行状态
	App2.Ch1.Tim.StopTimCnt = 0;
	//PID参数初始化
	YSPI_Init();
	//有刷电机TRM初始化
	MCPwm_Init1();								//初始化有刷电机1TMR
	//Buck调压初始化
	App2.Log.VbusNowPWM = 0;			//PWM爬坡清零
	App2.Log.VbusSetPWM = 0;			//PWM爬坡清0
	Vbus_Tim_ON();								//Buck电路开
	//允许启动
	App2.Ch1.Status = CTLS_OPENLOOP_RUN;		//跳转到开环
	App2.Ch1.AllowRun = 1;
	if(App2.Err > 0){
		App2.Ch1.Status= CTLS_ERROR; 
	}
	
}

//通道2初始化状态
void Ctl_Init2(void){
	
	//通道2有刷刀头电阻电感初始化
	if(App2.Log.Drill == 1){		//通道2刀头1
		//通道2刀头1电阻电感和转换系数、、保护电流
		App2.Ch2.Emf.qRs = (float)YSPara2[15] / 1000.0f;
		App2.Ch2.Emf.qLsDtq = (float)YSPara2[16] * PWM_FREQUENCY / 1000000.0f;
		App2.Ch2.Emf.qSpdxisu = (float)YSPara2[17];
		App2.Ch2.Emf.qRsxisu = (float)YSPara2[18] / 100.0f;
		if(App2.Log.CtlMode == 3){				//往复运动的过流保护值
			App2.Ch2.Prot.OverCurrent = App2.Ch2.Prot.OverCurrent2;
			App2.Ch2.Prot.CurCnt			 = App2.Ch2.Prot.CurCnt2;
		}else{													//单向运动的过流保护值
			App2.Ch2.Prot.OverCurrent = App2.Ch2.Prot.OverCurrent1;
			App2.Ch2.Prot.CurCnt			 = App2.Ch2.Prot.CurCnt1;
		}
	}
	else if(App2.Log.Drill == 2){	//通道2刀头2
		//通道2刀头2电阻电感和转换系数、保护电流
		App2.Ch2.Emf.qRs = (float)YSPara2[27] / 1000.0f;
		App2.Ch2.Emf.qLsDtq = (float)YSPara2[28] * PWM_FREQUENCY / 1000000.0f;
		App2.Ch2.Emf.qSpdxisu = (float)YSPara2[29];
		App2.Ch2.Emf.qRsxisu = (float)YSPara2[30] / 100.0f;
		if(App2.Log.CtlMode == 3){				//往复运动的过流保护值
			App2.Ch2.Prot.OverCurrent = App2.Ch2.Prot.OverCurrent4;
			App2.Ch2.Prot.CurCnt			 = App2.Ch2.Prot.CurCnt4;
		}else{													//单向运动的过流保护值
			App2.Ch2.Prot.OverCurrent = App2.Ch2.Prot.OverCurrent3;
			App2.Ch2.Prot.CurCnt			 = App2.Ch2.Prot.CurCnt3;
		}		
	}
	//Log/PWM参数初始化
	App2.SysCtl.MaxSpdSet = (float)( PWM_PERIOD_COUNT * YSPara2[12] /1000.0f);	//最大PWM
	App2.SysCtl.MinSpdSet = 0;
	App2.Ch2.Dir.Now = App2.Ch2.Dir.Set;		//确认启动方向
	App2.Log.VbusRef = 0;						//速度环计算后的输出值，给电压环的参考值
	App2.Log.BreakSta = 0;					//初始化刹车为正常运行状态
	App2.Ch2.Tim.StopTimCnt = 0;
	//PID参数初始化
	YSPI_Init();
	//有刷电机TRM初始化
	MCPwm_Init2();
	//Buck调压初始化
	App2.Log.VbusNowPWM = 0;				//PWM爬坡清零
	App2.Log.VbusSetPWM = 0;        //PWM爬坡清0
	Vbus_Tim_ON();						      //Buck电路开
	//允许启动
	App2.Ch2.AllowRun = 1;
	App2.Ch2.Status = CTLS_OPENLOOP_RUN;

	if(App2.Err > 0){
		App2.Ch2.Status = CTLS_ERROR; 
	}
	
}



//顺逆风检测
void Ctl_Check(void){
	
}

//顺逆风检测
void Ctl_Check2(void){
	
}
//刹车状态
void Ctl_Brake(void){
	
}
//刹车状态
void Ctl_Brake2(void){
	
}
//开环函数
void Ctl_OpenLoop(void){
	//直接进入闭环
	App2.Ch1.Status = CTLS_CLOSELOOP_RUN;
}
//开环函数
void Ctl_OpenLoop2(void){
	//直接进入闭环
	App2.Ch2.Status = CTLS_CLOSELOOP_RUN;
}


//通道1闭环状态
void Ctl_CloseLoop(void){
	TMR_CCxCmd(TMR1, TMR_Channel_3, TMR_CCx_Enable);  
	TMR_CCxNCmd(TMR1, TMR_Channel_3, TMR_CCxN_Enable); 
	TMR_Cmd(TMR1, ENABLE);
	//判断是否需要刹车
	if(	App2.Ch1.Start == 0 || (App2.Ch1.Dir.Now != App2.Ch1.Dir.Set)|| App2.Log.BreakSta >= 1 ){			//不允许启动，方向改变，
		//进入刹车状态
		if(App2.Log.BreakSta == 0){
			App2.Log.BreakSta = 1;			//进入刹车状态
		}
		//停止时间计时
		if(App2.Ch1.Tim.StopTimFg == 1){	
			App2.Ch1.Tim.StopTimFg = 0;		//刹车停止时间计数
			App2.Ch1.Tim.StopTimCnt ++;
		}
		//根据类型判断刹车
		if((	App2.Ch1.Start == 0 && App2.Ch1.Tim.StopTimCnt >= YSPara1[59])													//停止指令 或者报错停止的 刹车时间
			|| (App2.Ch1.Start == 1 && App2.Ch1.Tim.StopTimCnt >= YSPara1[60] && App2.Log.CtlMode !=3 )	//正常单向切换转向的刹车时间-->单向切换转相
			|| (App2.Ch1.Start == 1 && App2.Ch1.Tim.StopTimCnt >= YSPara1[61] && App2.Log.CtlMode == 3)	//往复模式下的刹车时间-->往复运动
			){
			App2.Ch1.Tim.StopTimCnt = 0;
			App2.Log.RunTimCnt1 = 0;
			App2.Ch1.AllowRun = 0;
			App2.Log.BreakSta = 0;				//刹车完成，进入正常状态
			TIME1_PWM_Stop_3Channel();
			Vbus_Tim_OFF();								//关Buck
			App2.Ch1.Status = CTLS_STOP;
		}
		
	}else if(App2.Ch1.Start == 1){
		App2.Ch1.Tim.StopTimCnt = 0;
	}
	
	if(App2.Err > 0){
		App2.Ch1.Status = CTLS_ERROR; 
	}
}

//通道2闭环状态
void Ctl_CloseLoop2(void){
	TMR_CCxCmd(TMR1, TMR_Channel_3, TMR_CCx_Enable);  
	TMR_CCxNCmd(TMR1, TMR_Channel_3, TMR_CCxN_Enable); 
	TMR_Cmd(TMR1, ENABLE);
	//判断是否需要刹车
	if(App2.Ch2.Start == 0 || (App2.Ch2.Dir.Now != App2.Ch2.Dir.Set)|| App2.Log.BreakSta >= 1){							//执行完一次刹车
		//进入刹车状态
		if(App2.Log.BreakSta == 0){
			App2.Log.BreakSta = 1;//进入刹车状态
		}
		//停止时间计时
		if(App2.Ch2.Tim.StopTimFg == 1){
			App2.Ch2.Tim.StopTimFg = 0;
			App2.Ch2.Tim.StopTimCnt ++;	
		}
		//根据类型判断刹车
		if((	App2.Ch2.Start == 0 && App2.Ch2.Tim.StopTimCnt >= YSPara2[59])		//停止指令 或者报错停止的 刹车时间
			|| (App2.Ch2.Start == 1 && App2.Ch2.Tim.StopTimCnt >= YSPara2[60] && App2.Log.CtlMode !=3 )	//正常单向切换转向的刹车时间-->单向切换转相
			|| (App2.Ch2.Start == 1 && App2.Ch2.Tim.StopTimCnt >= YSPara2[61] && App2.Log.CtlMode == 3)	//往复模式下的刹车时间-->往复运动
			){
			App2.Ch2.Tim.StopTimCnt = 0;
			App2.Log.RunTimCnt2 = 0;
			App2.Ch2.AllowRun = 0;
			App2.Log.BreakSta = 0;
			//刹车完成，进入正常状态
			TIME8_PWM_Stop_3Channel();
			Vbus_Tim_OFF();
			App2.Ch2.Status = CTLS_STOP;
		}
	}else if(App2.Ch2.Start == 1){
		App2.Ch2.Tim.StopTimCnt = 0;
	}
	if(App2.Err > 0){
		App2.Ch2.Status = CTLS_ERROR; 
	}
}


//通道1停止状态
void Ctl_Stop(void){
	App2.Ch1.AllowRun = 0;					//不允许运行
	TIME1_PWM_Stop_3Channel();			//关闭MOS
	App2.Ch1.Status = CTLS_WAIT;		//重新进入空闲状态
	if(App2.Err > 0){								//有错误则进入错误状态,
		App2.Ch1.Status = CTLS_ERROR; 
	}
}

//通道2停止状态
void Ctl_Stop2(void){
	App2.Ch2.AllowRun = 0;						//不允许运行
	TIME8_PWM_Stop_3Channel();        //关闭MOS
	App2.Ch2.Status = CTLS_WAIT;      //重新进入空闲状态
	if(App2.Err > 0){                 //有错误则进入错误状态,
		App2.Ch2.Status = CTLS_ERROR; 
	}
}

//通道1错误状态
void Ctl_Error(void){
	App2.Ch1.AllowRun = 0;						//不允许运行
	TIME1_PWM_Stop_3Channel();				//错误状态关闭MOS
	if(App2.Err == E_NONE){	
		App2.Ch1.Status = CTLS_WAIT;  	//无错误进入空闲状态
		App2.Log.BreakSta	= 0;	
	}
}

//通道2错误状态
void Ctl_Error2(void){
	App2.Ch2.AllowRun = 0;						//不允许运行
	TIME8_PWM_Stop_3Channel();        //错误状态关闭MOS
	if(App2.Err == E_NONE){           
		App2.Ch2.Status = CTLS_WAIT;    //无错误进入空闲状态
		App2.Log.BreakSta	= 0;		
	}
}

//有刷状态机
void YSMotor_Ctl1(void){
	switch(App2.Ch1.Status){//按照程序运行过程中执行的频率排序
		case CTLS_CLOSELOOP_RUN:		
				Ctl_CloseLoop();		//闭环
					break;
		case CTLS_WAIT:
				Ctl_Wait();					//等待
					break;
		case CTLS_OPENLOOP_RUN:
				Ctl_OpenLoop();    	//开环         
					break;
		case CTLS_ERROR:
				Ctl_Error();				//错误
					break;
		case CTLS_INIT:
				Ctl_Init();					//初始化
					break;
		case CTLS_STOP:              
				Ctl_Stop();					//停止
					break;
		case CTLS_CHECK:
				Ctl_Check();				//检测
					break;
		case CTLS_BRAKE:
				Ctl_Brake();				//刹车
					break;
		default:
			  Ctl_Wait(); 				//等待            
					break;
	}
}




void YSMotor_Ctl2(void){
		switch(App2.Ch2.Status){//按照程序运行过程中执行的频率排序
			case CTLS_CLOSELOOP_RUN:
					Ctl_CloseLoop2();				//闭环
						break;                
			case CTLS_WAIT:             
					Ctl_Wait2();            //等待
						break;                
			case CTLS_OPENLOOP_RUN:     
					Ctl_OpenLoop2();        //开环       
						break;                
			case CTLS_ERROR:            
					Ctl_Error2();           //错误
						break;                
			case CTLS_INIT:             
					Ctl_Init2();            //初始化
						break;                
			case CTLS_STOP:                   
					Ctl_Stop2();            //停止
						break;                
			case CTLS_CHECK:            
					Ctl_Check2();           //检测
						break;                
			case CTLS_BRAKE:            
					Ctl_Brake2();           //刹车
						break;                
			default:                    
					Ctl_Wait2();            //等待   
						break;
		}
}




//电机1刹车 关闭PWM输出
void TIME1_PWM_Stop_3Channel(void){
	TMR_SetCompare1(TMR1,0);
	TMR_SetCompare2(TMR1,0);

}
//电机2刹车 关闭PWM输出
void TIME8_PWM_Stop_3Channel(void){
	TMR_SetCompare1(TMR8,0);
	TMR_SetCompare2(TMR8,0);

}

//BUCK电路开启
void Vbus_Tim_ON(void){
	TMR_Cmd(TMR2, ENABLE);
	TMR_SetCompare4(TMR2,0);
	PVCC_PWM_ON;
}

//BUCK电路关闭
void Vbus_Tim_OFF(void){
	TMR_SetCompare4(TMR2,0);
}



