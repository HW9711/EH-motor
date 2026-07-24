#include "YSprotect.h"
#include "youshua.h"
#include "YSmcctl.h"

//有刷保护参数初始化
void YSProtect_Init(void){

	//系统保护参数值的初始化
	App2.SysPort.UnderVoltage = (float)YSPara1[5] / 10.0f;			//欠压保护值
	App2.SysPort.UnderVolCnt	 = 0;															//欠压计数值
	
	App2.SysPort.OverVoltage	 = (float)YSPara1[7] / 10.0f;			//过压保护值
	App2.SysPort.OverVolCnt	 = 0;															//过压保护值
		 
	App2.SysPort.OverTemperature = (float)YSPara1[9];					//过温保护值
	App2.SysPort.OverVolCnt			= 0;													//过温计数值
		
	App2.Ch1.Prot.OverCurrent1 = (float)YSPara1[19]/100.0f;			//通道1刀头1过流保护值（单向运动）
	App2.Ch1.Prot.CurCnt1			= YSPara1[20];
	App2.Ch1.Prot.OverCurrent2	= (float)YSPara1[21]/100.0f;			//通道1刀头1过流保护值（往复运动）
	App2.Ch1.Prot.CurCnt2			= YSPara1[22];
	App2.Ch1.Prot.OverCurrent3 = (float)YSPara1[31]/100.0f;			//通道1刀头2过流保护值（单向运动）
	App2.Ch1.Prot.CurCnt3			= YSPara1[32];
	App2.Ch1.Prot.OverCurrent4	= (float)YSPara1[33]/100.0f;			//通道1刀头2过流保护值（往复运动）
	App2.Ch1.Prot.CurCnt4			= YSPara1[34];
	App2.Ch1.Prot.OverCurrent  	= App2.Ch1.Prot.OverCurrent1;		//初始化最终赋值
	App2.Ch2.Prot.OverCurCnt		= App2.Ch1.Prot.CurCnt1;			//初始化最终赋值	
		
	App2.Ch2.Prot.OverCurrent1 = (float)YSPara2[19]/100.0f;			//通道2刀头1过流保护值（单向运动）
	App2.Ch2.Prot.CurCnt1			= YSPara2[20];
	App2.Ch2.Prot.OverCurrent2	= (float)YSPara2[21]/100.0f;			//通道2刀头1过流保护值（往复运动）
	App2.Ch2.Prot.CurCnt2			= YSPara2[22];
	App2.Ch2.Prot.OverCurrent3 = (float)YSPara2[31]/100.0f;			//通道2刀头2过流保护值（单向运动）
	App2.Ch2.Prot.CurCnt3			= YSPara2[32];
	App2.Ch2.Prot.OverCurrent4	= (float)YSPara2[33]/100.0f;			//通道2刀头2过流保护值（往复运动）
	App2.Ch2.Prot.CurCnt4			= YSPara2[34];
	App2.Ch2.Prot.OverCurrent   = App2.Ch2.Prot.OverCurrent1;		//初始化最终赋值
	App2.Ch2.Prot.OverCurCnt		= App2.Ch2.Prot.CurCnt1;			//初始化最终赋值

}


//有刷保护函数
void YSMotor_Protect(void){
if(App2.SysCtl.CheckCyc == 1 && App2.Err == E_NONE){
		App2.SysCtl.CheckCyc = 0;

		if(App.Logic.HandShakeProtCnt > App.Logic.HandShakeProtTim){
			App2.Err = E_HandShake;					//通讯错误
			
		}else{
			App.Logic.HandShakeProtCnt++;
		}
	
		if(App2.Log.Channel == 3){
			//堵转保护1,电机1			--正转和反转有效--往复正反转无效
			if((App2.Ch1.AllowRun == 1)&&(App2.Log.CtlMode !=3)){
				if(App2.Log.RunTimCnt1 <= YSPara1[55] * 10 || App2.Log.BreakSta == 1){
					App2.Ch1.Prot.RunStallCnt1 = 0;
				}else {
					if(App2.SysCtl.SpdNowLPF < YSPara1[35]){
						App2.Ch1.Prot.RunStallCnt1 ++;
						if(App2.Ch1.Prot.RunStallCnt1 > YSPara1[36]){
							App2.Err = E_RUNSTALL;
						}
					}else if(App2.Ch1.Prot.RunStallCnt1 > 0){
						App2.Ch1.Prot.RunStallCnt1 -- ;
					}
				}
			}else{
				App2.Ch1.Prot.RunStallCnt1 = 0;
			}			
			
			//堵转保护2，电机1				--快速正反转有效，正转反转无效
			if((App2.Log.CtlMode ==3)&&(App2.SysCtl.SpdSet > YSPara1[2]*10)){
				if(App2.Log.RunTimCnt1 <= YSPara1[55] * 3 || App2.Log.BreakSta == 1){
					App2.Ch1.Prot.RunStallCnt3 = 0;
				}else{		
					if(App2.SysCtl.SpdNowLPF < YSPara1[37]){
						App2.Ch1.Prot.RunStallCnt3 ++;
						if(App2.Ch1.Prot.RunStallCnt3 >YSPara1[38]){
							App2.Err = E_RUNSTALL;
							}
						}else if(App2.Ch1.Prot.RunStallCnt3 > 0){
							App2.Ch1.Prot.RunStallCnt3 -= YSPara1[53];
							if(App2.Ch1.Prot.RunStallCnt3 < 0){
								App2.Ch1.Prot.RunStallCnt3 = 0;
							}
					}
				}
			}else{
				App2.Ch1.Prot.RunStallCnt3 = 0;	
			}		
			
			//电机1软件过流检测
			if((App2.Log.BreakSta == 0)&&(App2.Ch1.AllowRun == 1)){
				if(App2.Ch1.ADC.CurLPF > App2.Ch1.Prot.OverCurrent){
					App2.Ch1.Prot.OverCurCnt ++;
						if(App2.Ch1.Prot.OverCurCnt > App2.Ch1.Prot.CurCnt){
							App2.Ch1.Prot.OverCurCnt = 0;
							App2.Err = E_OC1;          
						}
				}else if(App2.Ch1.Prot.OverCurCnt > 0){
					App2.Ch1.Prot.OverCurCnt --;
				}
			}			
		}
		else if (App2.Log.Channel == 4){
			
			//电机2			--正转和反转有效--往复正反转无效
			if((App2.Ch2.AllowRun == 1)&&(App2.Log.CtlMode !=3)){
				if(App2.Log.RunTimCnt2 <= YSPara2[55] * 10 || App2.Log.BreakSta == 1){
					App2.Ch2.Prot.RunStallCnt2 = 0;
				}else {
					if(App2.SysCtl.SpdNowLPF < YSPara2[23]){
						App2.Ch2.Prot.RunStallCnt2 ++;
						if(App2.Ch2.Prot.RunStallCnt2 > YSPara2[24]){
							App2.Err = E_RUNSTALL;
						}
					}else if(App2.Ch2.Prot.RunStallCnt2 > 0){
						App2.Ch2.Prot.RunStallCnt2 -- ;
					}
				}
			}else{
				App2.Ch2.Prot.RunStallCnt2 = 0;
			}	
			
			//堵转保护2，电机2				--快速正反转有效，正转反转无效
			if((App2.Log.CtlMode ==3)&&(App2.SysCtl.SpdSet > YSPara1[2]*10)){
				if(App2.Log.RunTimCnt2 <= YSPara2[55] * 3 || App2.Log.BreakSta == 1){
					App2.Ch2.Prot.RunStallCnt4 = 0;
				}else {			
					if(App2.SysCtl.SpdNowLPF < YSPara2[25]){
						App2.Ch2.Prot.RunStallCnt4 ++;
						if(App2.Ch2.Prot.RunStallCnt4 >YSPara2[26]){
							App2.Err = E_RUNSTALL;
							}
						}else if(App2.Ch2.Prot.RunStallCnt4 > 0){
							App2.Ch2.Prot.RunStallCnt4 -= YSPara2[53] ;
							if(App2.Ch2.Prot.RunStallCnt4 <0){
								App2.Ch2.Prot.RunStallCnt4 = 0;
							}
					}
				}
			}else{
				App2.Ch2.Prot.RunStallCnt4 = 0;	
			}

		//电机2软件过流检测
		if((App2.Log.BreakSta == 0)&&(App2.Ch2.AllowRun == 1)){
			if(App2.Ch2.ADC.CurLPF > App2.Ch2.Prot.OverCurrent){
				App2.Ch2.Prot.OverCurCnt ++;

					if(App2.Ch2.Prot.OverCurCnt > App2.Ch2.Prot.CurCnt){
						App2.Ch2.Prot.OverCurCnt = 0;
						App2.Err = E_OC1;          
					}
			}else if(App2.Ch2.Prot.OverCurCnt > 0){
				App2.Ch2.Prot.OverCurCnt --;
				}
			}							
		}
	}
}





