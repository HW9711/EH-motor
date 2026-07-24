#include "youshua.h"
#include "mcctl.h"
#include "userpara.h"
#include "adc.h"
#include "YSmcctl.h"




APP2_TYPEDEF App2;		//有刷电机参数App2

//有刷ADC中断处理
void Youshua_ADCISRTasks(void){
	volatile short phaseVolP = 0;					//采取电压
	float temp = 0;
	float temp1 = 0;

	//电压-vbus		//注入通道采集完触发中断
	temp = (float)get_Pvcc_adc_value*VOLTAGEP_ADC_PVCC_PHY_RATIO;
	//App.Ch1.ADC.Vbus * 0.9f + temp * 0.1f;													//滤波
	App2.Ch1.ADC.Vbus = temp;																					//不滤波
	App2.Ch2.ADC.Vbus = App2.Ch1.ADC.Vbus;														//公用BUCK
	
	temp1 = (float)get_shunt_t2_adc_value;
	App2.Ch1.Prot.LosePhaseVal = temp1;
	App2.Ch2.Prot.LosePhaseVal = App2.Ch1.Prot.LosePhaseVal;					//只有一个电流采样，共用一个电流采样
	temp = (float)temp1 * ADC_CURRENT_SCALE;
	if(temp < 0){				
		temp = 0;					
	}
	App2.Ch1.ADC.CurNow = temp;
	App2.Ch1.ADC.CurLPF = App2.Ch1.ADC.CurLPF * 0.9f + App2.Ch1.ADC.CurNow* 0.1f;			//滤波

	App2.Ch2.ADC.CurNow = App2.Ch1.ADC.CurNow;
	App2.Ch2.ADC.CurLPF = App2.Ch1.ADC.CurLPF;
	
	
	
	
	if((App2.Ch1.AllowRun == 1 && App2.Log.BreakSta == 0 )||(App2.Ch2.AllowRun == 1 && App2.Log.BreakSta == 0)){//通道1/2正常运行状态
		//根据电机模型读取转速
		if(App2.Log.Channel == 3){
			App2.Ch1.Emf.qIerr = App2.Ch1.ADC.CurNow - App2.Ch1.Emf.qLastI;
			App2.Ch1.Emf.qEmf = 
											App2.Ch1.ADC.Vbus //电机两端电压					
											- (App2.Ch1.ADC.CurNow * (App2.Ch1.Emf.qRs + App2.Ch1.ADC.CurNow * App2.Ch1.Emf.qRsxisu)) //线圈电阻	 
																//这里把电阻的阻值加上个（ App.ADC.CurNow * App.Emf.qRsxisu）
																//应该是考虑到了电阻在不同条件下电阻的非线性特性
																//应该是对电流较大的时候电阻值的补偿！实现更准的估算
											- (App2.Ch1.Emf.qLsDtq * App2.Ch1.Emf.qIerr);//线圈电感
											//- (App.ADC.CurNow * App.ADC.CurNow * 20.0f)//碳刷压差
			App2.Ch1.Emf.qEmfLPF = App2.Ch1.Emf.qEmfLPF * 0.9f + App2.Ch1.Emf.qEmf * 0.1f;
			App2.Ch1.Emf.qLastI = App2.Ch1.ADC.CurNow;
		}else if(App2.Log.Channel == 4){
			App2.Ch2.Emf.qIerr = App2.Ch2.ADC.CurNow - App2.Ch2.Emf.qLastI;
			App2.Ch2.Emf.qEmf = 
											App2.Ch2.ADC.Vbus //电机两端电压					
											- (App2.Ch2.ADC.CurNow * (App2.Ch2.Emf.qRs + App2.Ch2.ADC.CurNow * App2.Ch2.Emf.qRsxisu)) //线圈电阻	 
																//这里把电阻的阻值加上个（ App.ADC.CurNow * App.Emf.qRsxisu）
																//考虑到了电阻在不同条件下电阻的非线性特性
																//对电流较大的时候电阻值的补偿！更准的估算
											- (App2.Ch2.Emf.qLsDtq * App2.Ch2.Emf.qIerr);//线圈电感
											//- (App.ADC.CurNow * App.ADC.CurNow * 20.0f)//碳刷压差
			App2.Ch2.Emf.qEmfLPF = App2.Ch2.Emf.qEmfLPF * 0.9f + App2.Ch2.Emf.qEmf * 0.1f;
			App2.Ch2.Emf.qLastI = App2.Ch2.ADC.CurNow;
		}
		
		if((App2.Log.RunTimCnt1 >= YSPara1[55])||(App2.Log.RunTimCnt2 >= YSPara2[55])){
			//通道3、速度计算、速度爬坡、速度环、电压环计算、电压爬坡
			if(App2.Log.Channel == 3){
				App2.SysCtl.SpdNowLPF = App2.Ch1.Emf.qEmfLPF * App2.Ch1.Emf.qSpdxisu;
				//速度闭环
				if( (App2.SysCtl.SpdSetSet - App2.SysCtl.SpdSet) < -YSPara1[46]){
					App2.SysCtl.SpdSetSet += YSPara1[46];
				}else if( (App2.SysCtl.SpdSetSet - App2.SysCtl.SpdSet) > YSPara1[47]){
					App2.SysCtl.SpdSetSet -= YSPara1[47];
				}else {
					App2.SysCtl.SpdSetSet= App2.SysCtl.SpdSet;
				}
				YS_SpdPI_Out();		//速度环：输出PI给电压环
				YS_VBus_PI_Out();		//电压环：输出PI给PWM
				App2.Log.VbusSetPWM = MC_VBus.qOut;	//PWM的期望值，要经过PWM爬坡
				//PWM爬坡
				if((App2.Log.VbusNowPWM - App2.Log.VbusSetPWM) < -YSPara1[51]){
					App2.Log.VbusNowPWM += YSPara1[51];
				}else if( (App2.Log.VbusNowPWM - App2.Log.VbusSetPWM) > YSPara1[52]){
					App2.Log.VbusNowPWM -= YSPara1[52];
				}else {
					App2.Log.VbusNowPWM = App2.Log.VbusSetPWM;
				}
				TMR_SetCompare4(TMR2,App2.Log.VbusNowPWM);	//PWM赋值
			}
			//通道4、速度计算、速度爬坡、速度环、电压环计算、电压爬坡
			else if(App2.Log.Channel == 4){
				App2.SysCtl.SpdNowLPF = App2.Ch2.Emf.qEmfLPF * App2.Ch2.Emf.qSpdxisu;
				//速度闭环
				if( (App2.SysCtl.SpdSetSet - App2.SysCtl.SpdSet) < -YSPara2[46]){
					App2.SysCtl.SpdSetSet += YSPara2[46];
				}else if( (App2.SysCtl.SpdSetSet - App2.SysCtl.SpdSet) > YSPara2[47]){
					App2.SysCtl.SpdSetSet -= YSPara2[47];
				}else {
					App2.SysCtl.SpdSetSet = App2.SysCtl.SpdSet;
				}
				YS_SpdPI_Out();		//速度环：输出PI给电压环
				YS_VBus_PI_Out();		//电压环：输出PI给PWM
				
				App2.Log.VbusSetPWM = MC_VBus.qOut;	//PWM的期望值，要经过PWM爬坡
				
				if((App2.Log.VbusNowPWM - App2.Log.VbusSetPWM) < -YSPara2[51]){
					App2.Log.VbusNowPWM += YSPara2[51];
				}else if( (App2.Log.VbusNowPWM - App2.Log.VbusSetPWM) > YSPara2[52]){
					App2.Log.VbusNowPWM -= YSPara2[52];
				}else {
					App2.Log.VbusNowPWM = App2.Log.VbusSetPWM;
				}
					TMR_SetCompare4(TMR2,App2.Log.VbusNowPWM);	//最后赋值
			}
		}else {
			App2.SysCtl.SpdNow = 0;
			App2.SysCtl.SpdNowLPF = 0;
			
			App2.Log.VbusNowPWM = 0;
			App2.Log.VbusSetPWM = 0;
			TMR_SetCompare4(TMR2,App2.Log.VbusNowPWM);//还没启动!不允许有值	
		}

	}else {
		App2.SysCtl.SpdNow = 0;
		App2.SysCtl.SpdNowLPF = 0;		
		App2.SysCtl.SpdSetSet = 0;
		
		if(App2.Log.Channel == 3){//旧值传递
			App2.Ch1.Emf.qLastI = App2.Ch1.ADC.CurNow;
		}else if(App2.Log.Channel == 4){//旧值传递
			App2.Ch2.Emf.qLastI = App2.Ch2.ADC.CurNow;
		}
	}
	
	//缺相保护判定
	App2.Ch1.Prot.LosePhaseAllTempVal += App2.Ch1.Prot.LosePhaseVal;		//电流积分		
	App2.Ch2.Prot.LosePhaseAllTempVal += App2.Ch2.Prot.LosePhaseVal;		//电流积分
	App2.Ch1.Prot.LosePhaseTim ++;					//观测10次电流的积分来判断
	if(App2.Ch1.Prot.LosePhaseTim >= 10){
		App2.Ch1.Prot.LosePhaseTim = 0;
		//滤波
		App2.Ch1.Prot.LosePhaseAllVal = App2.Ch1.Prot.LosePhaseAllVal * 0.9f + App2.Ch1.Prot.LosePhaseAllTempVal * 0.1f;
		App2.Ch2.Prot.LosePhaseAllVal = App2.Ch2.Prot.LosePhaseAllVal * 0.9f + App2.Ch2.Prot.LosePhaseAllTempVal * 0.1f;
		//清值
		App2.Ch1.Prot.LosePhaseAllTempVal = 0;
		App2.Ch2.Prot.LosePhaseAllTempVal = 0;
		//缺相保护电机1 
		if(App2.Ch1.AllowRun == 1 && App2.Log.BreakSta == 0){//不在刹车时检测
			if(
				App2.Ch1.Prot.LosePhaseAllVal < YSPara1[39] 
				&& App2.SysCtl.SpdNowLPF >= YSPara1[41]
				&& App2.SysCtl.SpdSet >= YSPara1[40]
			){
				App2.Ch1.Prot.LosePhaseCnt += 1;
				if(App2.Ch1.Prot.LosePhaseCnt > YSPara1[42]){
					App2.Ch1.Prot.LosePhaseCnt = 0;
					App2.Err = E_PhaseLoss;
				}
			}else if(App2.Ch1.Prot.LosePhaseCnt > 0){
				App2.Ch1.Prot.LosePhaseCnt --;
			}
		}

	//缺相保护电机2 
	if((App2.Ch2.AllowRun == 1 && App2.Log.BreakSta == 0)){
			if(
				App2.Ch2.Prot.LosePhaseAllVal < YSPara2[39] 
				&& App2.SysCtl.SpdNowLPF >= YSPara2[41]
				&& App2.SysCtl.SpdSet	>= YSPara2[40]
			){
				App2.Ch2.Prot.LosePhaseCnt += 1;
				if(App2.Ch2.Prot.LosePhaseCnt > YSPara2[42]){
					App2.Ch2.Prot.LosePhaseCnt = 0;
					App2.Err = E_PhaseLoss;
				}
			}else if(App2.Ch2.Prot.LosePhaseCnt > 0){
				App2.Ch2.Prot.LosePhaseCnt --;
			}		
		}
	}
	
}

//初始化
void YoushuaInit(void){
	
	
	
}

//有刷TMR的初始化通道1
void MCPwm_Init1(void){
	//CH3全关，有刷用不到，防止有影响
	TMR_CCxNCmd(TMR1, TMR_Channel_3, TMR_CCxN_Disable); 
	TMR_CCxCmd(TMR1, TMR_Channel_3, TMR_CCx_Disable); 
	if(App2.Ch1.Dir.Now == CW){
		//上桥PWM
		TMR_CCxNCmd(TMR1, TMR_Channel_1, TMR_CCxN_Disable);  
		TMR_CCxCmd(TMR1, TMR_Channel_1, TMR_CCx_Enable);  
		TMR_SetCompare1(TMR1,PWM_PERIOD_COUNT);									//全开	
		//下桥全开
		TMR_CCxCmd(TMR1, TMR_Channel_2, TMR_CCx_Disable);
		TMR_CCxNCmd(TMR1, TMR_Channel_2, TMR_CCxN_Enable);  
		TMR_SetCompare2(TMR1,PWM_PERIOD_COUNT);									//全开	
	}
	//CH2-上桥全开 下桥全关 CH1-上桥关闭 下桥PWM
	else {
		TMR_CCxNCmd(TMR1, TMR_Channel_2, TMR_CCxN_Disable);
		TMR_CCxCmd(TMR1, TMR_Channel_2, TMR_CCx_Enable);  		
		TMR_SetCompare2(TMR1,PWM_PERIOD_COUNT);									//全开	
		
		TMR_CCxCmd(TMR1, TMR_Channel_1, TMR_CCx_Disable);
		TMR_CCxNCmd(TMR1, TMR_Channel_1, TMR_CCxN_Enable);  
		TMR_SetCompare1(TMR1,PWM_PERIOD_COUNT);									//全开	
	}
	TMR_Cmd(TMR1, ENABLE);
}

//有刷TMR的初始化通道2
void MCPwm_Init2(void){
	//CH3全关，有刷用不到，防止有影响
	TMR_CCxNCmd(TMR8, TMR_Channel_3, TMR_CCxN_Disable); 
	TMR_CCxCmd(TMR8, TMR_Channel_3, TMR_CCx_Disable); 
	if(App2.Ch2.Dir.Now == CW){
		//上桥PWM
		TMR_CCxNCmd(TMR8, TMR_Channel_1, TMR_CCxN_Disable);  
		TMR_CCxCmd(TMR8, TMR_Channel_1, TMR_CCx_Enable);  
		TMR_SetCompare1(TMR8,PWM_PERIOD_COUNT);									//全开	
		//下桥全开
		TMR_CCxCmd(TMR8, TMR_Channel_2, TMR_CCx_Disable);
		TMR_CCxNCmd(TMR8, TMR_Channel_2, TMR_CCxN_Enable);  
		TMR_SetCompare2(TMR8,PWM_PERIOD_COUNT);									//全开
	}
	//CH2-上桥全开 下桥全关 CH1-上桥关闭 下桥PWM
	else {
		TMR_CCxNCmd(TMR8, TMR_Channel_2, TMR_CCxN_Disable);
		TMR_CCxCmd(TMR8, TMR_Channel_2, TMR_CCx_Enable);  		
		TMR_SetCompare2(TMR8,PWM_PERIOD_COUNT);									//全开
		
		TMR_CCxCmd(TMR8, TMR_Channel_1, TMR_CCx_Disable);
		TMR_CCxNCmd(TMR8, TMR_Channel_1, TMR_CCxN_Enable);  
		TMR_SetCompare1(TMR8,PWM_PERIOD_COUNT);									//全开
	}
	TMR_Cmd(TMR8, ENABLE);
}





















