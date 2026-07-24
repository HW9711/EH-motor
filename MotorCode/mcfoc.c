#include <math.h>
#include "mcfoc.h"
#include "mcLib.h"
#include "mcCtl.h"
#include "tim.h"
#include "userpara.h"
#include "hallfoc.h"
#include "adc.h"
#include "enocfoc.h"


#if Debug_vis == 1
int32_t vis_pos_set_spd = 0;
int32_t vis_pos_set = 0;
int32_t vis_pos_now = 0;

int32_t vis_spd_set_iq = 0;
int32_t vis_spd_set = 0;
int32_t vis_spd_now = 0;

int32_t vis_cur_a = 0;
int32_t vis_cur_b = 0;
int32_t vis_cur_c = 0;

#endif
/****************************************************************************
*函数名：ADC采样服务函数
*传入参数：void
*传出参数：void
*****************************************************************************/
void mcApp_ADCISRTasks(void){

//	float VoltageP = 0;
	volatile short phaseCurrentA = 0;
	volatile short phaseCurrentB = 0;
	volatile short phaseVol = 0;
	volatile short phaseVolP = 0;
	volatile short phaseCurrentT = 0;
	volatile int32_t temp_pwm = 0;
	float PWM_UP = MCPara[135];
	float PWM_Down = MCPara[136];
	
	//获取 电流 ADC值
//	phaseCurrentA = get_shunt_a_adc_value;		//A相电流
//	phaseCurrentB = get_shunt_b_adc_value;		//B相电流
//	
//	phaseCurrentT = get_shunt_t_adc_value;		//总电流
	
	//获取 电压 ADC值
	phaseVol = 	get_vbus_adc_value;			//总电压
	
	phaseVolP = get_Pvcc_adc_value;			//BUCK降压后电压
	
	//获取电压
	mcApp_focParam.DCBusVoltage_T = (float)phaseVol * VOLTAGE_ADC_TO_PHY_RATIO;
	
	mcApp_focParam.DCBusVoltage = (float)phaseVolP * VOLTAGEP_ADC_PVCC_PHY_RATIO;	//不跳开BUCK电路
	
//	mcApp_focParam.DCBusVoltage = mcApp_focParam.DCBusVoltage_T;		//跳开BUCK电路
	
/**********************************************************************************/	
/////**VPP调压**/

	if((App.M1.AllowRun ==1)||(App.M2.AllowRun ==1)){
//		if(App.M1.Tim.CycPWMSetFg ==1){
//			App.M1.Tim.CycPWMSetFg = 0;
			//Buck测量值给定
			mcApp_VoltageP_PIParam.qInMeas = mcApp_focParam.DCBusVoltage;
			//Buck期望值给定
			mcApp_VoltageP_PIParam.qInRef  = (float)MCPara[137]/10.0f;
			//Buck的PI计算
			mcLib_CalcPI(&mcApp_VoltageP_PIParam);
			//输出值给temp；
			temp_pwm = (uint16_t)mcApp_VoltageP_PIParam.qOut;
			//给定期望值，但是需要爬坡
			App.M1.Log.VBusSetPWM = temp_pwm;
			if((App.M1.Log.VBusNowPWM - App.M1.Log.VBusSetPWM) < -PWM_UP){
					App.M1.Log.VBusNowPWM += PWM_UP;
				}else if( (App.M1.Log.VBusNowPWM - App.M1.Log.VBusSetPWM) >PWM_Down){
					App.M1.Log.VBusNowPWM -=PWM_Down;
				}else {
					App.M1.Log.VBusNowPWM = App.M1.Log.VBusSetPWM;
				}

				TMR_SetCompare4(TMR2, App.M1.Log.VBusNowPWM);
//		}else
//		{
//	//		App.M1.Log.VBusNowPWM = 0;
//	//		App.M1.Log.VBusSetPWM = 0;
//		}
				
	}else{
		App.M1.Log.VBusNowPWM = 0;
		App.M1.Log.VBusSetPWM = 0;
		mcApp_VoltageP_PIParam.qdSum=0;
		mcApp_VoltageP_PIParam.qOut=0;
		
	}
/**********************************************************************************/	
	mcApp_focParam.MaxPhaseVoltage = mcApp_focParam.DCBusVoltage * ONE_BY_SQRT3;
	//计算相电流
	
	//正确的
	phaseCurrentA = mcApp_I_ABCParam.MidAvgA - phaseCurrentA; 
	phaseCurrentB = mcApp_I_ABCParam.MidAvgB - phaseCurrentB;
	
	mcApp_I_ABCParam.a = (float)phaseCurrentA * ADC_CURRENT_SCALE;    
	mcApp_I_ABCParam.b = (float)phaseCurrentB * ADC_CURRENT_SCALE;
	mcApp_I_ABCParam.c = -(mcApp_I_ABCParam.a + mcApp_I_ABCParam.b);
	
	mcApp_I_ABCParam.t = (float)phaseCurrentT - mcApp_I_ABCParam.MidAvgT;
	

	//Clarke
	mcLib_ClarkeTransform(&mcApp_I_ABCParam, &mcApp_I_AlphaBetaParam);
	//Park
	mcLib_ParkTransform(&mcApp_I_AlphaBetaParam,  &mcApp_SincosParam, 
											&mcApp_I_DQParam);

	if(App.M1.Hpll.HallorPll == 1){//PLL 估算器给定角度
		mcLib_PLLEstimator(&mcApp_EstimParam, &mcApp_SincosParam, &mcApp_focParam, 
										 &mcApp_ControlParam, &mcApp_I_AlphaBetaParam, 
										 &mcApp_V_AlphaBetaParam);
		//开闭环电流给定
		MC_APP_MC_DoControl();
		//开闭环角度给定
		MC_APP_MC_CalculateParkAngle();
		//角度给定
		
		if(App.M1.OpenLoop.OpenLoop == 1){
			mcApp_SincosParam.Angle = App.M1.OpenLoop.Ang;
		}else {
			mcApp_SincosParam.Angle = mcApp_EstimParam.qRho;
			
		}
		
	}else if(App.M1.Hpll.HallorPll == 2){//hallfoc 给定角度
		hall_cal();
		//计算速度
		//hall运行时
		pll_run(App.M1.Hall.Ang, App.M1.CalSpd.Dt, &App.M1.CalSpd.OldRad, &App.M1.CalSpd.NowRad);
		//闭环电流给定
		Hall_FOC_DoControl();
		//开闭环角度给定
		Hall_FOC_CalculateParkAngle();
		//角度给定
		if(App.M1.OpenLoop.OpenLoop == 1){//==1 在学习时
			mcApp_SincosParam.Angle = App.M1.OpenLoop.Ang;
		}else {
			//hall运行时
			//mcApp_SincosParam.Angle = App.M1.Hall.Ang;
			//编码器运行时
			mcApp_SincosParam.Angle = App.M1.Hall.Ang;
		}
		
	}else{	
	}
	//计算角度的SinCos
	mcLib_SinCosGen(&mcApp_SincosParam); 
	//逆Park变化
	mcLib_InvParkTransform(&mcApp_V_DQParam,&mcApp_SincosParam, &mcApp_V_AlphaBetaParam);  
	if(App.M1.AllowRun == 1){
		//Clack逆变换
		mcLib_SVPWMGen(&mcApp_V_AlphaBetaParam , &mcApp_SVGenParam); 
		
		TMR_SetCompare1(TMR1, (uint16_t) mcApp_SVGenParam.dPWM_A);
		TMR_SetCompare2(TMR1, (uint16_t) mcApp_SVGenParam.dPWM_B);
		TMR_SetCompare3(TMR1, (uint16_t) mcApp_SVGenParam.dPWM_C);
		
		if(App.M1.Prot.PhaseLossSta == 0){
			App.M1.Prot.PhaseLossSta = 1;
			
			App.M1.Prot.PhaseLossNum = 0;
			App.M1.Prot.PhaseLossACnt = 0;
			App.M1.Prot.PhaseLossBCnt = 0;
			App.M1.Prot.PhaseLossCCnt = 0;
			
		}else if(App.M1.Prot.PhaseLossSta == 1){
			App.M1.Prot.PhaseLossACnt += myabs(mcApp_I_ABCParam.a);
			App.M1.Prot.PhaseLossBCnt += myabs(mcApp_I_ABCParam.b);
			App.M1.Prot.PhaseLossCCnt += myabs(mcApp_I_ABCParam.c);
			App.M1.Prot.PhaseLossNum ++;
			if(App.M1.Prot.PhaseLossNum >= MCPara[91]){
				App.M1.Prot.PhaseLossNum = 0;
				App.M1.Prot.PhaseLossSta = 2;
			}
		}
		
	}
	
	if(mcApp_I_ABCParam.t < 0){
		mcApp_I_ABCParam.t = 0;
	}
	
	if(mcApp_SVGenParam.Duty != 0){
		//计算电流
		App.M1.Cal.AllCur = ADC_CURRENT_SCALE_t * mcApp_I_ABCParam.t * 1.0f;// * mcApp_SVGenParam.Duty
		
		App.M1.Cal.AllCurLPF = Filter(App.M1.Cal.AllCurLPF,App.M1.Cal.AllCur,0.005f);
		
	}

	App.M1.Log.CalFg = 1;
	
	
	
	
	//获取ADC值
//	phaseCurrentA = get_shunt_b2_adc_value;
//	phaseCurrentB = get_shunt_a2_adc_value;
//	phaseCurrentT = get_shunt_t2_adc_value;
	
//	phaseCurrentA = get_shunt_a2_adc_value;
//	phaseCurrentB = get_shunt_b2_adc_value;
//	phaseCurrentT = get_shunt_t2_adc_value;
	//获取电压
	mcApp_focParam2.DCBusVoltage_T = (float)phaseVol * VOLTAGE_ADC_TO_PHY_RATIO;
	
	mcApp_focParam2.DCBusVoltage = mcApp_focParam.DCBusVoltage;
	mcApp_focParam2.MaxPhaseVoltage = mcApp_focParam.MaxPhaseVoltage;
	//计算相电流
	
	//正确的
	phaseCurrentA = mcApp_I2_ABCParam.MidAvgA - phaseCurrentA; 
	phaseCurrentB = mcApp_I2_ABCParam.MidAvgB - phaseCurrentB;
	
	mcApp_I2_ABCParam.a = (float)phaseCurrentA * ADC_CURRENT_SCALE;    	
	mcApp_I2_ABCParam.b = (float)phaseCurrentB * ADC_CURRENT_SCALE;
	mcApp_I2_ABCParam.c = -(mcApp_I2_ABCParam.a + mcApp_I2_ABCParam.b);
	
	mcApp_I2_ABCParam.t = (float)phaseCurrentT - mcApp_I2_ABCParam.MidAvgT;
	
	//Clarke
	mcLib_ClarkeTransform(&mcApp_I2_ABCParam, &mcApp_I2_AlphaBetaParam);
	//Park
	mcLib_ParkTransform(&mcApp_I2_AlphaBetaParam,  &mcApp_SincosParam2, 
											&mcApp_I2_DQParam);

	if(App.M2.Hpll.HallorPll == 1){//PLL 估算器给定角度
		mcLib_PLLEstimator(&mcApp_EstimParam2, &mcApp_SincosParam2, &mcApp_focParam2, 
										 &mcApp_ControlParam2, &mcApp_I2_AlphaBetaParam, 
										 &mcApp_V2_AlphaBetaParam);
		//开闭环电流给定
		
		MC_APP_MC_DoControl2();
		//开闭环角度给定
		MC_APP_MC_CalculateParkAngle2();
		//角度给定
		
		if(App.M2.OpenLoop.OpenLoop == 1){
			mcApp_SincosParam2.Angle = App.M2.OpenLoop.Ang;
		}else {
			mcApp_SincosParam2.Angle = mcApp_EstimParam2.qRho;
			
		}
		
	}
	else if(App.M2.Hpll.HallorPll == 2){//hallfoc 给定角度
		hall_cal2();
		//计算速度
		//hall运行时
		pll_run2(App.M2.Hall.Ang, App.M2.CalSpd.Dt, &App.M2.CalSpd.OldRad, &App.M2.CalSpd.NowRad);//这里计算出FOC现在的角度！
		if(App.M2.CalSpd.NowRad >= 0){
			App.M2.Dir.Now = CW;
		}else
		{
			App.M2.Dir.Now = CCW;
		}
//		pll_run(App.M2.CalSpd.NowAng, App.M2.CalSpd.Dt, &App.M2.CalSpd.OldRad, &App.M2.CalSpd.NowRad);
		//闭环电流给定
		Hall_FOC_DoControl2();
		//开闭环角度给定
		Hall_FOC_CalculateParkAngle2();
		//角度给定
		if(App.M2.OpenLoop.OpenLoop == 1){//==1 在学习时
			mcApp_SincosParam2.Angle = App.M2.OpenLoop.Ang;			//赋值开环角度给FOC开环去控制
		}else {
			//hall运行时
			//mcApp_SincosParam.Angle = App.M2.Hall.Ang;
			mcApp_SincosParam2.Angle =App.M2.Hall.Ang;		//现在霍尔估算出来的角度！！FOC角度的给定
//			mcApp_SincosParam2.Angle =(App.M2.Hall.Ang+0.5f);		//给定角度+30度测试测试
			
		}
	}else{
	}
	
	//计算角度的SinCos
	mcLib_SinCosGen(&mcApp_SincosParam2); 
	//逆Park变化
	mcLib_InvParkTransform(&mcApp_V2_DQParam,&mcApp_SincosParam2, &mcApp_V2_AlphaBetaParam);  
	if(App.M2.AllowRun == 1){
		//Clack逆变换
		mcLib_SVPWMGen(&mcApp_V2_AlphaBetaParam , &mcApp_SVGenParam2); 
		//原理图
		TMR_SetCompare1(TMR8, (uint16_t) mcApp_SVGenParam2.dPWM_A);
		TMR_SetCompare2(TMR8, (uint16_t) mcApp_SVGenParam2.dPWM_B);
		TMR_SetCompare3(TMR8, (uint16_t) mcApp_SVGenParam2.dPWM_C);
		
//		TMR_SetCompare3(TMR8, (uint16_t) mcApp_SVGenParam2.dPWM_A);
//		TMR_SetCompare2(TMR8, (uint16_t) mcApp_SVGenParam2.dPWM_B);
//		TMR_SetCompare1(TMR8, (uint16_t) mcApp_SVGenParam2.dPWM_C);	

		if(App.M2.Prot.PhaseLossSta == 0){
			App.M2.Prot.PhaseLossSta = 1;
			
			App.M2.Prot.PhaseLossNum = 0;
			App.M2.Prot.PhaseLossACnt = 0;
			App.M2.Prot.PhaseLossBCnt = 0;
			App.M2.Prot.PhaseLossCCnt = 0;
			
		}else if(App.M2.Prot.PhaseLossSta == 1){
			App.M2.Prot.PhaseLossACnt += myabs(mcApp_I2_ABCParam.a);
			App.M2.Prot.PhaseLossBCnt += myabs(mcApp_I2_ABCParam.b);
			App.M2.Prot.PhaseLossCCnt += myabs(mcApp_I2_ABCParam.c);
			App.M2.Prot.PhaseLossNum ++;
			if(App.M2.Prot.PhaseLossNum >= MCPara2[91]){
				App.M2.Prot.PhaseLossNum = 0;
				App.M2.Prot.PhaseLossSta = 2;
			}
		}
	
	}
	if(mcApp_I2_ABCParam.t < 0){
		mcApp_I2_ABCParam.t = 0;
	}
	
	if(mcApp_SVGenParam2.Duty != 0){
		//计算电流
		App.M2.Cal.AllCur = ADC_CURRENT_SCALE_t * mcApp_I2_ABCParam.t * 1.0f;// * mcApp_SVGenParam.Duty
		
		App.M2.Cal.AllCurLPF = Filter(App.M2.Cal.AllCurLPF,App.M2.Cal.AllCur,0.005f);

	}
	App.M2.Log.CalFg = 1;
	
	
	
	
	
	
	#if Debug_vis == 1
//	vis_pos_set_spd = App.M2.Spd.Set;
	vis_pos_set = App.Ctl.OldPrsINSendCnt;
	vis_pos_now = App.Ctl.PrsINSendCnt;
	
//	vis_spd_set_iq = mcApp_Speed2_PIParam.qOut * 100;
//	vis_spd_set = mcApp_Speed2_PIParam.qInRef;
//	vis_spd_now = mcApp_Speed2_PIParam.qInMeas;
	
//	vis_pos_set_spd = App.M1.Spd.Set;
//	vis_pos_set = App.M1.Pos.SetCnt;
//	vis_pos_now = App.M1.Pos.NowCnt;

//	vis_cur_a = mcApp_I2_ABCParam.a * 1000.0f;
//	vis_cur_b = mcApp_I2_ABCParam.b * 1000.0f;
//	vis_cur_c = mcApp_I2_ABCParam.c * 1000.0f;

	#endif
}



//开闭环 电流控制
void MC_APP_MC_DoControl(void){
		float DoControl_Temp1, DoControl_Temp2;
    if(App.M1.OpenLoop.OpenLoop == 1){//开环
        if(App.M1.OpenLoop.ChangeMode == 1){//初始化一次
					App.M1.OpenLoop.ChangeMode = 0;

					mcApp_ControlParam.IqRef = 0;
					mcApp_ControlParam.IdRef = 0;

					App.M1.OpenLoop.LocateTimCnt = 0;//开环时间
					App.M1.OpenLoop.RampAngleRadsPerSec = 0;//开环角度累加
					App.M1.OpenLoop.Ang = 0;//开环角度
        }
				
				//开环给定电流
				
				//如果用Id来拖，那么开环值是正的
				if(mcApp_ControlParam.IdRef < App.M1.OpenLoop.RampupCurD){
					mcApp_ControlParam.IdRef += App.M1.OpenLoop.RampupCurD / 10.0f;
				}
				
				
				//如果用Iq来拖，开环值
				if(App.M1.Dir.Now == CW){
					mcApp_ControlParam.IqRef = App.M1.OpenLoop.RampupCurQ;
				}else {
					mcApp_ControlParam.IqRef = -App.M1.OpenLoop.RampupCurQ;
				}
				
				
        mcApp_Q_PIParam.qInMeas = mcApp_I_DQParam.q;
        mcApp_Q_PIParam.qInRef  = mcApp_ControlParam.IqRef;
        mcLib_CalcPI(&mcApp_Q_PIParam);
        mcApp_V_DQParam.q = mcApp_Q_PIParam.qOut;
				
        mcApp_D_PIParam.qInMeas = mcApp_I_DQParam.d;
        mcApp_D_PIParam.qInRef  = mcApp_ControlParam.IdRef;
        mcLib_CalcPI(&mcApp_D_PIParam);
        mcApp_V_DQParam.d = mcApp_D_PIParam.qOut;
    }else{//闭环控制
	    	if(App.M1.OpenLoop.ChangeMode == 1){//闭环量初始化
            App.M1.OpenLoop.ChangeMode = 0;
						
						//如果是CW方向，那么Iqref 的初始值是正的
						if(App.M1.Dir.Now == CW){
							if(mcApp_ControlParam.IqRef != 0){//如果Iqref != 0,那么就是用Iq 来开环拖
								mcApp_ControlParam.IqRef = App.M1.OpenLoop.RampupCurQ;
							}else {//如果Iqref == 0,那么就是用Id 来开环拖
								mcApp_ControlParam.IqRef = mcApp_ControlParam.IdRef;
							}
						}
						//如果是CCW方向，那么Iqref 的初始值是负的
						else {
							if(mcApp_ControlParam.IqRef != 0){//如果Iqref != 0,那么就是用Iq 来开环拖
								mcApp_ControlParam.IqRef = -App.M1.OpenLoop.RampupCurQ;
							}else {//如果Iqref == 0,那么就是用Id 来开环拖
								mcApp_ControlParam.IqRef = -App.M1.OpenLoop.RampupCurD;
							}
						}
						
						mcApp_Speed_PIParam.qdSum = mcApp_ControlParam.IqRef;
						mcApp_Speed_PIParam.qOut = mcApp_ControlParam.IqRef;
						
					mcApp_ControlParam.IdRef = 0;
						App.M1.OpenLoop.CloLoopCurD = 0;
						//初始转速赋值
						mcApp_ControlParam.VelRef = mcApp_EstimParam.qVelEstim;
						//设置转速赋值
						App.M1.Spd.RadSet = ((App.M1.Spd.Set / 60.0f) * (2.0f*M_PI)) * MCPara[1];
        }
				
				//弱磁控制
				Id_Ctl();
				
				//转速闭环
				Spd_PI_Out();
				
        // PI control for D
        mcApp_D_PIParam.qInMeas = mcApp_I_DQParam.d;          // This is in Amps
        mcApp_D_PIParam.qInRef  = mcApp_ControlParam.IdRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_D_PIParam);
        mcApp_V_DQParam.d    =  mcApp_D_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)
				
        DoControl_Temp2 = mcApp_D_PIParam.qOut * mcApp_D_PIParam.qOut;
        DoControl_Temp1 = sqrtMax - DoControl_Temp2;
        mcApp_Q_PIParam.qOutMax = sqrt(DoControl_Temp1);
		mcApp_Q_PIParam.qOutMin = -mcApp_Q_PIParam.qOutMax;        
				
        mcApp_Q_PIParam.qInMeas = mcApp_I_DQParam.q;          // This is in Amps
        mcApp_Q_PIParam.qInRef  = mcApp_ControlParam.IqRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_Q_PIParam);
        mcApp_V_DQParam.q    = mcApp_Q_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)       
    }

}




//开闭环 电流控制
void MC_APP_MC_DoControl2(void){
		float DoControl_Temp1, DoControl_Temp2;
    if(App.M2.OpenLoop.OpenLoop == 1){//开环
        if(App.M2.OpenLoop.ChangeMode == 1){//初始化一次
					App.M2.OpenLoop.ChangeMode = 0;

					mcApp_ControlParam2.IqRef = 0;
					mcApp_ControlParam2.IdRef = 0;

					App.M2.OpenLoop.LocateTimCnt = 0;//开环时间
					App.M2.OpenLoop.RampAngleRadsPerSec = 0;//开环角度累加
					App.M2.OpenLoop.Ang = 0;//开环角度
        }
				
				//开环给定电流
				
				//如果用Id来拖，那么开环值是正的
				if(mcApp_ControlParam2.IdRef < App.M2.OpenLoop.RampupCurD){
					mcApp_ControlParam2.IdRef += App.M2.OpenLoop.RampupCurD / 10.0f;
				}
				
				
				//如果用Iq来拖，开环值
				if(App.M2.Dir.Now == CW){
					mcApp_ControlParam2.IqRef = App.M2.OpenLoop.RampupCurQ;
				}else {
					mcApp_ControlParam2.IqRef = -App.M2.OpenLoop.RampupCurQ;
				}
				
				
        mcApp_Q2_PIParam.qInMeas = mcApp_I2_DQParam.q;
        mcApp_Q2_PIParam.qInRef  = mcApp_ControlParam2.IqRef;
        mcLib_CalcPI(&mcApp_Q2_PIParam);
        mcApp_V2_DQParam.q = mcApp_Q2_PIParam.qOut;
				
        mcApp_D2_PIParam.qInMeas = mcApp_I2_DQParam.d;
        mcApp_D2_PIParam.qInRef  = mcApp_ControlParam2.IdRef;
        mcLib_CalcPI(&mcApp_D2_PIParam);
        mcApp_V2_DQParam.d = mcApp_D2_PIParam.qOut;
    }else{//闭环控制
	    	if(App.M2.OpenLoop.ChangeMode == 1){//闭环量初始化
            App.M2.OpenLoop.ChangeMode = 0;
						
						//如果是CW方向，那么Iqref 的初始值是正的
						if(App.M2.Dir.Now == CW){
							if(mcApp_ControlParam2.IqRef != 0){//如果Iqref != 0,那么就是用Iq 来开环拖
								mcApp_ControlParam2.IqRef = App.M2.OpenLoop.RampupCurQ;
							}else {//如果Iqref == 0,那么就是用Id 来开环拖
								mcApp_ControlParam2.IqRef = mcApp_ControlParam2.IdRef;
							}
						}
						//如果是CCW方向，那么Iqref 的初始值是负的
						else {
							if(mcApp_ControlParam2.IqRef != 0){//如果Iqref != 0,那么就是用Iq 来开环拖
								mcApp_ControlParam2.IqRef = -App.M2.OpenLoop.RampupCurQ;
							}else {//如果Iqref == 0,那么就是用Id 来开环拖
								mcApp_ControlParam2.IqRef = -App.M2.OpenLoop.RampupCurD;
							}
						}
						
						mcApp_Speed2_PIParam.qdSum = mcApp_ControlParam2.IqRef;
						mcApp_Speed2_PIParam.qOut = mcApp_ControlParam2.IqRef;
						
            mcApp_ControlParam2.IdRef = 0;
						App.M2.OpenLoop.CloLoopCurD = 0;
						//初始转速赋值
						mcApp_ControlParam2.VelRef = mcApp_EstimParam2.qVelEstim;
						//设置转速赋值
						App.M2.Spd.RadSet = ((App.M2.Spd.Set / 60.0f) * (2.0f*M_PI)) * MCPara2[1];
        }
				
				//弱磁控制
				Id_Ctl2();
				
				//转速闭环
				Spd_PI_Out2();
				
        // PI control for D
        mcApp_D2_PIParam.qInMeas = mcApp_I2_DQParam.d;          // This is in Amps
        mcApp_D2_PIParam.qInRef  = mcApp_ControlParam2.IdRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_D2_PIParam);
        mcApp_V2_DQParam.d    =  mcApp_D2_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)
				
        DoControl_Temp2 = mcApp_D2_PIParam.qOut * mcApp_D2_PIParam.qOut;
        DoControl_Temp1 = sqrtMax - DoControl_Temp2;
        mcApp_Q2_PIParam.qOutMax = sqrt(DoControl_Temp1);
		mcApp_Q2_PIParam.qOutMin = -mcApp_Q2_PIParam.qOutMax;        
				
        mcApp_Q2_PIParam.qInMeas = mcApp_I2_DQParam.q;          // This is in Amps
        mcApp_Q2_PIParam.qInRef  = mcApp_ControlParam2.IqRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_Q2_PIParam);
        mcApp_V2_DQParam.q    = mcApp_Q2_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)       
    }

}






//开环定位和角度累加
//闭环初始角度偏移
void MC_APP_MC_CalculateParkAngle(void){
	if(App.M1.OpenLoop.OpenLoop == 1){                                                 //开环运行
		if(App.M1.OpenLoop.LocateTimCnt < App.M1.OpenLoop.LocateTimSum){                 //启动前定位 时间 
			App.M1.OpenLoop.LocateTimCnt ++;
			
			mcApp_EstimParam.qOmegaMr = 0;
			mcApp_EstimParam.qRho = 0;
			
		}else if (App.M1.OpenLoop.RampAngleRadsPerSec < App.M1.OpenLoop.RampupRad2SecLooptime){//定位完成，在设定时间内加速到设定速度
			App.M1.OpenLoop.RampAngleRadsPerSec += App.M1.OpenLoop.RampupInc;
		}else{                                                                          //跳转到闭环
       if(App.M1.OpenLoop.AngCtlMode > 0){//如果是开环模式，不跳转
					App.M1.OpenLoop.ChangeMode = 1;
					App.M1.OpenLoop.OpenLoop = 0;
       }
		}
		
		//根据方向给定开环拖角度
		if(App.M1.Dir.Now == CW){
			App.M1.OpenLoop.Ang += App.M1.OpenLoop.RampAngleRadsPerSec;
		}else {
			App.M1.OpenLoop.Ang -= App.M1.OpenLoop.RampAngleRadsPerSec;
		}
		
    if(App.M1.OpenLoop.Ang > ANGLE_2PI){
			App.M1.OpenLoop.Ang -= ANGLE_2PI;
		}else if(App.M1.OpenLoop.Ang < 0){
			App.M1.OpenLoop.Ang += ANGLE_2PI;
		}
  }else{                                                                            //闭环运行   
		
		//如果是CW，角度偏移值校正往0方向校正
		if(App.M1.Dir.Now == CW){
			if(mcApp_EstimParam.RhoOffset > RhoOffsetMin){
				mcApp_EstimParam.RhoOffset -= RhoOffsetMin ; 
			}
		}
		//如果是CCW，角度偏移值往2Pi方向校正
		else {
			if(mcApp_EstimParam.RhoOffset > M_PI){
				if(mcApp_EstimParam.RhoOffset < ANGLE_2PI){
					mcApp_EstimParam.RhoOffset += RhoOffsetMin ; 
				}
			}else {
				if(mcApp_EstimParam.RhoOffset > RhoOffsetMin){
					mcApp_EstimParam.RhoOffset -= RhoOffsetMin ; 
				}
			}
		}
	}
}




void MC_APP_MC_CalculateParkAngle2(void){
	if(App.M2.OpenLoop.OpenLoop == 1){                                                 //开环运行
		if(App.M2.OpenLoop.LocateTimCnt < App.M2.OpenLoop.LocateTimSum){                 //启动前定位 时间 
			App.M2.OpenLoop.LocateTimCnt ++;
			
			mcApp_EstimParam2.qOmegaMr = 0;
			mcApp_EstimParam2.qRho = 0;
			
		}else if (App.M2.OpenLoop.RampAngleRadsPerSec < App.M2.OpenLoop.RampupRad2SecLooptime){//定位完成，在设定时间内加速到设定速度
			App.M2.OpenLoop.RampAngleRadsPerSec += App.M2.OpenLoop.RampupInc;
		}else{                                                                          //跳转到闭环
       if(App.M2.OpenLoop.AngCtlMode > 0){//如果是开环模式，不跳转
					App.M2.OpenLoop.ChangeMode = 1;
					App.M2.OpenLoop.OpenLoop = 0;
       }
		}
		
		//根据方向给定开环拖角度
		if(App.M2.Dir.Now == CW){
			App.M2.OpenLoop.Ang += App.M2.OpenLoop.RampAngleRadsPerSec;
		}else {
			App.M2.OpenLoop.Ang -= App.M2.OpenLoop.RampAngleRadsPerSec;
		}
		
    if(App.M2.OpenLoop.Ang > ANGLE_2PI){
			App.M2.OpenLoop.Ang -= ANGLE_2PI;
		}else if(App.M2.OpenLoop.Ang < 0){
			App.M2.OpenLoop.Ang += ANGLE_2PI;
		}
  }else{                                                                            //闭环运行   
		
		//如果是CW，角度偏移值校正往0方向校正
		if(App.M2.Dir.Now == CW){
			if(mcApp_EstimParam2.RhoOffset > RhoOffsetMin){
				mcApp_EstimParam2.RhoOffset -= RhoOffsetMin ; 
			}
		}
		//如果是CCW，角度偏移值往2Pi方向校正
		else {
			if(mcApp_EstimParam2.RhoOffset > M_PI){
				if(mcApp_EstimParam2.RhoOffset < ANGLE_2PI){
					mcApp_EstimParam2.RhoOffset += RhoOffsetMin ; 
				}
			}else {
				if(mcApp_EstimParam2.RhoOffset > RhoOffsetMin){
					mcApp_EstimParam2.RhoOffset -= RhoOffsetMin ; 
				}
			}
		}
	}
	
	
}






void CalSpd_Init(void){
	App.M1.CalSpd.Dt = 1.0f / ((float)PWM_FREQUENCY);
	App.M2.CalSpd.Dt = 1.0f / ((float)PWM_FREQUENCY);

}
//把角度规范到 -Π---+Π之间，
void utils_norm_angle_rad(float *angle) {
	while (*angle < -M_PI) {
		*angle += 2.0f * M_PI;
	}

	while (*angle >  M_PI) {
		*angle -= 2.0f * M_PI;
	}
}



//计算转速
/***********************************************************************
函数名		：pll_run:编码器运行时转速的计算
phase		：App.M2.CalSpd.NowAng：这一时刻的角度
dt			：   △t，也就是时间的最小单位=1/16k
phase_var	：App.M2.CalSpd.OldRad：上一次的角度
speed_var	：计算得到的转速
************************************************************************/
void pll_run(float phase, float dt, volatile float *phase_var,volatile float *speed_var) {
	float delta_theta = 0;		//角度差
	delta_theta = phase - *phase_var;				//霍尔得到的角度
	utils_norm_angle_rad(&delta_theta);//规范角度范围
	*phase_var += (*speed_var + 100.0f * delta_theta) * dt;			//100
	utils_norm_angle_rad((float*)phase_var);
	*speed_var += 90000.0f * delta_theta * dt;				//90000

}


void pll_run2(float phase, float dt, volatile float *phase_var,volatile float *speed_var) {
	float delta_theta = 0;		//角度差
	delta_theta = phase - *phase_var;				//霍尔得到的角度
	utils_norm_angle_rad(&delta_theta);//规范角度范围
	*phase_var += (*speed_var + 100.0f * delta_theta) * dt;		
	utils_norm_angle_rad((float*)phase_var);
	*speed_var += 90000.0f * delta_theta * dt;		
}



//所有PI 初始化
void mcApp_InitControlParameters(void){
	mcApp_D_PIParam.qKp = (float)MCPara[20] / 10000.0f;     
	mcApp_D_PIParam.qKi = (float)MCPara[21] / 1000000.0f;           
	mcApp_D_PIParam.qKc = 0.5f;//mcApp_D_PIParam.qKi * 1.2f;      
	if(mcApp_D_PIParam.qKc > 0.5f){
		mcApp_D_PIParam.qKc = 0.5f;
	} 	
	mcApp_D_PIParam.qOutMax = D_CURRCNTR_OUTMAX;
	mcApp_D_PIParam.qOutMin = -mcApp_D_PIParam.qOutMax;
	mcLib_InitPI(&mcApp_D_PIParam);

	mcApp_Q_PIParam.qKp = (float)MCPara[18] / 10000.0f; 
	mcApp_Q_PIParam.qKi = (float)MCPara[19] / 1000000.0f; 
	mcApp_Q_PIParam.qKc = 0.5f;//mcApp_Q_PIParam.qKi * 1.2f;
	if(mcApp_Q_PIParam.qKc > 0.5f){
		mcApp_Q_PIParam.qKc = 0.5f;
	}
	mcApp_Q_PIParam.qOutMax = Q_CURRCNTR_OUTMAX;
	mcApp_Q_PIParam.qOutMin = -mcApp_Q_PIParam.qOutMax;
	mcLib_InitPI(&mcApp_Q_PIParam);

	mcApp_Speed_PIParam.qKp = (float)MCPara[22] / 10000.0f;        
	mcApp_Speed_PIParam.qKi = (float)MCPara[23] / 1000000.0f;     
	mcApp_Speed_PIParam.qKc = 0.5f;//mcApp_Speed_PIParam.qKi * 5.2f;   
	if(mcApp_Speed_PIParam.qKc > 0.5f){
		mcApp_Speed_PIParam.qKc = 0.5f;
	}
	mcApp_Speed_PIParam.qOutMax = (float)MCPara[6] / 100.0f;   
	mcApp_Speed_PIParam.qOutMin = -mcApp_Speed_PIParam.qOutMax;  
	mcLib_InitPI(&mcApp_Speed_PIParam);
	
//	mcApp_VoltageP_PIParam.qKp  = (float)MCPara[130] / 10000.0f;
//	mcApp_VoltageP_PIParam.qKi  = (float)MCPara[131] / 1000000.0f; 
//	mcApp_VoltageP_PIParam.qKc	= 0.5f;
//	if(mcApp_VoltageP_PIParam.qKc > 0.5f){
//		mcApp_VoltageP_PIParam.qKc = 0.5f;
//	}
//	mcApp_VoltageP_PIParam.qOutMax = (float)MCPara[132];
//	mcApp_VoltageP_PIParam.qOutMin = (float)MCPara[133];
//	mcLib_InitPI(&mcApp_VoltageP_PIParam);
	
	if(App.M1.Hpll.HallorPll == 1){//foc pll
		//电流环
		mcApp_Cur_PIParam.qKp = (float)MCPara[60] / 1.0f;        
		mcApp_Cur_PIParam.qKi = (float)MCPara[61] / 10.0f;     
		mcApp_Cur_PIParam.qKc = 0.5f;
		if(mcApp_Cur_PIParam.qKc > 0.5f){
			mcApp_Cur_PIParam.qKc = 0.5f;
		}
		mcApp_Cur_PIParam.qOutMax = ((MCPara[9] / 60.0f) * (2.0f * M_PI)) * MCPara[1];//最大转速  
		mcApp_Cur_PIParam.qOutMin = ((MCPara[8] / 60.0f) * (2.0f * M_PI)) * MCPara[1];//最小转速
		mcLib_InitPI(&mcApp_Cur_PIParam);
		
	}else if(App.M1.Hpll.HallorPll == 2){//foc hall
		//电流环
		mcApp_Cur_PIParam.qKp = (float)MCPara[60] / 1000.0f;        
		mcApp_Cur_PIParam.qKi = (float)MCPara[61] / 100000.0f;     
		mcApp_Cur_PIParam.qKc = 0.5f;
		if(mcApp_Cur_PIParam.qKc > 0.5f){
			mcApp_Cur_PIParam.qKc = 0.5f;
		}
		mcApp_Cur_PIParam.qOutMax = (float)MCPara[6] / 100.0f;//最大电流参考
		mcApp_Cur_PIParam.qOutMin = - mcApp_Cur_PIParam.qOutMax;//最大电流参考
		mcLib_InitPI(&mcApp_Cur_PIParam);

	}
	
}



//所有PI 初始化
void mcApp_InitControlParameters2(void){
	mcApp_D2_PIParam.qKp = (float)MCPara2[20] / 10000.0f;     
	mcApp_D2_PIParam.qKi = (float)MCPara2[21] / 1000000.0f;           
	mcApp_D2_PIParam.qKc = 0.5f;//mcApp_D_PIParam.qKi * 1.2f;      
	if(mcApp_D2_PIParam.qKc > 0.5f){
		mcApp_D2_PIParam.qKc = 0.5f;
	} 	
	mcApp_D2_PIParam.qOutMax = D_CURRCNTR_OUTMAX;
	mcApp_D2_PIParam.qOutMin = -mcApp_D2_PIParam.qOutMax;
	mcLib_InitPI(&mcApp_D2_PIParam);

	mcApp_Q2_PIParam.qKp = (float)MCPara2[18] / 10000.0f; 
	mcApp_Q2_PIParam.qKi = (float)MCPara2[19] / 1000000.0f; 
	mcApp_Q2_PIParam.qKc = 0.5f;//mcApp_Q_PIParam.qKi * 1.2f;
	if(mcApp_Q2_PIParam.qKc > 0.5f){
		mcApp_Q2_PIParam.qKc = 0.5f;
	}
	mcApp_Q2_PIParam.qOutMax = Q_CURRCNTR_OUTMAX;
	mcApp_Q2_PIParam.qOutMin = -mcApp_Q2_PIParam.qOutMax;
	mcLib_InitPI(&mcApp_Q2_PIParam);

	mcApp_Speed2_PIParam.qKp = (float)MCPara2[22] / 10000.0f;        
	mcApp_Speed2_PIParam.qKi = (float)MCPara2[23] / 1000000.0f;     
	mcApp_Speed2_PIParam.qKc = 0.5f;//mcApp_Speed_PIParam.qKi * 5.2f;   
	if(mcApp_Speed2_PIParam.qKc > 0.5f){
		mcApp_Speed2_PIParam.qKc = 0.5f;
	}
	mcApp_Speed2_PIParam.qOutMax = (float)MCPara2[6] / 100.0f;   
	mcApp_Speed2_PIParam.qOutMin = -mcApp_Speed2_PIParam.qOutMax;  
	mcLib_InitPI(&mcApp_Speed2_PIParam);
	
	if(App.M2.Hpll.HallorPll == 1){//foc pll
		//电流环
		mcApp_Cur2_PIParam.qKp = (float)MCPara2[60] / 1.0f;        
		mcApp_Cur2_PIParam.qKi = (float)MCPara2[61] / 10.0f;     
		mcApp_Cur2_PIParam.qKc = 0.5f;
		if(mcApp_Cur2_PIParam.qKc > 0.5f){
			mcApp_Cur2_PIParam.qKc = 0.5f;
		}
		mcApp_Cur2_PIParam.qOutMax = ((MCPara2[9] / 60.0f) * (2.0f * M_PI)) * MCPara2[1];//最大转速  
		mcApp_Cur2_PIParam.qOutMin = ((MCPara2[8] / 60.0f) * (2.0f * M_PI)) * MCPara2[1];//最小转速
		mcLib_InitPI(&mcApp_Cur2_PIParam);
	}else if(App.M2.Hpll.HallorPll == 2){//foc hall
		//电流环
		mcApp_Cur2_PIParam.qKp = (float)MCPara2[60] / 1000.0f;        
		mcApp_Cur2_PIParam.qKi = (float)MCPara2[61] / 100000.0f;     
		mcApp_Cur2_PIParam.qKc = 0.5f;
		if(mcApp_Cur2_PIParam.qKc > 0.5f){
			mcApp_Cur2_PIParam.qKc = 0.5f;
		}
		mcApp_Cur2_PIParam.qOutMax = (float)MCPara2[6] / 100.0f;//最大电流参考
		mcApp_Cur2_PIParam.qOutMin = - mcApp_Cur2_PIParam.qOutMax;//最大电流参考
		mcLib_InitPI(&mcApp_Cur2_PIParam);

	}
	
}


/*****************************************************************************
函数功能：
PLL 估算器初始化
******************************************************************************/
void mcApp_InitEstimParm(void){		
	CalSpd_Init();
	
	mcApp_EstimParam.qLsDtq = ((float)MCPara[3]/L_coefficient)/LOOPTIME_SEC;   //(float)(MOTOR_PER_PHASE_INDUCTANCE/LOOPTIME_SEC);
	mcApp_EstimParam.qLsDtd = ((float)MCPara[4]/L_coefficient)/LOOPTIME_SEC; 

	mcApp_EstimParam.qRs = (float)MCPara[2]/R_coefficient;  // MOTOR_PER_PHASE_RESISTANCE;

//	mcApp_EstimParam.qKFi = -((((float)MCPara[5]/BEMF_coefficient)/(1.732f)/(1000.0f))*(60.0f))/(2.0f*M_PI)/(float)MCPara[1];
	mcApp_EstimParam.qKFi = ((((float)MCPara[5]/BEMF_coefficient)/(1.732f)/(1000.0f))*(60.0f))/(2.0f*M_PI)/(float)MCPara[1];

	mcApp_EstimParam.qInvKFi_Below_Nominal_Speed = 1.0f/ mcApp_EstimParam.qKFi;//(float)INVKFi_BELOW_BASE_SPEED;

	mcApp_EstimParam.qLs_DIV_2_PI = ((float)MCPara[3]/L_coefficient)/(2.0f*M_PI)   ;//(float) MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI;

	mcApp_EstimParam.qNominal_Speed = (float)(((MCPara[9]/60)*2*M_PI)*(float)MCPara[1])  ;//(float)NOMINAL_SPEED_RAD_PER_SEC_ELEC;
	mcApp_EstimParam.qDecimate_Nominal_Speed = mcApp_EstimParam.qNominal_Speed/10.0f;

	mcApp_EstimParam.qKfilterEsdq = (float)MCPara[24]/32767.0f  ;//KFILTER_ESDQ;
	mcApp_EstimParam.qVelEstimFilterK = (float)MCPara[25]/32767.0f ;//KFILTER_VELESTIM;


	mcApp_EstimParam.qDeltaT = LOOPTIME_SEC;
	mcApp_EstimParam.qSquareDeltaT = LOOPTIME_SEC*LOOPTIME_SEC;

	//角度偏移值给定
	if(App.M1.Dir.Now == CW){
		mcApp_EstimParam.RhoOffset = (float)(MCPara[27] * (M_PI/180.0f));
	}else {
		if(MCPara[27] == 0){
			mcApp_EstimParam.RhoOffset = 0;
		}else {
			mcApp_EstimParam.RhoOffset = ANGLE_2PI - (float)(MCPara[27] * (M_PI/180.0f));
		}
	}
}






void mcApp_InitEstimParm2(void){		
	CalSpd_Init();
	
	mcApp_EstimParam2.qLsDtq = ((float)MCPara2[3]/L_coefficient)/LOOPTIME_SEC;   //(float)(MOTOR_PER_PHASE_INDUCTANCE/LOOPTIME_SEC);
	mcApp_EstimParam2.qLsDtd = ((float)MCPara2[4]/L_coefficient)/LOOPTIME_SEC; 

	mcApp_EstimParam2.qRs = (float)MCPara2[2]/R_coefficient;  // MOTOR_PER_PHASE_RESISTANCE;

	mcApp_EstimParam2.qKFi = ((((float)MCPara2[5]/BEMF_coefficient)/(1.732f)/(1000.0f))*(60.0f))/(2.0f*M_PI)/(float)MCPara2[1];
	mcApp_EstimParam2.qInvKFi_Below_Nominal_Speed = 1.0f/ mcApp_EstimParam2.qKFi;//(float)INVKFi_BELOW_BASE_SPEED;

	mcApp_EstimParam2.qLs_DIV_2_PI = ((float)MCPara2[3]/L_coefficient)/(2.0f*M_PI)   ;//(float) MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI;

	mcApp_EstimParam2.qNominal_Speed = (float)(((MCPara2[9]/60)*2*M_PI)*(float)MCPara2[1])  ;//(float)NOMINAL_SPEED_RAD_PER_SEC_ELEC;
	mcApp_EstimParam2.qDecimate_Nominal_Speed = mcApp_EstimParam2.qNominal_Speed/10.0f;

	mcApp_EstimParam2.qKfilterEsdq = (float)MCPara2[24]/32767.0f  ;//KFILTER_ESDQ;
	mcApp_EstimParam2.qVelEstimFilterK = (float)MCPara2[25]/32767.0f ;//KFILTER_VELESTIM;

	mcApp_EstimParam2.qDeltaT = LOOPTIME_SEC;
	mcApp_EstimParam2.qSquareDeltaT = LOOPTIME_SEC * LOOPTIME_SEC;

	//角度偏移值给定
	if(App.M2.Dir.Now == CW){
		mcApp_EstimParam2.RhoOffset = (float)(MCPara2[27] * (M_PI/180.0f));
	}else {
		if(MCPara2[27] == 0){
			mcApp_EstimParam2.RhoOffset = 0;
		}else {
			mcApp_EstimParam2.RhoOffset = ANGLE_2PI - (float)(MCPara2[27] * (M_PI/180.0f));
		}
	}
	
}




void Spd_PI_Out(void){
	if(App.M1.Hpll.HallorPll == 2){//hallfoc 计算时，可以电流闭环
		
		//电流闭环
		mcApp_Cur_PIParam.qInRef = App.M1.Log.MaxCur;//参考电流
		mcApp_Cur_PIParam.qInMeas = App.M1.Cal.AllCurLPF;
		mcLib_CalcPI1(&mcApp_Cur_PIParam);
		
		mcApp_Speed_PIParam.qOutMax = mcApp_Cur_PIParam.qOut;
		mcApp_Speed_PIParam.qOutMin = -mcApp_Speed_PIParam.qOutMax;
		
		//速度给定
		if(App.M1.Dir.Set == CW){
			mcApp_ControlParam.RadSet = App.M1.Spd.RadSet;
		}else {
			mcApp_ControlParam.RadSet = -App.M1.Spd.RadSet;
		}
		
	}else if(App.M1.Hpll.HallorPll == 1){//foc pll
		#if PLL_Cur_Pi_SW == 1
		mcApp_Cur_PIParam.qOutMax = App.M1.Spd.RadSet;//限制电流环输出最大值
		//电流闭环
		mcApp_Cur_PIParam.qInRef = App.M1.Log.MaxCur;//参考电流
		mcApp_Cur_PIParam.qInMeas = App.M1.Cal.AllCurLPF;
		mcLib_CalcPI1(&mcApp_Cur_PIParam);
		#elif PLL_Cur_Pi_SW == 0
		mcApp_Cur_PIParam.qOut = App.M1.Spd.RadSet;
		#endif
		
		//速度给定
		if(App.M1.Dir.Set == CW){
			mcApp_ControlParam.RadSet = mcApp_Cur_PIParam.qOut;
		}else {
			mcApp_ControlParam.RadSet = -mcApp_Cur_PIParam.qOut;
		}
		
	}
	
	//控制速度的爬升和下降 也就是控制给定值的大小
	if(App.M1.Start == 0 || App.M1.Dir.Set != App.M1.Dir.Now){
		if(mcApp_ControlParam.VelRef > 0){
			mcApp_ControlParam.VelRef -= App.M1.Spd.RampdownDelta;
		}else {
			mcApp_ControlParam.VelRef += App.M1.Spd.RampdownDelta;
		}
	}else {
		if(App.M1.Dir.Set == CW){
			mcApp_ControlParam.Diff = (float)(mcApp_ControlParam.RadSet - mcApp_ControlParam.VelRef);//差值 
			
			if(mcApp_ControlParam.Diff >= App.M1.Spd.RampupDelta){
					mcApp_ControlParam.VelRef += App.M1.Spd.RampupDelta;   
			}else if(mcApp_ControlParam.Diff <= -App.M1.Spd.RampdownDelta){
					mcApp_ControlParam.VelRef -= App.M1.Spd.RampdownDelta; 
			}else{
					mcApp_ControlParam.VelRef = mcApp_ControlParam.RadSet;
			}
			
//			if(App.M1.Hpll.HallorPll == 1){//pll foc
				if(mcApp_ControlParam.VelRef < App.M1.Spd.MinRad){
					mcApp_ControlParam.VelRef = App.M1.Spd.MinRad;
				}
//			}
			
		}else{
			mcApp_ControlParam.Diff = (float)(mcApp_ControlParam.VelRef - mcApp_ControlParam.RadSet);//差值 
			
			if(mcApp_ControlParam.Diff >= App.M1.Spd.RampupDelta){
					mcApp_ControlParam.VelRef -= App.M1.Spd.RampupDelta;   
			}else if(mcApp_ControlParam.Diff <= -App.M1.Spd.RampdownDelta){
					mcApp_ControlParam.VelRef += App.M1.Spd.RampdownDelta; 
			}else{
					mcApp_ControlParam.VelRef = mcApp_ControlParam.RadSet;
			}
			
//			if(App.M1.Hpll.HallorPll == 1){//pll foc
				if(mcApp_ControlParam.VelRef > -App.M1.Spd.MinRad){
					mcApp_ControlParam.VelRef = -App.M1.Spd.MinRad;
				}
//			}
			
		}
	}
	
	
	
	//速度PI 测量值 给定
	if(App.M1.Hpll.HallorPll == 0){//选取速度Pi的 测量值 给定
		
	}else if(App.M1.Hpll.HallorPll == 1){
		
		mcApp_Speed_PIParam.qInMeas = mcApp_EstimParam.qVelEstim;
		
//		mcApp_Speed_PIParam.qInMeas = mcApp_EstimParam.qVelEstim;		
	}else  if(App.M1.Hpll.HallorPll == 2){
		mcApp_Speed_PIParam.qInMeas = App.M1.CalSpd.NowRad;
	}
	
	mcApp_Speed_PIParam.qInRef  = mcApp_ControlParam.VelRef;
	mcLib_CalcPI1(&mcApp_Speed_PIParam);
	mcApp_ControlParam.IqRef = mcApp_Speed_PIParam.qOut;
	
}




void Spd_PI_Out2(void){
	if(App.M2.Hpll.HallorPll == 2){//hallfoc 计算时，可以电流闭环
		
		//电流闭环
		mcApp_Cur2_PIParam.qInRef = App.M2.Log.MaxCur;//参考电流
		mcApp_Cur2_PIParam.qInMeas = App.M2.Cal.AllCurLPF;
		mcLib_CalcPI1(&mcApp_Cur2_PIParam);
		
		mcApp_Speed2_PIParam.qOutMax = mcApp_Cur2_PIParam.qOut;
		mcApp_Speed2_PIParam.qOutMin = -mcApp_Speed2_PIParam.qOutMax;
		
		
		//速度给定
		if(App.M2.Dir.Set == CW){
			mcApp_ControlParam2.RadSet = App.M2.Spd.RadSet;
		}else {
			mcApp_ControlParam2.RadSet = -App.M2.Spd.RadSet;
		}
		
	}else if(App.M2.Hpll.HallorPll == 1){//foc pll
		#if PLL_Cur_Pi_SW == 1
		mcApp_Cur2_PIParam.qOutMax = App.M2.Spd.RadSet;//限制电流环输出最大值
		//电流闭环
		mcApp_Cur2_PIParam.qInRef = App.M2.Log.MaxCur;//参考电流
		mcApp_Cur2_PIParam.qInMeas = App.M2.Cal.AllCurLPF;
		mcLib_CalcPI1(&mcApp_Cur2_PIParam);
		#elif PLL_Cur_Pi_SW == 0
		mcApp_Cur2_PIParam.qOut = App.M2.Spd.RadSet;
		#endif
		
		//速度给定
		if(App.M2.Dir.Set == CW){
			mcApp_ControlParam2.RadSet = mcApp_Cur2_PIParam.qOut;
		}else {
			mcApp_ControlParam2.RadSet = -mcApp_Cur2_PIParam.qOut;
		}
		
	}
	
	//控制速度的爬升和下降 也就是控制给定值的大小
	if(App.M2.Start == 0 || App.M2.Dir.Set != App.M2.Dir.Now){
		if(mcApp_ControlParam2.VelRef > 0){
			mcApp_ControlParam2.VelRef -= App.M2.Spd.RampdownDelta;
		}else {
			mcApp_ControlParam2.VelRef += App.M2.Spd.RampdownDelta;
		}
	}else {
		if(App.M2.Dir.Set == CW){
			mcApp_ControlParam2.Diff = (float)(mcApp_ControlParam2.RadSet - mcApp_ControlParam2.VelRef);//差值 
			
			if(mcApp_ControlParam2.Diff >= App.M2.Spd.RampupDelta){
					mcApp_ControlParam2.VelRef += App.M2.Spd.RampupDelta;   
			}else if(mcApp_ControlParam2.Diff <= -App.M2.Spd.RampdownDelta){
					mcApp_ControlParam2.VelRef -= App.M2.Spd.RampdownDelta; 
			}else{
					mcApp_ControlParam2.VelRef = mcApp_ControlParam2.RadSet;
			}
			
//			if(App.M2.Hpll.HallorPll == 1){//pll foc
				if(mcApp_ControlParam2.VelRef < App.M2.Spd.MinRad){
					mcApp_ControlParam2.VelRef = App.M2.Spd.MinRad;
				}
//			}
			
		}else{
			mcApp_ControlParam2.Diff = (float)(mcApp_ControlParam2.VelRef - mcApp_ControlParam2.RadSet);//差值 
			
			if(mcApp_ControlParam2.Diff >= App.M2.Spd.RampupDelta){
					mcApp_ControlParam2.VelRef -= App.M2.Spd.RampupDelta;   
			}else if(mcApp_ControlParam2.Diff <= -App.M2.Spd.RampdownDelta){
					mcApp_ControlParam2.VelRef += App.M2.Spd.RampdownDelta; 
			}else{
					mcApp_ControlParam2.VelRef = mcApp_ControlParam2.RadSet;
			}
			
//			if(App.M2.Hpll.HallorPll == 1){//pll foc
				if(mcApp_ControlParam2.VelRef > -App.M2.Spd.MinRad){
					mcApp_ControlParam2.VelRef = -App.M2.Spd.MinRad;
				}
//			}
		}
	}
	
	
	
	//速度PI 测量值 给定
	if(App.M2.Hpll.HallorPll == 0){//选取速度Pi的 测量值 给定
		
	}else if(App.M2.Hpll.HallorPll == 1){
		mcApp_Speed2_PIParam.qInMeas = mcApp_EstimParam2.qVelEstim;

	}else  if(App.M2.Hpll.HallorPll == 2){
		mcApp_Speed2_PIParam.qInMeas = App.M2.CalSpd.NowRad;			//速度测量值给定！
	}
	
	mcApp_Speed2_PIParam.qInRef  = mcApp_ControlParam2.VelRef;
	mcLib_CalcPI1(&mcApp_Speed2_PIParam);
	mcApp_ControlParam2.IqRef = mcApp_Speed2_PIParam.qOut;
	
}


void Id_Ctl(void){	

	mcApp_ControlParam.IdRef = 	1.2f;	
//	float  Spd_Now = 0;
//	App.M1.Spd.Id_Cycle ++;
//	if(App.M1.Spd.Id_Cycle > MCPara[110]){
//		App.M1.Spd.Id_Cycle = 0;
//		//取速度的绝对值
//		if (App.M1.Spd.Now >= 0){
//			Spd_Now = App.M1.Spd.Now;
//		}else{
//			Spd_Now = App.M1.Spd.Now * (-1);
//		}
//		//开始回差
//		if(Spd_Now <  MCPara[107] ){		//速度小于Id控制M1转速下限
//			if(mcApp_ControlParam.IdRef < Id_Ctrl_Max ){			//限幅
//				mcApp_ControlParam.IdRef += App.M1.Spd.Id_Up_Down;	
//			}else{
//				mcApp_ControlParam.IdRef = Id_Ctrl_Max;
//			}
//		}
//		else if (Spd_Now > MCPara[108]){//速度大于Id控制M1转速上限
//			mcApp_ControlParam.IdRef -= App.M1.Spd.Id_Up_Down;
//			if(mcApp_ControlParam.IdRef < 0){
//				mcApp_ControlParam.IdRef = 0;
//			}
//		}
//	//	mcApp_ControlParam.IdRef = 1.0f;
//	}else{
//	}
}

void Id_Ctl2(void){
	mcApp_ControlParam2.IdRef = 1.2f;
	
}


