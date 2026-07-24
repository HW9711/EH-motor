#include "fangbo.h"
#include "hallfoc.h"
#include "tim.h"
#include "mclib.h"
#include "gpio.h"
#include "math.h"
#include "mcfoc.h"
#include "mcflash.h"
#include "adc.h"
#include "fangbo_nohall.h"
#include "at32_board.h"
#include "YSstatemachine.h"
#include "YSprotect.h"


//132645 
uint8_t NextCCWhall[7] = {
	0,
	3,//1
	6,//2
	2,//3
	5,//4
	1,//5
	4,//6
};

uint8_t	LastCCWhall[7] = {
	0,
	5,//1
	3,//2
	1,//3
	6,//4
	4,//5
	2,//6
};

//546231
uint8_t NextCWhall[7] = {
	0,
	5,//1
	3,//2
	1,//3
	6,//4
	4,//5
	2,//6
};

uint8_t LastCWhall[7] = {
	0,
	3,//1
	6,//2
	2,//3
	5,//4
	1,//5
	4,//6
};




// 5-4-6-2-3-1
const uint8_t RAMP_TABLE_Hall[7] = {
	5,//0
	4,//1
	6,//2		
	2,//3
	3,//4
	1,//5
	0,//6
};


//方波ADC中断服务函数 
void HallSqu_ADCISRTasks(void){
	volatile short phaseVolP = 0;					//采取电压
	volatile uint8_t i = 0;								//速度读取用
	volatile int8_t j=0;
	volatile int8_t y=0;

	phaseVolP = get_Pvcc_adc_value;			//BUCK降压后电压
	App.FB.Prot.BusVolP = (float)phaseVolP * VOLTAGEP_ADC_PVCC_PHY_RATIO;	//BUCK电路输出电压
	
/************************************无霍尔方波************************************************************************/	
	if((App.Logic.LogicHall_or_pll == 1)&&(App.Logic.CtlMode == 1 ||App.Logic.CtlMode == 2 ||App.Logic.CtlMode == 3)){		//无霍尔的ADC中断，在闭环时在这里进行换相，
	//通道1
		if(App.FB.AllRun == 1){
			if(App.FB.OpenLoop.StartHallOkFg == 1 ){	//相序正确
				//计算速度
					for(i = 0 ; i<6 ;i++){
						App.FB.OpenLoop.SpdSum += App.FB.OpenLoop.SpdCnt_Arr[i];				//累计6次换向的全部时间，更加准
					}
					if(App.FB.OpenLoop.SpdSum != 0 ){
						if(App.FB.Dir.Now ==CW){
							 App.FB.RadFilTemp1 =1000000*(2.0f*M_PI)/(App.FB.OpenLoop.SpdSum * 5.0f);				//0.000005
							App.FB.Spd_Now = App.FB.RadFilTemp1*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm
						}else{
							App.FB.RadFilTemp1 = -1000000*(2.0f*M_PI)/(App.FB.OpenLoop.SpdSum * 5.0f);				//0.000005
							App.FB.Spd_Now = App.FB.RadFilTemp1*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm						
						}
						App.FB.Rad_Now = App.FB.RadFilTemp1*0.1f + App.FB.Rad_Now* 0.9f;
						App.FB.OpenLoop.SpdSum = 0;
				}
			}else if(App.FB.OpenLoop.StartHallOkFg == 0){	//相序还不正确
				//计算速度
					for(i = 0 ; i<6 ;i++){
						App.FB.OpenLoop.SpdSum += App.FB.OpenLoop.SpdCnt_Arr[i];				//累计6次换向的全部时间，更加准
					}
					if(App.FB.OpenLoop.SpdSum != 0 ){
					App.FB.Rad_Now =1000000*(2.0f*M_PI)/(App.FB.OpenLoop.SpdSum * 5.0f);				//0.000005
//					if(App.FB.Rad_Now >300){		//切换角速度，防止速度环过冲，启动转速测量误差较大
//						App.FB.Rad_Now = 300;
//					}
					App.FB.Spd_Now = App.FB.Rad_Now*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm
					App.FB.OpenLoop.SpdSum = 0;			
				}
			}
		}else{
				App.FB.Rad_Now = 0;
				App.FB.Spd_Now = 0;
		}
	//通道2
		if(App.FB2.AllRun == 1){
			if(App.FB2.OpenLoop.StartHallOkFg == 1 ){	//相序正确
				//计算速度
					for(i = 0 ; i<7 ;i++){
						App.FB2.OpenLoop.SpdSum += App.FB2.OpenLoop.SpdCnt_Arr[i];				//累计6次换向的全部时间，更加准
					}
					if(App.FB2.OpenLoop.SpdSum != 0 ){
						if(App.FB2.Dir.Now ==CW){
							App.FB2.RadFilTemp2 =1000000*(2.0f*M_PI)/(App.FB2.OpenLoop.SpdSum * 5.0f);				//0.000005
							App.FB2.Spd_Now = App.FB2.RadFilTemp2*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm	

						}else{
							App.FB2.RadFilTemp2 = -1000000*(2.0f*M_PI)/(App.FB2.OpenLoop.SpdSum * 5.0f);				//0.000005
							App.FB2.Spd_Now = App.FB2.RadFilTemp2*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm
						}
						App.FB2.Rad_Now = App.FB2.RadFilTemp2*0.1f + App.FB2.Rad_Now*0.9f;
					App.FB2.OpenLoop.SpdSum = 0;
				}
			}else if(App.FB2.OpenLoop.StartHallOkFg == 0){	//相序还不正确
				//计算速度
					for(i = 0 ; i<7 ;i++){
						App.FB2.OpenLoop.SpdSum += App.FB2.OpenLoop.SpdCnt_Arr[i];				//累计6次换向的全部时间，更加准
					}
					if(App.FB2.OpenLoop.SpdSum != 0 ){
					App.FB2.Rad_Now =1000000*(2.0f*M_PI)/(App.FB2.OpenLoop.SpdSum * 5.0f);				//0.000005
					App.FB2.Spd_Now = App.FB2.Rad_Now*(60.0f)/(2.0f*M_PI);		//1对极	
					App.FB2.OpenLoop.SpdSum = 0;			
				}
//					if(App.FB2.Rad_Now >300){		//切换角速度，防止速度环过冲，启动转速测量误差较大
//						App.FB2.Rad_Now = 300;
//					}
			}
		}else{
				App.FB2.Rad_Now = 0;
				App.FB2.Spd_Now = 0;	
		}
	}
/************************************有霍尔方波************************************************************************/	
	else if((App.Logic.LogicHall_or_pll == 2 )&&(App.Logic.CtlMode == 1 ||App.Logic.CtlMode == 2 ||App.Logic.CtlMode == 3)){	//有霍尔的ADC中断，在闭环时在这里进行换相，
		//通道1
		App.FB.HallVal = gethall;			//获取Hall值
		if((App.FB.HallVal !=0)&&(App.FB.HallVal !=7)){			//判定霍尔的值不为0和7才给他判定方向
			if((App.FB.HallVal != App.FB.OldHallVal)&&(App.FB.OldHallVal !=0)){
				App.FB.Spd_Enable =0;
			}
		}
		//预判Hall处理，防止干扰打断Hall,需要校验本次的HallVal是否正确，如果不正确，那么需要在换向前，替换掉错误的
		if(App.FB.Dir.Now == CW){
			//判断本次hall在CW方向是否一致
			if(	App.FB.OldHallVal == App.FB.HallVal){	//判断正确
				App.FB.OldHallVal =App.FB.HallVal;//传递旧值
			}else if(App.FB.OldHallVal != App.FB.HallVal){
				if(App.FB.HallVal == NextCCWhall[App.FB.OldHallVal]){
					App.FB.OldHallVal = App.FB.HallVal;
				}else{
					App.FB.HallVal = NextCCWhall[App.FB.OldHallVal];
					App.FB.OldHallVal =App.FB.HallVal;
				}
			}
		}else if(App.FB.Dir.Now == CCW){
			//判断本次hall在CW方向是否一致
			if(	App.FB.OldHallVal == App.FB.HallVal){	//判断正确
				App.FB.OldHallVal =App.FB.HallVal;//传递旧值
			}else if(App.FB.OldHallVal != App.FB.HallVal){
				if(App.FB.HallVal == NextCWhall[App.FB.OldHallVal]){
					App.FB.OldHallVal = App.FB.HallVal;
				}else{
					App.FB.HallVal = NextCWhall[App.FB.OldHallVal];
					App.FB.OldHallVal =App.FB.HallVal;
				}
			}
		}
		
	if(App.FB.AllRun == 1){		//在允许启动才给换相

		Hall_PhaseChange(App.FB.HallVal,7000);		//90%
		if((App.FB.Hall_Timer_Sum_Now != 0)&&(App.FB.Spd_Enable == 0)){
			if(App.FB.Dir.Now == CW){
				App.FB.Rad_Now = 10000000*(2.0f*M_PI)/(App.FB.Hall_Timer_Sum_Now);								//16K计算一次角速度
				App.FB.Spd_Now = App.FB.Rad_Now*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm

			}else{
				App.FB.Rad_Now = -10000000*(2.0f*M_PI)/(App.FB.Hall_Timer_Sum_Now);								//16K计算一次角速度
				App.FB.Spd_Now = App.FB.Rad_Now*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm
			}
		}else{
		App.FB.Rad_Now =0;
		App.FB.Spd_Now =0;	
		}
	}else{
		App.FB.Rad_Now =0;
		App.FB.Spd_Now =0;
	}
	
		//如果电机运行，开始计时
	if(App.FB.AllRun == 1){
		App.FB.FB_Cnt_Time ++;
		if(App.FB.FB_Cnt_Time > 10000)	//1s
		{
			App.FB.FB_Cnt_Time =0;
		}
	}else{
		App.FB.FB_Cnt_Time =0;
	}
/************************************************************************************************************/		
	//电机2
	App.FB2.HallVal = gethall2;
	if((App.FB2.HallVal !=0)||(App.FB2.HallVal!=7)){			//判定霍尔的值不为0和7才给他判定方向
			if((App.FB2.HallVal != App.FB2.OldHallVal)&&(App.FB2.OldHallVal !=0)){
				App.FB2.Spd_Enable =0;
			}
		}
	
		//预判Hall处理，防止干扰打断Hall,需要校验本次的HallVal是否正确，如果不正确，那么需要在换向前，替换掉错误的
		if(App.FB2.Dir.Now == CW){
			//判断本次hall在CW方向是否一致
			if(	App.FB2.OldHallVal == App.FB2.HallVal){	//判断正确
				App.FB2.OldHallVal =App.FB2.HallVal;//传递旧值
			}else if(App.FB2.OldHallVal != App.FB2.HallVal){
				if(App.FB2.HallVal == NextCCWhall[App.FB2.OldHallVal]){
					App.FB2.OldHallVal = App.FB2.HallVal;
				}else{
					App.FB2.HallVal = NextCCWhall[App.FB2.OldHallVal];
					App.FB2.OldHallVal =App.FB2.HallVal;
				}
			}
		}else if(App.FB2.Dir.Now == CCW){
			//判断本次hall在CW方向是否一致
			if(	App.FB2.OldHallVal == App.FB2.HallVal){	//判断正确
				App.FB2.OldHallVal =App.FB2.HallVal;//传递旧值
			}else if(App.FB2.OldHallVal != App.FB2.HallVal){
				if(App.FB2.HallVal == NextCWhall[App.FB2.OldHallVal]){
					App.FB2.OldHallVal = App.FB2.HallVal;
				}else{
					App.FB2.HallVal = NextCWhall[App.FB2.OldHallVal];
					App.FB2.OldHallVal =App.FB2.HallVal;
				}
			}
		}
		
		
		
		
		
		
		
		
		
		
	if(App.FB2.AllRun == 1){
		Hall_PhaseChange2(App.FB2.HallVal,7000);		//90%
	if((App.FB2.Hall_Timer_Sum_Now != 0)&&(App.FB2.Spd_Enable == 0)){
			
			if(App.FB2.Dir.Now == CW){
				App.FB2.Rad_Now = 10000000*(2.0f*M_PI)/(App.FB2.Hall_Timer_Sum_Now);								//16K计算一次角速度
				App.FB2.Spd_Now = App.FB2.Rad_Now*(60.0f)/(2.0f*M_PI);				//1对极		角速度转化为rpm
			}else{
				App.FB2.Rad_Now = -10000000*(2.0f*M_PI)/(App.FB2.Hall_Timer_Sum_Now);								//16K计算一次角速度
				App.FB2.Spd_Now = App.FB2.Rad_Now*(60.0f)/(2.0f*M_PI);				//1对极		角速度转化为rpm
			}
		}else{
		App.FB2.Rad_Now =0;
		App.FB2.Spd_Now =0;	
		}
	}else{
		App.FB2.Rad_Now =0;
		App.FB2.Spd_Now =0;
	}	
		
  }	
	else if(App.Logic.CtlMode == 4 ){				//位置模式，通过指令来控制他移动的步数

	}
	else if(App.Logic.CtlMode == 5){

	}
	//012345
	//546231
	if(App.Pos.LockFlag1 == 1){			//位置定位过,就需要实时刻刻去考虑Hall的位置，记录钻头的位置正负1圈 
		//电机1
		App.Pos.HallValue1 = gethall;		
		if(App.Pos.HallOldValue1 != App.Pos.HallValue1){
			//更新Hall了
				for(j=0 ;j<6;j++){
					if(App.Pos.HallOldValue1 == RAMP_TABLE_Hall[j]){
						if(RAMP_TABLE_Hall[j+1] == App.Pos.HallValue1){							//判断是否正转
							App.Pos.OutSidePosition1 += App.Pos.OutSideAngle1 ;			//正转路径累计
						}else if(RAMP_TABLE_Hall[j-1] == App.Pos.HallValue1){				//判断是否反转
							App.Pos.OutSidePosition1-= App.Pos.OutSideAngle1 ;			//反转路径累计
						}
						else if(j == 0){			//头
							if(RAMP_TABLE_Hall[5] == App.Pos.HallValue1){							//判断是否反转						
								App.Pos.OutSidePosition1-= App.Pos.OutSideAngle1 ;		//反转路径累计
							}
						}else if(j == 5){			//尾
							if(RAMP_TABLE_Hall[0] == App.Pos.HallValue1){								//判断是否正转
								App.Pos.OutSidePosition1 += App.Pos.OutSideAngle1 ;			//正转路径累计
							}							
						}
						break;			//跳出整个for
					}
				}
				if(App.Pos.OutSidePosition1 >= 1.0000f){					//正一圈溢出
					App.Pos.OutSidePosition1 -= 1.0000f;
				}
				else if(App.Pos.OutSidePosition1 <= -1.0000f){		//负一圈溢出
					App.Pos.OutSidePosition1 += 1.0000f;
				}
			App.Pos.HallOldValue1 = App.Pos.HallValue1;//传递旧值
		}
			App.Pos.PlanPosStep1 =(int16_t) (App.Pos.OutSidePosition1 / App.Pos.OutSideAngle1);			//时时刻刻计算步数

	}
	
	
	if(App.Pos.LockFlag2 == 1){
		//电机2		
		App.Pos.HallValue2 = gethall2;		
		if(App.Pos.HallOldValue2 != App.Pos.HallValue2){
			//更新Hall了
				for(y=0 ;y<6;y++){
					if(App.Pos.HallOldValue2 == RAMP_TABLE_Hall[y]){
						if(RAMP_TABLE_Hall[y+1] == App.Pos.HallValue2){							//判断是否正转
							App.Pos.OutSidePosition2 += App.Pos.OutSideAngle2 ;			//正转路径累计
						}else if(RAMP_TABLE_Hall[y-1] == App.Pos.HallValue2){				//判断是否反转
							App.Pos.OutSidePosition2-= App.Pos.OutSideAngle2 ;			//反转路径累计
						}
						else if(y == 0){			//头
							if(RAMP_TABLE_Hall[5] == App.Pos.HallValue2){							//判断是否反转						
								App.Pos.OutSidePosition2-= App.Pos.OutSideAngle2 ;		//反转路径累计
							}
						}else if(y == 5){			//尾
							if(RAMP_TABLE_Hall[0] == App.Pos.HallValue2){								//判断是否正转
								App.Pos.OutSidePosition2 += App.Pos.OutSideAngle2 ;			//正转路径累计
							}							
						}
						break;			//跳出整个for
					}
				}
				if(App.Pos.OutSidePosition2 >= 1.0000f){					//正一圈溢出
					App.Pos.OutSidePosition2 -= 1.0000f;
				}
				else if(App.Pos.OutSidePosition2 <= -1.0000f){		//负一圈溢出
					App.Pos.OutSidePosition2 += 1.0000f;
				}
			App.Pos.HallOldValue2 = App.Pos.HallValue2;//传递旧值
		}
			App.Pos.PlanPosStep2 =(int16_t) (App.Pos.OutSidePosition2 / App.Pos.OutSideAngle2);			//时时刻刻计算步数

	}

}	
	

/************************************************************************************************************/
//方波上电初始化
void Fangbo_Init(void){
//通道1
	App.FB.Status = HS_WAIT;
	App.FB.AllRun = 0;
	App.FB.Dir.Set =CW;
//通道2	
	App.FB2.Status = HS_WAIT;
	App.FB2.AllRun = 0;
	App.FB2.Dir.Set =CW;
//上电需要给一个初始值	
	App.FB.Prot.OverCurrent  = App.FB.Prot.OverCurrent1;
	App.FB2.Prot.OverCurrent = App.FB2.Prot.OverCurrent1; 
}


//	根据速度去电压闭环
void Hall_Vbus_PI_Out(void){
	volatile int32_t temp_pwm = 0;
	
	
	float PWM_UP 	 = 0;
	float PWM_Down = 0;
	
	if(App.Logic.LogicHall_or_pll == 1){
		PWM_UP 	 = MCPara[64];
		PWM_Down = MCPara[65];
	}else{
		PWM_UP 	 = MCPara[36];
		PWM_Down = MCPara[37];
	}
	
	if(App.FB.AllRun ==1){
		//Buck测量值给定
		if(App.FB.Dir.Now == CW){
			mcApp_VoltageP_PIParam.qInMeas = App.FB.Rad_Now;
		}else{
			mcApp_VoltageP_PIParam.qInMeas = -App.FB.Rad_Now;
		}
		//Buck期望值给定
		mcApp_VoltageP_PIParam.qInRef  = App.FB.Rad_Set;
		//Buck的PI计算
		mcLib_CalcPI(&mcApp_VoltageP_PIParam);

			//输出值给temp；
				temp_pwm = (uint16_t)mcApp_VoltageP_PIParam.qOut;
			//给定期望值，但是需要爬坡
				App.Logic.VBusSetPWM = temp_pwm;

			if((App.Logic.VBusNowPWM - App.Logic.VBusSetPWM) < -PWM_UP){
					App.Logic.VBusNowPWM += PWM_UP;
				}else if( (App.Logic.VBusNowPWM - App.Logic.VBusSetPWM) >PWM_Down){
					App.Logic.VBusNowPWM -=PWM_Down;
				}else {
					App.Logic.VBusNowPWM = App.Logic.VBusSetPWM;
				}
				if(App.Logic.VBusNowPWM >mcApp_VoltageP_PIParam.qOutMax){
					App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam.qOutMax ;
				}else if(App.Logic.VBusNowPWM < mcApp_VoltageP_PIParam.qOutMin){
					App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam.qOutMin ;
				} 
				TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);

	
		}
	else{
//		App.Logic.VBusNowPWM = 0;					//都关
//		App.Logic.VBusSetPWM = 0;
//		mcApp_VoltageP_PIParam.qdSum=0;				//都关
//		mcApp_VoltageP_PIParam.qOut=0;	
//		TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);	//都关	
	}
}



//	根据速度去电压闭环
void Hall_Vbus_PI_Out2(void){
	volatile int32_t temp_pwm = 0;
	float PWM_UP2 	 = 0;
	float PWM_Down2 = 0;
	
	if(App.Logic.LogicHall_or_pll == 1){
		PWM_UP2 	 = MCPara2[64];
		PWM_Down2 = MCPara2[65];
	}else{
		PWM_UP2 	 = MCPara2[36];
		PWM_Down2 = MCPara2[37];
	}	
	
	
	if(App.FB2.AllRun ==1){
		//Buck测量值给定
		if(App.FB2.Dir.Now == CW){
			mcApp_VoltageP_PIParam2.qInMeas = App.FB2.Rad_Now;
		}else{
			mcApp_VoltageP_PIParam2.qInMeas = -App.FB2.Rad_Now;
		}
		//Buck期望值给定
		mcApp_VoltageP_PIParam2.qInRef  = App.FB2.Rad_Set;
		//Buck的PI计算
		mcLib_CalcPI(&mcApp_VoltageP_PIParam2);
		//输出值给temp；
		temp_pwm = (uint16_t)mcApp_VoltageP_PIParam2.qOut;
		
		//给定期望值，但是需要爬坡
		App.Logic.VBusSetPWM = temp_pwm;
		if((App.Logic.VBusNowPWM - App.Logic.VBusSetPWM) < -PWM_UP2){
				App.Logic.VBusNowPWM += PWM_UP2;
			}else if( (App.Logic.VBusNowPWM - App.Logic.VBusSetPWM) >PWM_Down2){
				App.Logic.VBusNowPWM -=PWM_Down2;
			}else {
				App.Logic.VBusNowPWM = App.Logic.VBusSetPWM;
			}
			if(App.Logic.VBusNowPWM >mcApp_VoltageP_PIParam2.qOutMax){
				App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam2.qOutMax ;
			}else if(App.Logic.VBusNowPWM < mcApp_VoltageP_PIParam2.qOutMin){
				App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam2.qOutMin ;
			} 
			TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);
		}else{
	}
}


//启动参数生成
void RAMP_TABLE_Init(float tim,float cnt){
	//启动拖动最终转速	=	60*1000/20000 /1/6 +0.5f  = 1
	App.FB.OpenLoop.Step_Min_Tim = (60.0f * tim) / MCPara[25] / MCPara[1] / 6 + 0.5f;
	if(App.FB.OpenLoop.Step_Min_Tim < 1){
		App.FB.OpenLoop.Step_Min_Tim = 1;		//最小值1
	}
	App.FB.OpenLoop.Step_Delay_Tim[63] = App.FB.OpenLoop.Step_Min_Tim;		//赋值给
	
	App.FB.OpenLoop.Step_temp_tim = App.FB.OpenLoop.Step_Min_Tim;
	for(App.FB.OpenLoop.StepCnt = 1;App.FB.OpenLoop.StepCnt < 64; App.FB.OpenLoop.StepCnt ++){
		App.FB.OpenLoop.Step_temp_tim = App.FB.OpenLoop.Step_temp_tim + cnt;		//CNT:拖动步进 40/1000 =0.04
		//规划每一步的换相时间！，不断减少
		App.FB.OpenLoop.Step_Delay_Tim[63 - App.FB.OpenLoop.StepCnt] = App.FB.OpenLoop.Step_temp_tim;
		
	}

}



//启动参数生成2
void RAMP_TABLE_Init2(float tim,float cnt){
	//启动拖动最终转速	=	60*1000/20000 /1/6 +0.5f  = 1
	App.FB2.OpenLoop.Step_Min_Tim = (60.0f * tim) / MCPara2[25] / MCPara2[1] / 6 + 0.5f;
	if(App.FB2.OpenLoop.Step_Min_Tim < 1){
		App.FB2.OpenLoop.Step_Min_Tim = 1;		//最小值1
	}
	//[63]是
	App.FB2.OpenLoop.Step_Delay_Tim[63] = App.FB2.OpenLoop.Step_Min_Tim;		//赋值给
	
	App.FB2.OpenLoop.Step_temp_tim = App.FB2.OpenLoop.Step_Min_Tim;
	for(App.FB2.OpenLoop.StepCnt = 1;App.FB2.OpenLoop.StepCnt < 64; App.FB2.OpenLoop.StepCnt ++){
		App.FB2.OpenLoop.Step_temp_tim = App.FB2.OpenLoop.Step_temp_tim + cnt;		//CNT:拖动步进 40/1000 =0.04
		//规划每一步的换相时间！，不断减少
		App.FB2.OpenLoop.Step_Delay_Tim[63 - App.FB2.OpenLoop.StepCnt] = App.FB2.OpenLoop.Step_temp_tim;
	}
}


//方波状态机各种状态

//等待 
void HallSqu_Wait(void){
	if(App.FB.Start == 1){			//Start由LOGIC决定	：条件：无错误、设定速度大于固定值、有方向、启动电压大于固定值、方向不改变
		App.FB.Status = HS_INIT;	//现在是指令启动
	}else{

	}
}



//等待 
void HallSqu_Wait2(void){
	if(App.FB2.Start == 1){
		App.FB2.Status = HS_INIT;	//现在是指令启动
	}else{

	}
}




//初始化
void HallSqu_Init(void){
	uint8_t i = 0;

	TIME1_PWM_Stop_3Channel();		//防止有刷一起转
	TIME8_PWM_Stop_3Channel();		//防止有刷一起转
	PVCC_PWM_ON;
	for(i = 0 ; i<7 ;i++){			//清空初始化速度计算数组
		App.FB.OpenLoop.SpdCnt_Arr[i] = 0;
	}
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 ||App.Logic.CtlMode ==3 ){
		if(App.Logic.LogicHall_or_pll == 2){	
			HALL2_GPIO_INIT_Disable();		//失能HALL中断
			HALL1_GPIO_INIT();			//有霍尔的话初始化HALL
			TIM3_DISABLE();
		}else{
			TIM_ENABC_Confi();						//无HALL定时器3
			HALL1_GPIO_INIT_Disable();		//失能HALL中断
			HALL2_GPIO_INIT_Disable();		//失能HALL中断
		}
		//启动初始化
		//保护初始化：参数
		if(App.Logic.LogicHall_or_pll == 2){
			if(App.Logic.CtlMode == 3){
				App.FB.Prot.OverCurrent = App.FB.Prot.OverCurrent2;				//有Hall往复运动保护电流
			}else{
				App.FB.Prot.OverCurrent = App.FB.Prot.OverCurrent1;				//有Hall单向运动保护电流
			}
		}else if (App.Logic.LogicHall_or_pll == 1){
			App.FB.Prot.OverCurrent = App.FB.Prot.OverCurrent3;					//无Hall单向运动保护电流
		}
		//速度PI初始化
		VoltageP1_Init();
	}else if (App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){
		HALL1_GPIO_INIT();			//有霍尔的话初始化HALL
		TIM3_DISABLE();	
		//位置模式保护参数初始化
	}
	//充电
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 ||App.Logic.CtlMode ==3 ){
		if(App.Logic.LogicHall_or_pll == 2){					//有霍尔判断要不要快速去闭环。跳过充电	
			if(App.FB.Chang_Flag ==1){		//如果是快速切换转向，就不用再花多时间去充电，直接跳转闭环运行		
				App.FB.Status = HS_START;	
				App.FB.AllRun = 1;//允许运行
				App.FB.Chang_Flag = 0;		//清
			}else{
				App.FB.Status = HS_CHARGE;	//充电
				App.FB.ChaTim1msCnt = 0;		//清除充电时间
			}
		}else{
				App.FB.Status = HS_CHARGE;	//充电
				App.FB.ChaTim1msCnt = 0;		//清除充电时间		
		}
	}else if (App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){		//充电
			App.FB.Status = HS_POSDRAG;	//充电
			App.FB.ChaTim1msCnt = 0;		//清除充电时间		
	}
	App.FB.Dir.Now = App.FB.Dir.Set ;		//给设定的方向
	
	if(App.FB.Dir.Now == CW){
		App.FB.HallVal = gethall;
		App.FB.OldHallVal = LastCWhall[App.FB.HallVal];//传递旧值
	}else{
		App.FB.HallVal = gethall;
		App.FB.OldHallVal = LastCCWhall[App.FB.HallVal];//传递旧值
	}

	
}


//初始化
void HallSqu_Init2(void){
	uint8_t i = 0;
	TIME1_PWM_Stop_3Channel();		//防止有刷一起转
	TIME8_PWM_Stop_3Channel();		//防止有刷一起转
	PVCC_PWM_ON;
	for(i = 0 ; i<7 ;i++){				//清空初始化速度计算数组
		App.FB2.OpenLoop.SpdCnt_Arr[i] = 0;
	}
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 ||App.Logic.CtlMode ==3 ){
		if(App.Logic.LogicHall_or_pll == 2){		//有霍尔
			HALL1_GPIO_INIT_Disable();		//失能HALL中
			HALL2_GPIO_INIT();
			TIM3_DISABLE();
		}else{
//			TIM_HALL_Disable();						//失能定时器5中断
			TIM_ENABC_Confi();						//无HALL定时器3
			HALL1_GPIO_INIT_Disable();		//失能HALL中断
			HALL2_GPIO_INIT_Disable();	//失能HALL2中断
		}
		//启动初始化
		//保护初始化：参数
		if(App.Logic.LogicHall_or_pll == 2){
			if(App.Logic.CtlMode == 3){
				App.FB2.Prot.OverCurrent = App.FB2.Prot.OverCurrent2;				//有Hall往复运动保护电流
			}else{
				App.FB2.Prot.OverCurrent = App.FB2.Prot.OverCurrent1;				//有Hall单向运动保护电流
			}
		}else if (App.Logic.LogicHall_or_pll == 1){
			App.FB2.Prot.OverCurrent = App.FB2.Prot.OverCurrent3;					//无Hall单向运动保护电流
		}
		//速度PI初始化
		VoltageP2_Init();
	}else if (App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){
		HALL2_GPIO_INIT();			//有霍尔的话初始化HALL
		TIM3_DISABLE();	
		//位置模式保护参数初始化
	}
	//充电
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 ||App.Logic.CtlMode ==3 ){
		if(App.Logic.LogicHall_or_pll == 2){			//有霍尔去判断是否需要直接闭环，
			if(App.FB2.Chang_Flag ==1){		//如果是快速切换转向，就不用再花多时间去充电，直接跳转运行
				App.FB2.Status = HS_RUN;	//
				App.FB2.AllRun = 1;				//允许运行
				App.FB2.Chang_Flag = 0;		//清
			}else{
				App.FB2.Status = HS_CHARGE;	
				App.FB2.ChaTim1msCnt = 0;		//清0充电时间
			}
		}else{															//无霍尔就直接去充电
				App.FB2.Status = HS_CHARGE;	
				App.FB2.ChaTim1msCnt = 0;				//清0充电时间				
		}
	}else if (App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){		//充电
			App.FB2.Status = HS_POSDRAG;	//充电
			App.FB2.ChaTim1msCnt = 0;		//清除充电时间		
	}
	App.FB2.Dir.Now = App.FB2.Dir.Set ;		//给设定方向
	
	if(App.FB2.Dir.Now == CW){
		App.FB2.HallVal = gethall2;
		App.FB2.OldHallVal = LastCWhall[App.FB2.HallVal];//传递旧值
	}else{
		App.FB2.HallVal = gethall2;
		App.FB2.OldHallVal = LastCCWhall[App.FB2.HallVal];//传递旧值
	}
}


//充电
void HallSqu_Charge(void){
	if(App.FB.ChaTim1msFg == 1){
		App.FB.ChaTim1msFg = 0;
		App.FB.ChaTim1msCnt ++;
		if(App.FB.ChaTim1msCnt < 10){
			PWM1H_OFF;
			PWM1L_ON;
			PWM2H_OFF;
			PWM2L_OFF;
			PWM3H_OFF;
			PWM3L_OFF;
			PWM1(charge_PWM);
			PWM2(0);
			PWM3(0);
		}else if(App.FB.ChaTim1msCnt < 20){
			PWM1H_OFF;
			PWM1L_OFF;
			PWM2H_OFF;
			PWM2L_ON;
			PWM3H_OFF;
			PWM3L_OFF;
			PWM1(0);
			PWM2(charge_PWM);
			PWM3(0);
		}else if(App.FB.ChaTim1msCnt < 30){
			PWM1H_OFF;
			PWM1L_OFF;
			PWM2H_OFF;
			PWM2L_OFF;
			PWM3H_OFF;
			PWM3L_ON;
			PWM1(0);
			PWM2(0);
			PWM3(charge_PWM);
		}else {
			//充电完成
			if(App.Logic.CtlMode ==1 ||App.Logic.CtlMode==2||App.Logic.CtlMode ==3){	//区分工作模式	
				if(App.Logic.LogicHall_or_pll == 1){//区分是有霍尔还是无霍尔，有霍尔转入运行去闭环，无霍尔的话就需要初始化参数去开环
					RAMP_TABLE_Init(1000,(float)MCPara[26]/1000.0f);
					//启动方式是Vbus，启动力量给定值，限制一下开环BUCK输出的最大最小值
					App.FB.OpenLoop.PwmUp = MCPara[29];
					App.FB.OpenLoop.MinPwm = MCPara[27];
					App.FB.OpenLoop.MaxPwm = MCPara[28];
					App.Logic.VBusNowPWM = 0;
					//相关参数需要归零
					App.FB.OpenLoop.StartHallOkFg = 0;
					App.FB.OpenLoop.StartHallOkCnt = 0;
					App.FB.OpenLoop.OpenLoopTime = 0;
					App.FB.OpenLoop.Spd_NotHallCnt = 0;

					//切入开环
					App.FB.Status =	HS_START;
					App.FB.AllRun = 1;//允许运行
					PWM_3l_Start();		//允许运行
					
				}else if (App.Logic.LogicHall_or_pll == 2){
					App.FB.ChaTim1msCnt = 0;
					PWM_3l_Start();
					App.FB.Status = HS_RUN;//跳转到闭环	//有霍尔就直接去闭环吧，
					App.FB.AllRun = 1;//允许运行
				}
			}
		else if(App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){		//
				//切入开环
				App.FB.Status =	HS_POSDRAG;
				App.FB.AllRun = 1;//允许运行
				PWM_3l_Start();		//允许运行
			}					
		}
	}
	if(App.FB.Err != E_NONE){		//运行中发现错误
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();				//抱住下桥进行刹车
	}
	
}




//充电
void HallSqu_Charge2(void){
	if(App.FB2.ChaTim1msFg == 1){
		App.FB2.ChaTim1msFg = 0;
		
		App.FB2.ChaTim1msCnt ++;
		if(App.FB2.ChaTim1msCnt < 10){
			PWM1H_OFF;
			PWM1L_ON2;
			PWM2H_OFF2;
			PWM2L_OFF2;
			PWM3H_OFF2;
			PWM3L_OFF2;
			PWM12(charge_PWM);
			PWM22(0);
			PWM32(0);
		}else if(App.FB2.ChaTim1msCnt < 20){
			PWM1H_OFF2;
			PWM1L_OFF2;
			PWM2H_OFF2;
			PWM2L_ON2;
			PWM3H_OFF2;
			PWM3L_OFF2;
			PWM12(0);
			PWM22(charge_PWM);
			PWM32(0);
		}else if(App.FB2.ChaTim1msCnt < 30){
			PWM1H_OFF2;
			PWM1L_OFF2;
			PWM2H_OFF2;
			PWM2L_OFF2;
			PWM3H_OFF2;
			PWM3L_ON2;
			PWM12(0);
			PWM22(0);
			PWM32(charge_PWM);
		}else {
			//充电完成
			if(App.Logic.CtlMode == 1 ||App.Logic.CtlMode ==2||App.Logic.CtlMode ==3){
				//区分是有霍尔还是无霍尔，有霍尔转入运行去闭环，无霍尔的话就需要初始化参数去开环
				if(App.Logic.LogicHall_or_pll == 1){
					RAMP_TABLE_Init2(1000,(float)MCPara2[26]/1000.0f);
					//启动方式是Vbus，启动力量给定值，限制一下开环BUCK输出的最大最小值
					App.FB2.OpenLoop.PwmUp = MCPara2[29];
					App.FB2.OpenLoop.MinPwm = MCPara2[27];
					App.FB2.OpenLoop.MaxPwm = MCPara2[28];
					App.FB2.OpenLoop.Spd_NotHallCnt = 0;
					App.Logic.VBusNowPWM = 0;
					//相关参数需要归零
					App.FB2.OpenLoop.StartHallOkFg = 0;
					App.FB2.OpenLoop.StartHallOkCnt = 0;
					App.FB2.OpenLoop.OpenLoopTime = 0;
					//切入开环
					App.FB2.Status =	HS_START;
					App.FB2.AllRun = 1;//允许运行
					PWM_3l_Start2();		//允许运行

				}else if (App.Logic.LogicHall_or_pll == 2){
					App.FB2.ChaTim1msCnt = 0;
					PWM_3l_Start2();
					App.FB2.Status = HS_RUN;//跳转到闭环	//有霍尔就直接去闭环吧，
					App.FB2.AllRun = 1;//允许运行
				}
			}else if(App.Logic.CtlMode ==4||App.Logic.CtlMode ==5){
				//切入开环
				App.FB2.Status =	HS_POSDRAG;
				App.FB2.AllRun = 1;//允许运行
				PWM_3l_Start2();		//允许运行				
			}
		}
	}
	if(App.FB2.Err != E_NONE){		//运行中发现错误
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();				//抱住下桥进行刹车
	}
}

//启动开环，无感方波专用
void HallSqu_Start(void){					//没有开环，那就直接转闭环
	if(App.Logic.LogicHall_or_pll == 1 ){
		if(App.FB.OpenLoop.OpenStart1msFg == 1){
			App.FB.OpenLoop.OpenStart1msFg = 0;
			//对BUCK电路进行PWM给定爬坡，太复杂，先给固定的PWM给占空比看看可不可以拖动起来，后面再修改此部分
			//简单的来赋值VBUS，
			if(App.FB.Prot.AllCur >= ((float)MCPara[58]/100.0f) ){
				App.Logic.VBusNowPWM -= App.FB.OpenLoop.PwmUp; 
			}else{
				//下面是新增的给定爬坡
				App.Logic.VBusNowPWM += App.FB.OpenLoop.PwmUp;
				if(App.Logic.VBusNowPWM >= App.FB.OpenLoop.MaxPwm){
					App.Logic.VBusNowPWM = App.FB.OpenLoop.MaxPwm;
				}else if(App.Logic.VBusNowPWM < App.FB.OpenLoop.MinPwm){
					App.Logic.VBusNowPWM  =	App.FB.OpenLoop.MinPwm;
				}				
			}
			TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);			//固定的PWM去拖，先托起来，调节MCPara[132]可以暂时调节，后面更改成爬坡
			//启动拖
			App.FB.OpenLoop.OpenStartTimCnt ++;
			if(App.FB.OpenLoop.OpenStartTimCnt >= App.FB.OpenLoop.Step_Delay_Tim[App.FB.OpenLoop.StepRampCnt]){
					App.FB.OpenLoop.OpenStartTimCnt = 0;
				//还没有正确的检测到相序，换相慢慢累加
				if(App.FB.OpenLoop.StartHallOkFg == 0){
					App.FB.OpenLoop.StepRampCnt++;	//换相步数增加
					
					if(App.FB.OpenLoop.StepRampCnt  >= STEP_RAMP_SIZE){	//如果大于最大换相步数都还没有检测到正确的相序，那么就报错
						App.FB.OpenLoop.StepRampCnt = STEP_RAMP_SIZE - 1;	//逐渐减小拖动换相的间隔

					}
					
					if(App.FB.Dir.Now == CW){
						App.FB.OpenLoop.HallStartStepCnt ++;
					}else{
						App.FB.OpenLoop.HallStartStepCnt --;
					}
					
					if(App.FB.OpenLoop.HallStartStepCnt < 0){	
						App.FB.OpenLoop.HallStartStepCnt = 5;
					}else if(App.FB.OpenLoop.HallStartStepCnt >5){
						App.FB.OpenLoop.HallStartStepCnt = 0;	
					}
					//开始换相拖动
					App.FB.OpenLoop.HallVal = RAMP_TABLE_Hall[App.FB.OpenLoop.HallStartStepCnt];
					
					App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
					App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清					
					
					Hall_PhaseChange(App.FB.OpenLoop.HallVal,7000);
				}else{
					//检测到正确的相序了，准备转入闭环

				}
			}
		}
		
	}else if(App.Logic.LogicHall_or_pll == 2)
	{
		App.FB.Status = HS_RUN;//跳转到闭环	
	}
	
	if(App.FB.Dir.Now != App.FB.Dir.Set){		//方向发生改变
		App.FB.Status = HS_BRA;

	}
	if(App.FB.Start != 1){
		App.FB.Status = HS_BRA;				//不符合启动条件
	}
	if(App.FB.Err != E_NONE){		//运行中发现错误
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();				//抱住下桥进行刹车
	}
	
}

//通道2启动
void HallSqu_Start2(void){					//没有开环，那就直接转闭环
	
	if(App.Logic.LogicHall_or_pll == 1 ){
		if(App.FB2.OpenLoop.OpenStart1msFg == 1){
			App.FB2.OpenLoop.OpenStart1msFg = 0;

			if(App.FB2.Prot.AllCur >= ((float)MCPara[58]/100.0f) ){
				App.Logic.VBusNowPWM -= App.FB2.OpenLoop.PwmUp; 
			}else{
				//下面是新增的给定爬坡。
				App.Logic.VBusNowPWM += App.FB2.OpenLoop.PwmUp;
				if(App.Logic.VBusNowPWM >= App.FB2.OpenLoop.MaxPwm){
					App.Logic.VBusNowPWM = App.FB2.OpenLoop.MaxPwm;
				}	else if(App.Logic.VBusNowPWM < App.FB2.OpenLoop.MinPwm){
					App.Logic.VBusNowPWM  =	App.FB2.OpenLoop.MinPwm;
				}							
			}
			TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);			//固定的PWM去拖，先托起来，调节MCPara[132]可以暂时调节，后面更改成爬坡
			//启动拖
			App.FB2.OpenLoop.OpenStartTimCnt ++;
			if(App.FB2.OpenLoop.OpenStartTimCnt >= App.FB2.OpenLoop.Step_Delay_Tim[App.FB2.OpenLoop.StepRampCnt]){
					App.FB2.OpenLoop.OpenStartTimCnt = 0;
				//还没有正确的检测到相序，换相慢慢累加
				if(App.FB2.OpenLoop.StartHallOkFg == 0){
					App.FB2.OpenLoop.StepRampCnt++;	//换相步数增加
					if(App.FB2.OpenLoop.StepRampCnt  >= STEP_RAMP_SIZE){	//如果大于最大换相步数都还没有检测到正确的相序，那么就报错
						App.FB2.OpenLoop.StepRampCnt = STEP_RAMP_SIZE - 1;	//逐渐减小拖动换相的间隔

					}
					
					if(App.FB2.Dir.Now == CW){
						App.FB2.OpenLoop.HallStartStepCnt ++;
					}else{
						App.FB2.OpenLoop.HallStartStepCnt --;
					}
					
					if(App.FB2.OpenLoop.HallStartStepCnt < 0){	
						App.FB2.OpenLoop.HallStartStepCnt = 5;
					}else if(App.FB2.OpenLoop.HallStartStepCnt >5){
						App.FB2.OpenLoop.HallStartStepCnt = 0;	
					}
					//开始换相拖动
					App.FB2.OpenLoop.HallVal = RAMP_TABLE_Hall[App.FB2.OpenLoop.HallStartStepCnt];
					
					App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
					App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清					
					
					Hall_PhaseChange2(App.FB2.OpenLoop.HallVal,7000);
				}else{
					//检测到正确的相序了，准备转入闭环

				}
			}
		}
		
	}else if(App.Logic.LogicHall_or_pll == 2)
	{
		App.FB2.Status = HS_RUN;//跳转到闭环	
	}
	
	if(App.FB2.Dir.Now != App.FB2.Dir.Set){		//方向发生改变
		App.FB2.Status = HS_BRA;
	}
	if(App.FB2.Start != 1){										//不符合启动条件
		App.FB2.Status = HS_BRA;
	}
	if(App.FB2.Err != E_NONE){		//运行中发现错误
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();				//抱住下桥进行刹车
	}
}

//运行
void HallSqu_Run(void){
	if(App.FB.Dir.Now != App.FB.Dir.Set){		//方向发生改变
		App.FB.Status = HS_BRA;

	}
	if(App.FB.Start != 1){
		App.FB.Status = HS_BRA;				//不符合启动条件
	}
	
	if(App.FB.Err != E_NONE){		//运行中发现错误
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();				//抱住下桥进行刹车
	}

}

//运行
void HallSqu_Run2(void){
	if(App.FB2.Dir.Now != App.FB2.Dir.Set){		//方向发生改变
		App.FB2.Status = HS_BRA;
	}
	if(App.FB2.Start != 1){										//不符合启动条件
		App.FB2.Status = HS_BRA;
	}
	
	if(App.FB2.Err != E_NONE){		//运行中发现错误
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();				//抱住下桥进行刹车
	}
}
//位置拖动
void HallSqu_Drag(void){
	if(App.Logic.CtlMode == 0 || App.Logic.CtlMode == 1 ||App.Logic.CtlMode == 2||App.Logic.CtlMode == 3||App.Logic.Motor_Kind!=1){
		//退出位置拖动
		App.FB.Status = HS_WAIT;	
		//失能
		PWM_3l_Stop();
		PWM_3l_Stop2();		
		//记录这个位置，后续每次转动都需要停止在这个位置：
		App.Pos.OutSidePosition1 = 0;		
		App.Pos.LockFlag1 = 1;
		//清除在位置模式的临时值	
		App.Pos.PosNow1 = 0x1F;		
	}

	if(App.FB.Err != E_NONE){		//运行中发现错误
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();				//抱住下桥进行刹车
	}	
	
}
//位置拖动2
void HallSqu_Drag2(void){
	if(App.Logic.CtlMode == 0 || App.Logic.CtlMode == 1 ||App.Logic.CtlMode == 2||App.Logic.CtlMode == 3 ||App.Logic.Motor_Kind!=2){
		//退出位置拖动
		App.FB2.Status = HS_WAIT;	
		//失能
		PWM_3l_Stop();
		PWM_3l_Stop2();	
		//记录这个位置，后续每次转动都需要停止在这个位置：
		App.Pos.OutSidePosition2 = 0;		
		App.Pos.LockFlag2 = 1;
		//清除在位置模式的临时值	
		App.Pos.PosNow2 = 0x1F;			
		
	}
	if(App.FB2.Err != E_NONE){		//运行中发现错误
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();				//抱住下桥进行刹车
	}
}


//刹车 刹车方式
void HallSqu_Bra(void){
	if(App.FB.Bra1msFg == 1){
		App.FB.Bra1msFg = 0;
		App.FB.BraTimCnt ++;
		
	if((App.Logic.VBusNowPWM < MCPara[45])||(App.Logic.Brake_TimeCnt >=10)){
			App.Logic.Brake_StopPwmOK=1;
			App.FB.Status = HS_STOP;
			App.FB.BraTimCnt = 0;
			if((App.Logic.CtlMode ==3 )&&(App.Logic.LogicHall_or_pll ==2)){
				App.FB.Chang_Flag = 1;
			}
		}
	}
	if(App.FB.Err != E_NONE){
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();
	}
}



//刹车 刹车方式
void HallSqu_Bra2(void){
	if(App.FB2.Bra1msFg == 1){
		App.FB2.Bra1msFg = 0;
		App.FB2.BraTimCnt ++;
		
	if((App.Logic.VBusNowPWM < MCPara2[45])||(App.Logic.Brake_TimeCnt >=10)){
			App.Logic.Brake_StopPwmOK =1;	
			App.FB2.Status = HS_STOP;
			App.FB2.BraTimCnt = 0;
			if((App.Logic.CtlMode ==3 )&&(App.Logic.LogicHall_or_pll ==2)){
				App.FB2.Chang_Flag = 1;
			}
		}	
	
	}
	if(App.FB2.Err != E_NONE){
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();
	}
}

//位置刹车
void HallSqu_PosBra(void){

	
	
	//如果报错也退出
		if(App.FB.Err != E_NONE){
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();
	}
}

//位置刹车
void HallSqu_PosBra2(void){
	
	
	//如果报错也退出
	if(App.FB2.Err != E_NONE){
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();
	}
}

//停止
void HallSqu_Stop(void){
	volatile int8_t temp =0 ;
	int8_t i = 0;
	if((App.Pos.LockFlag1 == 1)&&(App.Logic.CtlMode !=3)&&(App.Pos.PosFlag == 1)){				//模式3先不给进
		if(App.Pos.WaitStopTimCnt1 > App.Pos.WaitStopTim1){
			App.Pos.PosFlag = 0;
			App.Pos.WaitStopTimCnt1 = 0;
			App.FB.AllRun = 0;
			PWM_3l_Start();		//允许运行
			temp = gethall;						//读取Hall;
			for(i=0; i<7; i++ ){
				if(temp == RAMP_TABLE_Hall[i]){
					App.Pos.Tempi1 = i;
					if(App.Pos.Tempi1== 0){		//往前位移一个数组
						App.Pos.Tempi1 = 5;
					}else{
						App.Pos.Tempi1 --;
					}
					break;
				}
			}
			App.FB.Status = HS_POSBRA;
			App.FB.Prot.HallDragCnt = 0;
			TMR_SetCompare4(TMR2, MCPara[53]);			//
			App.Pos.PlanPosStep1 =(int16_t) (App.Pos.OutSidePosition1 / App.Pos.OutSideAngle1);			//计算本次停止应该执行的步数
		}else{	
			App.FB.AllRun = 0;
			PWM_3l_Stop();
		}
	}else{
		App.FB.Status = HS_WAIT;
		App.FB.AllRun = 0;
		PWM_3l_Stop();
	}
	
}


//停止
void HallSqu_Stop2(void){
	
	volatile int8_t temp =0 ;
	int8_t i = 0;
	if((App.Pos.LockFlag2 == 1)&&(App.Logic.CtlMode !=3)&&(App.Pos.PosFlag == 1)){				//模式3先不给进
		if(App.Pos.WaitStopTimCnt2 > App.Pos.WaitStopTim2){
		App.Pos.PosFlag = 0;
			App.Pos.WaitStopTimCnt2 = 0;
			App.FB2.AllRun = 0;
			PWM_3l_Start2();						//允许运行
			temp = gethall2;						//读取Hall;
			for(i=0; i<7; i++ ){
				if(temp == RAMP_TABLE_Hall[i]){
					App.Pos.Tempi2 = i;
					if(App.Pos.Tempi2== 0){		//往前位移一个数组
						App.Pos.Tempi2 = 5;
					}else{
						App.Pos.Tempi2 --;
					}
					break;
				}
			}
			App.FB2.Status = HS_POSBRA;
			App.FB2.Prot.HallDragCnt = 0;
			TMR_SetCompare4(TMR2, MCPara2[53]);			//
			App.Pos.PlanPosStep2 =(int16_t) (App.Pos.OutSidePosition2 / App.Pos.OutSideAngle2);			//计算本次停止应该执行的步数
		}else{	
		App.FB2.AllRun = 0;
		PWM_3l_Stop2();
		}
	}else{
		App.FB2.Status = HS_WAIT;
		App.FB2.AllRun = 0;
		PWM_3l_Stop2();
	}
}

//错误
void HallSqu_Err(void){
	App.FB.AllRun = 0;
	App.Logic.VBusNowPWM = 0;			//错误需要清零PWM现在值，防止重启时
	PWM_3l_Stop();
	TMR_SetCompare4(TMR2,0);
	if(App.FB.Err == E_NONE){
		App.FB.Status = HS_WAIT;
	}
}

//错误
void HallSqu_Err2(void){
	App.FB2.AllRun = 0;
	App.Logic.VBusNowPWM = 0;			//错误需要清零PWM现在值，防止重启时
	PWM_3l_Stop2();
	TMR_SetCompare4(TMR2,0);
	if(App.FB2.Err == E_NONE){
		App.FB2.Status = HS_WAIT;
	}
}



//状态机控制
void HallSqu_Ctl(void){
	switch(App.FB.Status){
				 case HS_WAIT : //等待
					HallSqu_Wait();
						break;      
				 case HS_INIT:   //初始化
					HallSqu_Init();
						break; 
				 case HS_CHARGE: //充电
					HallSqu_Charge();
						break;
				 case HS_START:  //启动/开环
					HallSqu_Start();
						break;
				 case HS_RUN:    //运行
					HallSqu_Run();
						break;
				 case HS_BRA:    //刹车
					HallSqu_Bra();
						break;
				 case HS_STOP:    //停止
					HallSqu_Stop();
						break; 
				 case HS_ERR:     //错误
					HallSqu_Err();
						break; 
				 case HS_POSBRA:
					 HallSqu_PosBra();//刹车拖动
						break;
				 case HS_POSDRAG:
					 HallSqu_Drag();//拖动
						break;
					 
				 
			   default:
					 break;
	}
}



//状态机控制
void HallSqu_Ctl2(void){
	switch(App.FB2.Status){
				 case HS_WAIT : //等待
					HallSqu_Wait2();
						break;      
				 case HS_INIT:   //初始化
					HallSqu_Init2();
						break; 
				 case HS_CHARGE: //充电
					HallSqu_Charge2();
						break;
				 case HS_START:  //启动/开环
					HallSqu_Start2();
						break;
				 case HS_RUN:    //运行
					HallSqu_Run2();
						break;
				 case HS_BRA:    //刹车
					HallSqu_Bra2();
						break;
				 case HS_STOP:    //停止
					HallSqu_Stop2();
						break; 
				 case HS_ERR:     //错误
					HallSqu_Err2();
						break; 
				 case HS_POSBRA:
					 HallSqu_PosBra2();//刹车拖动
						break;
				 case HS_POSDRAG:
					 HallSqu_Drag2();//拖动
						break;
				 
			   default:
					
					 break;
	}
}



//方波换相
								// 5 1 3 2 6 4		CCW
//换相中断			// 5 4 6 2 3 1		CW
							// AB-AC-BC-BA-CA-CB
void Hall_PhaseChange(u8 hall_val,u16 pwm){	
	
		if(App.FB.Dir.Now == CCW){
			hall_val = 7 - hall_val;		//反方向
		}
		switch(hall_val){
					case 5 : //BA
					
					PWM3H_OFF;
					PWM3L_OFF;
					PWM3(0);
					
					PWM2L_OFF;
					PWM1H_OFF;
					
					PWM1(PWM_ARR);
					PWM2H_ON;

					PWM1L_ON;
					PWM2(PWM_ARR);
												
						break;      
				 case 1:  //CA
					 PWM2H_OFF;
					PWM2L_OFF;
					PWM2(0);
				 
					PWM3L_OFF;
					PWM1H_OFF;
					
					
				 
					PWM1(PWM_ARR);
					PWM3H_ON;

					PWM1L_ON;
					PWM3(PWM_ARR);
					
						break; 
				 case 3: //CB
					 PWM1H_OFF;
					PWM1L_OFF;
					PWM1(0);
				 
					PWM3L_OFF;
					PWM2H_OFF;
					
					
				 
					PWM2(PWM_ARR);
					PWM3H_ON;

					PWM2L_ON;
					PWM3(PWM_ARR);
					
						break;
				 case 2:  //AB
					 PWM3H_OFF;
					PWM3L_OFF;
					PWM3(0);
				 
					PWM1L_OFF;
					PWM2H_OFF;
					
					
				 
					PWM2(PWM_ARR);
					PWM1H_ON;

					PWM2L_ON;
					PWM1(PWM_ARR);
					
						break;
				 case 6:  //AC		
					 PWM2H_OFF;
					PWM2L_OFF;
					PWM2(0);
				 
					PWM1L_OFF;
					PWM3H_OFF;
					
					PWM3(PWM_ARR);
					PWM1H_ON;

					PWM3L_ON;
					PWM1(PWM_ARR);
					
						break;
					case 4: //BC			
						PWM1H_OFF;
					PWM1L_OFF;
					PWM1(0);
					
					PWM2L_OFF;
					PWM3H_OFF;
					
					PWM3(PWM_ARR);
					PWM2H_ON;

					PWM3L_ON;
					PWM2(PWM_ARR);
					
						break; 
					default:

					break;
	}
}




//换相中断
void Hall_PhaseChange2(u8 hall_val,u16 pwm){	
	
		if(App.FB2.Dir.Now == CCW){
			hall_val = 7 - hall_val;		//反方向
		}
		switch(hall_val){
					case 5 : //BA
					
					PWM3H_OFF2;
					PWM3L_OFF2;
					PWM32(0);
					
					PWM2L_OFF2;
					PWM1H_OFF2;
					
					PWM12(PWM_ARR);
					PWM2H_ON2;

					PWM1L_ON2;
					PWM22(PWM_ARR);
					
						break;      
				 case 1:  //CA
					PWM2H_OFF2;
					PWM2L_OFF2;
					PWM22(0);
				 
					PWM3L_OFF2;
					PWM1H_OFF2;
					
					
				 
					PWM12(PWM_ARR);
					PWM3H_ON2;

					PWM1L_ON2;
					PWM32(PWM_ARR);
					
						break; 
				 case 3: //CB
					PWM1H_OFF2;
					PWM1L_OFF2;
					PWM12(0);
				 
					PWM3L_OFF2;
					PWM2H_OFF2;
					
					
				 
					PWM22(PWM_ARR);
					PWM3H_ON2;

					PWM2L_ON2;
					PWM32(PWM_ARR);
					
						break;
				 case 2:  //AB
					 PWM3H_OFF2;
					PWM3L_OFF2;
					PWM32(0);
				 
					PWM1L_OFF2;
					PWM2H_OFF2;
					
					
				 
					PWM22(PWM_ARR);
					PWM1H_ON2;

					PWM2L_ON2;
					PWM12(PWM_ARR);
					
						break;
				 case 6:  //AC
					 PWM2H_OFF2;
					PWM2L_OFF2;
					PWM22(0);
				 
					PWM1L_OFF2;
					PWM3H_OFF2;
					
					PWM32(PWM_ARR);
					PWM1H_ON2;

					PWM3L_ON2;
					PWM12(PWM_ARR);
					
						break;
					case 4: //BC
					PWM1H_OFF2;
					PWM1L_OFF2;
					PWM12(0);
					
					PWM2L_OFF2;
					PWM3H_OFF2;
					
					PWM32(PWM_ARR);
					PWM2H_ON2;

					PWM3L_ON2;
					PWM22(PWM_ARR);
					
						break; 
					default:
					break;
	}
}


//下桥全开
void PWM_3l_L_Open(void){
	PWM1H_OFF;
	PWM2H_OFF;
	PWM3H_OFF;
	
	PWM1L_ON;
	PWM2L_ON;
	PWM3L_ON;
	
	PWM1(PWM_ARR);
	PWM2(PWM_ARR);
	PWM3(PWM_ARR);
}

//全关
void PWM_3l_Stop(void){
	PWM1H_OFF;
	PWM1L_OFF;
	PWM2H_OFF;
	PWM2L_OFF;
	PWM3H_OFF;
	PWM3L_OFF;
	PWM1(0);
	PWM2(0);
	PWM3(0);	
}

//全关
void PWM_3l_Stop2(void){
	PWM1H_OFF2;
	PWM1L_OFF2;
	PWM2H_OFF2;
	PWM2L_OFF2;
	PWM3H_OFF2;
	PWM3L_OFF2;
	PWM12(0);
	PWM22(0);
	PWM32(0);	
}

//全开
void PWM_3l_Start(void){
	PWM1H_ON;
	PWM1L_ON;
	PWM2H_ON;
	PWM2L_ON;
	PWM3H_ON;
	PWM3L_ON;
	PWM1(0);
	PWM2(0);
	PWM3(0);
}
//全开
void PWM_3l_Start2(void){
	PWM1H_ON2;
	PWM1L_ON2;
	PWM2H_ON2;
	PWM2L_ON2;
	PWM3H_ON2;
	PWM3L_ON2;
	PWM12(0);
	PWM22(0);
	PWM32(0);
}       

//通道1电压环初始化
void VoltageP1_Init(void){
	if(App.Logic.CtlMode == 3){		//往复运动的PID参数
	mcApp_VoltageP_PIParam.qKp  = (float)MCPara[46]/ 1000.0f;
	mcApp_VoltageP_PIParam.qKi  = (float)MCPara[47]/ 1000.0f;
	mcApp_VoltageP_PIParam.qKc	= (float)MCPara[48]/ 1000.0f;
	}else{												//单向运动的PID参数
		if(App.Logic.LogicHall_or_pll == 2){	//有Hall单向运动
		mcApp_VoltageP_PIParam.qKp  = (float)MCPara[31]/ 1000.0f;
		mcApp_VoltageP_PIParam.qKi  = (float)MCPara[32]/ 1000.0f;
		mcApp_VoltageP_PIParam.qKc	= (float)MCPara[33]/ 1000.0f;
		}else{		//无Hall单向运动
		mcApp_VoltageP_PIParam.qKp  = (float)MCPara[61]/ 1000.0f;
		mcApp_VoltageP_PIParam.qKi  = (float)MCPara[62]/ 1000.0f;
		mcApp_VoltageP_PIParam.qKc	= (float)MCPara[63]/ 1000.0f;
		}
	}
	mcApp_VoltageP_PIParam.qOutMax = (float)MCPara[34];
	mcApp_VoltageP_PIParam.qOutMin = (float)MCPara[35];
	mcApp_VoltageP_PIParam.qdSumMax = (float)MCPara[34];
	mcApp_VoltageP_PIParam.qdSumMin = -mcApp_VoltageP_PIParam.qdSumMax;
	
	
	if(App.Logic.LogicHall_or_pll == 1){
		mcApp_VoltageP_PIParam.qOutMin = (float)MCPara[27];
		mcApp_VoltageP_PIParam.qOutMax = (float)MCPara[28];
	}
		
	
	mcApp_VoltageP_PIParam.qdSum=0;
	mcApp_VoltageP_PIParam.qOut=0;	
	mcLib_InitPI(&mcApp_VoltageP_PIParam);
}

//通道2电压环初始化
void VoltageP2_Init(void){
	
	if(App.Logic.CtlMode == 3){		//往复运动的PID参数
	mcApp_VoltageP_PIParam.qKp  = (float)MCPara2[46]/ 1000.0f;
	mcApp_VoltageP_PIParam.qKi  = (float)MCPara2[47]/ 1000.0f;
	mcApp_VoltageP_PIParam.qKc	= (float)MCPara2[48]/ 1000.0f;			
	}else{
		if(App.Logic.LogicHall_or_pll == 2){	//有Hall单向运动
			mcApp_VoltageP_PIParam2.qKp  = (float)MCPara2[31]/ 1000.0f;
			mcApp_VoltageP_PIParam2.qKi  = (float)MCPara2[32]/ 1000.0f;
			mcApp_VoltageP_PIParam2.qKc	=  (float)MCPara2[33]/ 1000.0f;		
		}else{
			mcApp_VoltageP_PIParam2.qKp  = (float)MCPara2[61]/ 1000.0f;
			mcApp_VoltageP_PIParam2.qKi  = (float)MCPara2[62]/ 1000.0f;
			mcApp_VoltageP_PIParam2.qKc	=  (float)MCPara2[63]/ 1000.0f;		
		}
	}

	mcApp_VoltageP_PIParam2.qOutMax = (float)MCPara2[34];
	mcApp_VoltageP_PIParam2.qOutMin = (float)MCPara2[35];
	mcApp_VoltageP_PIParam2.qdSumMax = (float)MCPara2[34];
	mcApp_VoltageP_PIParam2.qdSumMin = -mcApp_VoltageP_PIParam2.qdSumMax;
	
	if(App.Logic.LogicHall_or_pll == 1){
		mcApp_VoltageP_PIParam.qOutMin = (float)MCPara2[27];
		mcApp_VoltageP_PIParam.qOutMax = (float)MCPara2[28];
	}
		
	
	mcApp_VoltageP_PIParam2.qdSum=0;
	mcApp_VoltageP_PIParam2.qOut=0;	
	mcLib_InitPI(&mcApp_VoltageP_PIParam2);
}

