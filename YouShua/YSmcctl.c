#include "YSmcctl.h"
#include "mcctl.h"
#include "mclib.h"
#include "youshua.h"
#include "userpara.h"
#include "adc.h"

//有刷速度环、电压环
mcParam_PIController2 MC_Spd;
mcParam_PIController2 MC_VBus;

//有刷通道1参数
uint16_t YSPara1[ParaNum] = {
	1,				//- 0		地址
	
	7000,			//- 1   最大转速设定*10							：系统参数以01为准
	30,				//- 2   最小转速设定*10							：系统参数以01为准
	100,			//- 3   允许启动电压/10V							：系统参数以01为准
	1001,			//- 4   有刷电机控制最大占空比	0-1001：系统参数以01为准	
	
	120,			//- 5   欠压保护值	/10V							：系统参数以01为准
	1000,			//- 6   欠压保护时间	ms							：系统参数以01为准
	360,			//- 7   过压保护值	/10V							：系统参数以01为准
	200,			//- 8   过压保护时间	ms							：系统参数以01为准
	70,				//- 9   过温保护值	度								：系统参数以01为准	
	500,			//- 10  过温保护时间	ms							：系统参数以01为准		 
	0,				//- 11  掉线保护时间 ms							：系统参数以01为准					
	
	260,			//- 12  同步整流闭环电压/10V - 最大闭环电压 			：系统参数以01为准
	950,			//- 13  同步整流输出最大占空比										：系统参数以01为准
	
	1500,			//- 14  通道1硬件保护值 
	
	2400,			//- 15  通道1刀头1电阻值 						mR					//2400
	33,		  	//- 16  通道1刀头1电感 							uH				//233
	1650,			//- 17  通道1刀头1速度转换系数									//1730
	280,				//- 18  通道1刀头1电阻与电流系数/ 100						//50
												
	110,			//- 19  通道1刀头1单向过流保护值					/100A		
	200,			//- 20  通道1刀头1单向过流保护时间阈值		ms
												
	220,			//- 21  通道1刀头1往复过流保护值		/100A
	300,			//- 22  通道1刀头1往复过流保护时间阈值		ms
												
	300,			//- 23  通道1刀头1单向堵转保护值 rpm 
	200,			//- 24	通道1刀头1单向堵转保护时间阈值		ms
												
	300,			//- 25	通道1刀头1往复堵转保护值 rpm
	200,			//- 26	通道1刀头1往复堵转保护时间阈值		ms
											
	2400,			//- 27	通道1刀头2电阻值 						mR					//2900
	33,			//- 28	通道1刀头2电感 							uH				//244
	1650,			//- 29	通道1刀头2速度转换系数									//450
	280,				//- 30	通道1刀头2电阻与电流系数 		/ 100				//50
													
	110,			//- 31	通道1刀头2单向过流保护值					/100A
	200,			//- 32	通道1刀头2单向过流保护时间阈值		ms
	      								
	220,			//- 33	通道1刀头2往复过流保护值		/100A
	300,			//- 34	通道1刀头2往复过流保护时间阈值		ms
	      								
	300,			//- 35	通道1刀头2单向堵转保护值 rpm 
	200,			//- 36	通道1刀头2单向堵转保护时间阈值		ms
	      								 
	300,			//- 37	通道1刀头2往复堵转保护值 rpm
	200,			//- 38	通道1刀头2往复堵转保护时间阈值		ms
	
	100,			//- 39	缺相保护值，刀头1与刀头2一样
	30,				//- 40  通道1缺相保护-设定转速
	30,				//- 41  通道1缺相保护-当前转速
	200,			//- 42	通道1缺陷保护时间
	
	
	1000,			//- 43	转速环Kp
	100,			//- 44	转速环Ki
	0,				//- 45	转速环Kd
	10,				//- 46	转速环目标速度闭环爬升		-100
	10,				//- 47	转速环目标速度闭环下降		-100 
		
	300,			//- 48	电压环Kp
	1,				//- 49	电压环Ki
	0,				//- 50	电压环Kd
	50,				//- 51	电压环PWM闭环攀升			-50
	50,				//- 52	电压环PWM闭环下降			-50
	
	7,					//- 53	往复运动堵转保护 切换转向速度小于保护值的时间计数衰减值		越小判定保护越敏感
	0,					//- 54	NC
	1,					//- 55  启动运行之后Nms才开始检测速度和判断堵转		ms

	50,					//- 56	该通道停止指令时刹车PWM下降速度
	100,				//- 57	该通道往复运动时刹车PWM下降速度
	100,				//- 58	该通道切换方向时刹车PWM下降速度
	
	300,			//- 59	停止指令的刹车时间
	300 ,    	//- 60  切换转向的刹车时间
	10 ,    	//- 61  往复运动的刹车时间
	
};



//有刷通道2参数
uint16_t YSPara2[ParaNum] = {

	2,				//- 0		地址
	
	7000,			//- 1   最大转速设定*10							：系统参数以01为准
	30,				//- 2   最小转速设定*10							：系统参数以01为准
	100,			//- 3   允许启动电压/10V						：系统参数以01为准
	1001,			//- 4   有刷电机控制最大占空比	0-1001：系统参数以01为准	
	
	120,			//- 5   欠压保护值	/10V						：系统参数以01为准
	1000,			//- 6   欠压保护时间	ms						：系统参数以01为准
	360,			//- 7   过压保护值	/10V						：系统参数以01为准
	200,			//- 8   过压保护时间	ms						：系统参数以01为准
	70,				//- 9   过温保护值	度							：系统参数以01为准	
	500,			//- 10  过温保护时间	ms						：系统参数以01为准		 
	0,				//- 11  掉线保护时间 ms						：系统参数以01为准					
	
	260,			//- 12  同步整流闭环电压/10V - 最大闭环电压 			：系统参数以01为准
	950,			//- 13  同步整流输出最大占空比										：系统参数以01为准
	
	1500,			//- 14  通道2硬件保护值 
	
	2400,			//- 15  通道2刀头1电阻值 						mR
	33,			//- 16  通道2刀头1电感 						uH
	1650,			//- 17  通道2刀头1速度转换系数
	280,				//- 18  通道2刀头1电阻与电流系数 		/ 100
									
	110,			//- 19  通道2刀头1单向过流保护值					/100A		
	200,			//- 20  通道2刀头1单向过流保护时间阈值		ms
				           
	220,			//- 21  通道2刀头1往复过流保护值		/100A
	300,			//- 22  通道2刀头1往复过流保护时间阈值		ms
				           
	300,			//- 23  通道2刀头1单向堵转保护值 rpm 
	200,			//- 24	通道2刀头1单向堵转保护时间阈值		ms
				           
	300,			//- 25	通道2刀头1往复堵转保护值 rpm
	200,			//- 26	通道2刀头1往复堵转保护时间阈值		ms
				
	2400,			//- 27	通道2刀头2电阻值 						mR
	33,			//- 28	通道2刀头2电感 							uH
	1650,			//- 29	通道2刀头2速度转换系数
	280,				//- 30	通道2刀头2电阻与电流系数 		/ 100
														
	110,			//- 31	通道2刀头2单向过流保护值					/100A
	200,			//- 32	通道2刀头2单向过流保护时间阈值		ms
	      								
	220,			//- 33	通道2刀头2往复过流保护值		/100A
	300,			//- 34	通道2刀头2往复过流保护时间阈值		ms
	      										
	300,			//- 35	通道2刀头2单向堵转保护值 rpm 
	200,			//- 36	通道2刀头2单向堵转保护时间阈值		ms
	      										 
	300,			//- 37	通道2刀头2往复堵转保护值 rpm
	200,			//- 38	通道2刀头2往复堵转保护时间阈值		ms
	
	100,			//- 39	缺相保护值，刀头1与刀头2一样
	30,				//- 40  通道2缺相保护-设定转速
	30,				//- 41  通道2缺相保护-当前转速
	200,			//- 42	通道2缺陷保护时间

	1000,			//- 43	转速环Kp
	100,			//- 44	转速环Ki
	0,				//- 45	转速环Kd
	10,				//- 46	转速环目标速度闭环爬升
	10,				//- 47	转速环目标速度闭环下降
	
	300,			//- 48	电压环Kp
	1,				//- 49	电压环Ki
	0,				//- 50	电压环Kd
	50,				//- 51	电压环PWM闭环攀升
	50,				//- 52	电压环PWM闭环下降
	
	7,				//- 53	往复运动堵转保护 切换转向速度小于保护值的时间计数衰减值		越小判定保护越敏感
	0,				//- 54	
	
	1,				//- 55  启动运行之后Nms才开始检测速度	ms	
	
	50,				//- 56	该通道停止指令时刹车PWM下降速度
	100,			//- 57	该通道往复运动时刹车PWM下降速度
	100,			//- 58	该通道切换方向时刹车PWM下降速度
	
	300,			//- 59	停止指令的刹车时间
	300 ,     //- 60  切换转向的刹车时间
	10 ,      //- 61  往复运动的刹车时间
	
};



//有刷1ms控制函数
void YS1msCtl(void){
		//有刷ADC采样触发
		ADC_SoftwareStart;
	
		App2.Ch1.Tim.StopTimFg = 1;		//刹车计时
		App2.Log.Mode31msFg = 1;			//正反转计时
		App2.Ch2.Tim.StopTimFg = 1;		//通道2刹车计时
		App2.SysCtl.CheckCyc = 1;			//保护检测时间计时
		//有刷通道1启动计时
		if(App2.Ch1.AllowRun == 1){				
			if(App2.Log.RunTimCnt1 < 30000){
				App2.Log.RunTimCnt1 ++;
			}
		}else {
			App2.Log.RunTimCnt1 = 0;
		}
		//有刷通道2启动计时		
		if(App2.Ch2.AllowRun == 1){
			if(App2.Log.RunTimCnt2 < 30000){
				App2.Log.RunTimCnt2 ++;
			}
		}else {
			App2.Log.RunTimCnt2 = 0;
		}
		
	if(App2.Ch1.AllowRun == 1){	
		//通道1当前在刹车状态-开启刹车电阻
		if(App2.Log.BreakSta == 1){		//通过BUCK电路去刹车
			App2.Log.VbusSetPWM = 0;		//通过爬坡去进行刹车
			if(App2.Ch1.Start == 0){		//根据不同的情况去进行不同爬坡的刹车
				App2.Log.VbusNowPWM -=YSPara1[56];			 //停止
			}else if(App2.Log.CtlMode == 3 && App2.Ch1.Dir.Now != App2.Ch1.Dir.Set && App2.Ch1.Start == 1){		//往复
				App2.Log.VbusNowPWM -=YSPara1[57];
			}else if(App2.Log.CtlMode != 3 && App2.Ch1.Dir.Now != App2.Ch1.Dir.Set && App2.Ch1.Start == 1){		//换向
				App2.Log.VbusNowPWM -=YSPara1[58];
			}else {
				App2.Log.VbusNowPWM -=YSPara1[56];				//停止									
			}
			
			if(App2.Log.VbusNowPWM  <0 ){
				App2.Log.VbusNowPWM  = 0;
			}
			TMR_SetCompare4(TMR2,App2.Log.VbusNowPWM);	//最后赋值PWM
		}
	}
	
	if(App2.Ch2.AllowRun == 1){
		//通道2当前在刹车状态-开启刹车电阻
		if(App2.Log.BreakSta == 1){		//通过BUCK电路去刹车
			App2.Log.VbusSetPWM = 0;		//通过爬坡去进行刹车
			if(App2.Ch2.Start == 0){    //根据不同的情况去进行不同爬坡的刹车
				App2.Log.VbusNowPWM -=YSPara2[56];			 //停止
			}else if(App2.Log.CtlMode == 3 && App2.Ch2.Dir.Now != App2.Ch1.Dir.Set && App2.Ch2.Start == 1){		//往复
				App2.Log.VbusNowPWM -=YSPara2[57];
			}else if(App2.Log.CtlMode != 3 && App2.Ch2.Dir.Now != App2.Ch1.Dir.Set && App2.Ch2.Start == 1){		//换向
				App2.Log.VbusNowPWM -=YSPara2[58];
			}else{																	   //停止
				App2.Log.VbusNowPWM -=YSPara2[56];			
			}
			if(App2.Log.VbusNowPWM  <0 ){
				App2.Log.VbusNowPWM  = 0;
			}
			TMR_SetCompare4(TMR2,App2.Log.VbusNowPWM);//最后赋值PWM
		}
	}	
		
}


//有刷500ms控制函数
void YS500msCtl(void){
	
}

//有刷PI初始化函数
void YSPI_Init(void){
	
	if(App2.Log.Channel == 3){								//通道1 PID参数
		//速度环
		MC_Spd.qKp = YSPara1[43] / 10000.0f;	
		MC_Spd.qKi = YSPara1[44] / 1000000.0f;	
		MC_Spd.qKc = YSPara1[45] / 10000.0f;	
		MC_Spd.qOutMax = YSPara1[12] * 100.0f / 10.0f;//输出电压参考值，并且放大100倍，这样PI好计算一些。	
		MC_Spd.qOutMin = 0;	
		MC_Spd.qErrMax = YSPara1[1]*10 / 2.0f;	
		MC_Spd.qErrMin = -MC_Spd.qErrMax;
		MC_Spd.qdSumMax = MC_Spd.qOutMax * 1.2f;
		MC_Spd.qdSumMin = MC_Spd.qOutMax - MC_Spd.qdSumMax;
		
		//电压环
		MC_VBus.qKp = YSPara1[48];		
		MC_VBus.qKi = YSPara1[49];		
		MC_VBus.qKc = YSPara1[50];	
		MC_VBus.qOutMax = PWM_PERIOD_COUNT * YSPara1[13] / 1000.0f;
		MC_VBus.qOutMin = 0;
		MC_VBus.qErrMax = YSPara1[12] / 10.0f / 2.0f;
		MC_VBus.qErrMin = -MC_VBus.qErrMax;
		MC_VBus.qdSumMax = MC_VBus.qOutMax * 1.2f;
		MC_VBus.qdSumMin = MC_VBus.qOutMax - MC_VBus.qdSumMax;
		mcLib_InitPI2(&MC_VBus);	

	}else if(App2.Log.Channel == 4){												//通道2 PID参数
		//速度环
		MC_Spd.qKp = YSPara2[43] / 10000.0f;
		MC_Spd.qKi = YSPara2[44] / 1000000.0f;
		MC_Spd.qKc = YSPara2[45] / 10000.0f;
		MC_Spd.qOutMax = YSPara2[12] * 100.0f / 10.0f;//输出电压参考值，并且放大100倍，这样PI好计算一些。
		MC_Spd.qOutMin = 0;
		MC_Spd.qErrMax = YSPara2[1]*10 / 2.0f;
		MC_Spd.qErrMin = -MC_Spd.qErrMax;
		MC_Spd.qdSumMax = MC_Spd.qOutMax * 1.2f;
		MC_Spd.qdSumMin = MC_Spd.qOutMax - MC_Spd.qdSumMax;
		mcLib_InitPI2(&MC_Spd);
		
		//电压环
		MC_VBus.qKp = YSPara2[48];
		MC_VBus.qKi = YSPara2[49];
		MC_VBus.qKc = YSPara2[50];
		MC_VBus.qOutMax = PWM_PERIOD_COUNT * YSPara2[13] / 1000.0f;
		MC_VBus.qOutMin = 0;
		MC_VBus.qErrMax = YSPara2[12] / 10.0f / 2.0f;
		MC_VBus.qErrMin = -MC_VBus.qErrMax;
		MC_VBus.qdSumMax = MC_VBus.qOutMax * 1.2f;
		MC_VBus.qdSumMin = MC_VBus.qOutMax - MC_VBus.qdSumMax;
		mcLib_InitPI2(&MC_VBus);		
	}
}


//速度环计算
void YS_SpdPI_Out(void){

	MC_Spd.qInMeas = App2.SysCtl.SpdNowLPF;		//实际估算转速
	MC_Spd.qInRef  = App2.SysCtl.SpdSetSet;		//目标转速
	mcLib_CalcPI2(&MC_Spd);
	App2.Log.VbusRef = MC_Spd.qOut / 100.0f;	//PI输出
	
}
//电压环计算
void YS_VBus_PI_Out(void){
	
	MC_VBus.qInMeas = App2.Ch1.ADC.Vbus;			//实际
	MC_VBus.qInRef  = App2.Log.VbusRef; 		//参考		//MCPara[30] / 10.0f; 
	mcLib_CalcPI2(&MC_VBus);
	
}

//初始化PID输出函数
void mcLib_InitPI2( mcParam_PIController2 *pParam){
  pParam->qdSum = 0;
  pParam->qOut = 0;
	pParam->qErr = 0;
	pParam->qInMeas = 0;
	pParam->qInRef = 0;
	
}

//PI运算函数
void mcLib_CalcPI2( mcParam_PIController2 *pParam){
	float U;
	float Exc;
	
	pParam->qErr  = pParam->qInRef - pParam->qInMeas; 
	
	if(pParam->qErr > pParam->qErrMax){
		pParam->qErr = pParam->qErrMax;
	}else if(pParam->qErr < pParam->qErrMin){
		pParam->qErr = pParam->qErrMin;
	}
	
	U  = pParam->qdSum + pParam->qKp * pParam->qErr;
	
	if( U > pParam->qOutMax ){
			pParam->qOut = pParam->qOutMax;
	}else if( U < pParam->qOutMin ){
			pParam->qOut = pParam->qOutMin;
	}else{
			pParam->qOut = U;  
	}
	
	Exc = U - pParam->qOut;
	pParam->qdSum = pParam->qdSum + pParam->qKi * pParam->qErr - pParam->qKc * Exc;
	
	if(pParam->qdSum > pParam->qdSumMax){
		pParam->qdSum = pParam->qdSumMax;
	}else if(pParam->qdSum < pParam->qdSumMin){
		pParam->qdSum = pParam->qdSumMin;
	}
}



