#include "protect.h"
#include "logic.h"
#include "youshua.h"
#include "adc.h"
void Protect_Init(void){
	App.FB.Prot.CurrentLPFXiSHU = (float)MCPara[9]/100.0f;
	
	App.FB.Prot.UnderVoltage = (float)MCPara[2] / 10.0f;			//系统欠压保护值
	App.FB.Prot.OverVoltage = (float)MCPara[4] / 10.0f;				//系统过压保护值
	App.FB.Prot.OverTemper	= (float)MCPara[6];								//系统驱动器过温保护值
	//通道1保护参数
	App.FB.Prot.OverCurrent1	=	(float)MCPara[19] / 100.0f;		//通道1有Hall单向运动过流保护值
	App.FB.Prot.OverCurrentCnt1 = MCPara[20];
	App.FB.Prot.OverCurrent2	= (float)MCPara[21] / 100.0f;		//通道1有Hall往复运动过流保护值
	App.FB.Prot.OverCurrentCnt2 = MCPara[22];
	App.FB.Prot.OverCurrent3	= (float)MCPara[23] / 100.0f;		//通道1无Hall单向运动过流保护值
	App.FB.Prot.OverCurrentCnt3 = MCPara[24];
	App.FB.Prot.OverCurrent4  = (float)MCPara[17] / 100.0f;		//通道1长时间大电流过流保护值
	App.FB.Prot.OverCurrentCnt4 = MCPara[18];
	
	//通道2保护参数
	App.FB2.Prot.OverCurrent1	=	(float)MCPara2[19] / 100.0f;		//通道1有Hall单向运动过流保护值
	App.FB2.Prot.OverCurrentCnt1 = MCPara2[20];
	App.FB2.Prot.OverCurrent2	= (float)MCPara2[21] / 100.0f;		//通道1有Hall往复运动过流保护值
	App.FB2.Prot.OverCurrentCnt2 = MCPara2[22];
	App.FB2.Prot.OverCurrent3	= (float)MCPara2[23] / 100.0f;		//通道1无Hall单向运动过流保护值
	App.FB2.Prot.OverCurrentCnt3 = MCPara2[24];
	App.FB2.Prot.OverCurrent4  = (float)MCPara2[17] / 100.0f;		//通道1长时间大电流过流保护值
	App.FB2.Prot.OverCurrentCnt4 = MCPara2[18];	
	
	//通讯保护时间
	App.Logic.HandShakeProtTim = MCPara[10]*100;	//转化成毫秒
	
}



//系统保护
void Motor_Protect(void){
	
#if _protect == 1	
	if(App.FB.CheckCyc == 1){ 
		App.FB.CheckCyc = 0;
		
		if((App.FB.AllRun == 1)&&(App.Logic.CtlMode != 4)&&(App.Logic.CtlMode != 5)&&(App.FB.Status != HS_POSBRA)){					//通道1模式1/2/3的保护都在这里，4/5需要分开
			//堵转保护1 电机1 		正转反转的时候的堵转保护
			if(myabs(App.FB.Spd_Now) < MCPara[14] && App.Logic.CtlMode !=3){
				App.FB.Prot.RunStallCnt1 ++;
				if(App.FB.Prot.RunStallCnt1 > MCPara[15]){
					App.FB.Err = E_RUNSTALL;			
					App.FB.Prot.RunStallCnt1 = 0;
				}
			}else if(App.FB.Prot.RunStallCnt1 > 0){
				App.FB.Prot.RunStallCnt1 -- ;
			}
			
			if(App.Logic.LogicHall_or_pll == 2){		//有Hall保护
			
				if(App.Logic.CtlMode != 3){						//有Hall单向保护
					if( App.FB.Prot.AllCur > App.FB.Prot.OverCurrent){
						App.FB.Prot.OverCurrentCnt ++;
						if(App.FB.Prot.OverCurrentCnt >App.FB.Prot.OverCurrentCnt1){		
								App.FB.Prot.OverCurrentCnt = 0;
								App.FB.Err = E_OC1;
							}							
						}else if (App.FB.Prot.OverCurrentCnt > 0){
								App.FB.Prot.OverCurrentCnt --;
					}
				}else if (App.Logic.CtlMode == 3){		//有Hall往复保护
					if(App.FB.Prot.AllCur > App.FB.Prot.OverCurrent ){
						App.FB.Prot.OverCurrentCnt ++;
						if(App.FB.Prot.OverCurrentCnt >App.FB.Prot.OverCurrentCnt2){		
								App.FB.Prot.OverCurrentCnt = 0;
								App.FB.Err = E_OC1;
							}							
						}else if (App.FB.Prot.OverCurrentCnt > 0){
								App.FB.Prot.OverCurrentCnt --;
					}					
				}
			}else if(App.Logic.LogicHall_or_pll == 1){
					if( App.FB.Prot.AllCur > App.FB.Prot.OverCurrent){
						App.FB.Prot.OverCurrentCnt ++;
						if(App.FB.Prot.OverCurrentCnt >App.FB.Prot.OverCurrentCnt3){		
								App.FB.Prot.OverCurrentCnt = 0;
								App.FB.Err = E_OC1;
							}							
						}else if (App.FB.Prot.OverCurrentCnt > 0){
								App.FB.Prot.OverCurrentCnt --;
					}								
			}else{
				
			}
			
		//通道1长时间大电流保护
		if(App.FB.Prot.AllCur> App.FB.Prot.OverCurrent4 ){
			App.FB.Prot.LongOverCurrentCnt ++;
			if(App.FB.Prot.LongOverCurrentCnt > App.FB.Prot.OverCurrentCnt4){
				App.FB.Prot.LongOverCurrentCnt= 0;
				App.FB.Err = E_RUNSTALL;	
			}	
		}else if(App.FB.Prot.LongOverCurrentCnt > 0){
			App.FB.Prot.LongOverCurrentCnt --;
		}			
			
		}else {
			App.FB.Prot.RunStallCnt1 = 0;
			App.FB.Prot.LongOverCurrentCnt = 0;
			App.FB.Prot.OverCurrentCnt = 0;
		}
		//往复运动的堵转保护
			if((myabs(App.FB.Spd_Now) < MCPara[14] && App.Logic.CtlMode == 3 
				&& App.FB.Spd_Set >= (MCPara[12]*10) && App.Logic.Motor_Kind ==1)
				&&(App.FB.AllRun ==1)&&(App.FB.Status != HS_POSBRA)){
				App.FB.Prot.RunStallCnt2 ++;
				if(App.FB.Prot.RunStallCnt2 > MCPara[15]){
					App.FB.Err = E_RUNSTALL;
					App.FB.Prot.RunStallCnt2 = 0;
				}
			}else if(App.FB.Prot.RunStallCnt2 > 0){
				App.FB.Prot.RunStallCnt2 -=MCPara[16] ;
				if(App.FB.Prot.RunStallCnt2<0){
					App.FB.Prot.RunStallCnt2 = 0;
				}
			}	 

		if((App.FB2.AllRun == 1)&&(App.Logic.CtlMode != 4)&&(App.Logic.CtlMode != 5)){					//通道1模式1/2/3的保护都在这里，4/5需要分开
			//电机2堵转
			if(myabs(App.FB2.Spd_Now) < MCPara2[14] && App.Logic.CtlMode !=3){
				App.FB2.Prot.RunStallCnt1 ++;
				if(App.FB2.Prot.RunStallCnt1 > MCPara2[15]){
					App.FB2.Err = E_RUNSTALL;
					App.FB2.Prot.RunStallCnt1 = 0;
				}
			}else if(App.FB2.Prot.RunStallCnt1 > 0){
				App.FB2.Prot.RunStallCnt1 -- ;
			}		
			
			//通道2的过流保护
			if(App.Logic.LogicHall_or_pll == 2){		//有Hall保护
			
				if(App.Logic.CtlMode != 3){						//有Hall单向保护
					if( App.FB2.Prot.AllCur > App.FB2.Prot.OverCurrent){
						App.FB2.Prot.OverCurrentCnt ++;
						if(App.FB2.Prot.OverCurrentCnt >App.FB2.Prot.OverCurrentCnt1){		
								App.FB2.Prot.OverCurrentCnt = 0;
								App.FB2.Err = E_OC1;
							}							
						}else if (App.FB2.Prot.OverCurrentCnt > 0){
								App.FB2.Prot.OverCurrentCnt --;
					}
				}else if (App.Logic.CtlMode == 3){		//有Hall往复保护
					if(App.FB2.Prot.AllCur > App.FB2.Prot.OverCurrent ){
						App.FB2.Prot.OverCurrentCnt ++;
						if(App.FB2.Prot.OverCurrentCnt >App.FB2.Prot.OverCurrentCnt2){		
								App.FB2.Prot.OverCurrentCnt = 0;
								App.FB2.Err = E_OC1;
							}							
						}else if (App.FB2.Prot.OverCurrentCnt > 0){
								App.FB2.Prot.OverCurrentCnt --;
					}					
				}
			}else if(App.Logic.LogicHall_or_pll == 1){
					if( App.FB2.Prot.AllCur > App.FB2.Prot.OverCurrent){
						App.FB2.Prot.OverCurrentCnt ++;
						if(App.FB2.Prot.OverCurrentCnt >App.FB2.Prot.OverCurrentCnt3){		
								App.FB2.Prot.OverCurrentCnt = 0;
								App.FB2.Err = E_OC1;
							}							
						}else if (App.FB2.Prot.OverCurrentCnt > 0){
								App.FB2.Prot.OverCurrentCnt --;
					}								
			}else{
				
			}

			//通道2长时间大电流保护
			if(App.FB2.Prot.AllCur> App.FB2.Prot.OverCurrent4 ){
				App.FB2.Prot.LongOverCurrentCnt ++;
				if(App.FB2.Prot.LongOverCurrentCnt > App.FB2.Prot.OverCurrentCnt4 ){
					App.FB2.Prot.LongOverCurrentCnt= 0;
					App.FB2.Err = E_RUNSTALL;	
				}	
			}else if(App.FB2.Prot.LongOverCurrentCnt> 0){
				App.FB2.Prot.LongOverCurrentCnt --;
			}			
			
		}else {
			App.FB2.Prot.LongOverCurrentCnt = 0;
			App.FB2.Prot.OverCurrentCnt = 0;
			App.FB2.Prot.RunStallCnt1 = 0;
	}

		//电机2往复堵转保护
			if((myabs(App.FB2.Spd_Now) < MCPara2[14] && App.Logic.CtlMode == 3 			
				&& App.FB2.Spd_Set >= (MCPara2[12]*10) && App.Logic.Motor_Kind ==2)
				&&(App.FB2.AllRun ==1)){
				App.FB2.Prot.RunStallCnt2 ++;
				if(App.FB2.Prot.RunStallCnt2 > MCPara2[15]){
					App.FB2.Err = E_RUNSTALL;
					App.FB2.Prot.RunStallCnt2= 0;
				}
			}else if(App.FB2.Prot.RunStallCnt2 > 0){
				App.FB2.Prot.RunStallCnt2 -=MCPara2[16] ;
				if(App.FB2.Prot.RunStallCnt2<0){
					App.FB2.Prot.RunStallCnt2 = 0;
				}
			}	 		
	}
	#endif
	
	if(App.FB.Err !=App.FB2.Err){
		if(App.FB.Err != E_NONE){
			STA_Led(App.FB.Err);
		}else{
			STA_Led(App.FB2.Err);
		}

	}else{
		STA_Led(App.FB.Err);
	}
}




//系统保护函数
void SystemProtect(void){

	if((App2.SysCtl.CalAdcFg == 1)&&(App.FB.Err == E_NONE && App.FB2.Err ==  E_NONE)&&(App2.Err == E_NONE))  {
		App2.SysCtl.CalAdcFg = 0;
		//总电压
		App.FB.Prot.BusVol = (App.FB.Prot.BusVol + (float)get_vbus_adc_value * VOLTAGE_ADC_TO_PHY_RATIO) / 2.0f;		//均值滤波		
		//温度
		App.FB.Prot.Temper = App.FB.Prot.Temper * 0.9f + 0.1f * temp_data(ADC1Value[0],10000);						//

	}
	if(App2.SysPort.SystemProtectFlag == 1){
			App2.SysPort.SystemProtectFlag = 0;
			//通讯保护
			if(App.Logic.HandShakeProtCnt >App.Logic.HandShakeProtTim){
				App2.Err = E_HandShake;
				App.FB.Err 	=	E_HandShake;
				App.FB2.Err =	E_HandShake;
			}else{
				App.Logic.HandShakeProtCnt++;
			}
		
			//欠压保护		
			if(App.FB.Prot.BusVol < App.FB.Prot.UnderVoltage){
				App.FB.Prot.UndVolCnt ++;
				if(App.FB.Prot.UndVolCnt > MCPara[3]){
					App.FB.Prot.UndVolCnt = 0;
					App2.Err = E_UV;
					App.FB.Err = E_UV;
					App.FB2.Err = E_UV;
				}
			}else {
				if(App.FB.Prot.UndVolCnt > 1){
					App.FB.Prot.UndVolCnt --;
				}
			}
			
			//过压保护
			if(App.FB.Prot.BusVol > App.FB.Prot.OverVoltage){
				App.FB.Prot.OverVolCnt ++;
				if(App.FB.Prot.OverVolCnt > MCPara[5]){
					App.FB.Prot.OverVolCnt = 0;
					App2.Err = E_OV;
					App.FB.Err = E_OV;
					App.FB2.Err = E_OV;				
				}
			}else {
				if(App.FB.Prot.OverVolCnt > 1){
					App.FB.Prot.OverVolCnt --;
				}
			}
			
			//过温保护
			if(App.FB.Prot.Temper > MCPara[6]){
				App.FB.Prot.OverTempCnt1 ++;
				if(App.FB.Prot.OverTempCnt1 > MCPara[7]){
					App.FB.Prot.OverTempCnt1 = 0;
					App2.Err = E_Temp;
					App.FB.Err = E_Temp;
					App.FB2.Err = E_Temp;
				}	
			}else if(App.FB.Prot.OverTempCnt1 > 0){
				App.FB.Prot.OverTempCnt1 --;
			}
	}
}

