#include "protect.h"
#include "logic.h"
#include "youshua.h"
#include "adc.h"
#include "mcuart.h"
/*
 * 函数功能：校验参数22/24的毫秒值，非法值统一回退500ms。
 * 输入参数：value为尚未截断的原始时间。
 * 返回参数：合法原值或500ms。
 */
uint16_t Protect_ValidateTime(uint32_t value)
{
    return ((value >= MOTOR_OVERCURRENT_TIME_MIN_MS) && (value <= MOTOR_OVERCURRENT_TIME_MAX_MS))
        ? (uint16_t)value : MOTOR_OVERCURRENT_TIME_DEFAULT_MS; /* 不把超大值钳位到1500ms，避免延长保护。 */
}

/*
 * 函数功能：在控制命令临界区内装载两项独立时间，运行中只允许缩短，重复帧不重置计数。
 * 输入参数：motor_kind为1/2无刷或3/4有刷物理通道；time22/time24为毫秒；stopped为全部输出已静止。
 * 返回参数：无；配置仅存在保护RAM缓存，不能写入参数Flash。
 */
void Protect_ApplyCommandTimes(uint8_t motor_kind, uint16_t time22, uint16_t time24, uint8_t stopped)
{
    time22 = Protect_ValidateTime(time22); /* 接收入口再次校验，非法往复时间不能进入运行态。 */
    time24 = Protect_ValidateTime(time24); /* 无霍尔独立回退，另一项合法配置不受影响。 */
    if ((motor_kind == 1U) || (motor_kind == 3U))
    {
        if ((stopped != 0U) || ((float)time22 < App.FB.Prot.OverCurrentCnt2))
            App.FB.Prot.OverCurrentCnt2 = time22; /* 静止时可装载新手柄；运行中不得增加当前过流窗口。 */
        if ((stopped != 0U) || ((float)time24 < App.FB.Prot.OverCurrentCnt3))
            App.FB.Prot.OverCurrentCnt3 = time24; /* 只更新阈值，OverCurrentCnt继续累计。 */
    }
    else if ((motor_kind == 2U) || (motor_kind == 4U))
    {
        if ((stopped != 0U) || ((float)time22 < App.FB2.Prot.OverCurrentCnt2))
            App.FB2.Prot.OverCurrentCnt2 = time22; /* B物理通道不修改A通道保护时间。 */
        if ((stopped != 0U) || ((float)time24 < App.FB2.Prot.OverCurrentCnt3))
            App.FB2.Prot.OverCurrentCnt3 = time24; /* 保留B通道已经累计的过流计数。 */
    }
}

/*
 * 函数功能：按Flash/RAM参数初始化保护缓存，并清理参数22/24的非法历史值。
 * 输入参数：无；仅在上电或完全停止的维护流程调用。
 * 返回参数：无。
 */
void Protect_Init(void){
    MCPara[22] = Protect_ValidateTime(MCPara[22]); /* A往复时间越界在RAM中回退，不自动擦写Flash。 */
    MCPara[24] = Protect_ValidateTime(MCPara[24]); /* A无霍尔时间独立检查。 */
    MCPara2[22] = Protect_ValidateTime(MCPara2[22]); /* B往复时间不继承A配置。 */
    MCPara2[24] = Protect_ValidateTime(MCPara2[24]); /* B无霍尔时间独立回退。 */
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
	App.Logic.HandShakeProtTim = MOTOR_HARD_COMMAND_TIMEOUT_MS; /* 控制租约固定200ms，参数区不能关闭或延长硬停机。 */
	
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




/*
 * 函数功能：执行无刷和有刷共用的1ms系统保护，并在手柄控制租约超时时立即关闭全部功率输出。
 * 输入参数：无。
 * 返回参数：无。
 */
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
			if(App.Logic.HandShakeProtCnt >= App.Logic.HandShakeProtTim){
				App.Logic.HandShakeProtCnt = App.Logic.HandShakeProtTim; /* 超时后计数饱和，避免长时间失联发生32位回绕。 */
				MotorCommand_ForceCommunicationStop(); /* 200ms未收到合法控制帧时直接关无刷、有刷PWM并进入通信故障。 */
			}else{
				App.Logic.HandShakeProtCnt++; /* 只有mcuart收到严格CRC且字段有效的控制帧才会清零本计数。 */
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

