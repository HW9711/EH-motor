#include "logic.h"
#include "mcctl.h"
#include "tim.h"
#include "virtual_oscilloscope.h"
#include "stdio.h"
#include "pm.h"
#include "key.h"
#include "adc.h"
#include "mcflash.h"
#include "fangbo.h"
#include "at32_board.h"
#include "mcuart.h"

//Hall换向数组
uint8_t hallxuhuan[8] = {
	0,
	2,//1
	4,//2
	6,//3
	1,//4
	3,//5 
	5,//6
	0,
};

//控制状态检测，切换霍尔，切换电机
void Control_State_Scan(void){
	if(App.Logic.Brake_Kind == 2){	//2:切换电机种类刹车
		if( App.Logic.Motor_Kind_Change_Tim >= MCPara[42]){
				App.Logic.Brake_StopTimOK = 1;				//时间ok
				App.Logic.Motor_Kind_Change_Tim =0;		//清
			}
		}	
	if(App.Logic.Brake_Kind == 4){	//4:切换霍尔刹车
		if(App.Logic.Motor_Hall_Change_Tim >= MCPara[43]){
					App.Logic.Brake_StopTimOK = 1;				//时间ok
					App.Logic.Motor_Hall_Change_Tim =0;		//清		
		}
	}
	if((App.Logic.Brake_Sign ==1 )&&(App.Logic.Brake_StopTimOK == 1)&&(App.Logic.Brake_StopPwmOK ==1)){		//1:停机刹车
		//刹车完成
		App.Logic.Brake_Sign = 0;
		App.Logic.Brake_Kind = 0;
		App.Logic.Brake_StopTimOK = 0;
		App.Logic.Brake_StopPwmOK = 0;
		//开始赋值
		App.Logic.Motor_Kind = App.Logic.Change_Kind;
		App.Logic.LogicHall_or_pll = App.Logic.Change_Hall;
		App.Logic.CtlMode = App.Logic.Change_Mode;
	}else if((App.Logic.Brake_Sign == 1)&&(App.Logic.Brake_StopPwmOK ==1)&&(App.Logic.Brake_Kind ==1)){		//1:停机刹车
		//刹车完成
		App.Logic.Brake_Sign = 0;
		App.Logic.Brake_Kind = 0;
		App.Logic.Brake_StopTimOK = 0;
		App.Logic.Brake_StopPwmOK = 0;
		//无Hall不用位置刹车
		if(App.Logic.LogicHall_or_pll == 1){
			App.Pos.PosFlag = 0;
		}		
		//开始赋值
		App.Logic.Motor_Kind = App.Logic.Change_Kind;
		App.Logic.LogicHall_or_pll = App.Logic.Change_Hall;
		App.Logic.CtlMode = App.Logic.Change_Mode;		
	}else if((App.Logic.Brake_Sign == 1)&&(App.Logic.Brake_StopPwmOK ==1)&&(App.Logic.Brake_Kind ==3)){	//3:切换电机模式刹车
		//刹车完成
		App.Logic.Brake_Sign = 0;
		App.Logic.Brake_Kind = 0;
		App.Logic.Brake_StopTimOK = 0;
		App.Logic.Brake_StopPwmOK = 0;
		//开始赋值
		App.Logic.Motor_Kind = App.Logic.Change_Kind;
		App.Logic.LogicHall_or_pll = App.Logic.Change_Hall;
		App.Logic.CtlMode = App.Logic.Change_Mode;	
	}else if((App.Logic.Brake_Sign == 1)&&(App.Logic.Brake_StopPwmOK ==1)&&(App.Logic.Brake_Kind ==5)){	//5：快速正反转调用刹车
		App.Logic.Brake_Sign = 0;
		App.Logic.Brake_Kind = 0;
		App.Logic.Brake_StopTimOK = 0;
		App.Logic.Brake_StopPwmOK = 0;
		
	}
	
	
	
	
	
	//通讯保护-定时重启串口
	if(App.FB.Err == E_HandShake){
		App.Logic.UasrtErrRestCnt++;
		if(App.Logic.UasrtErrRestCnt >=1000){
			App.Logic.UasrtErrRestCnt = 0;
			//串口初始化
			UART_Init();
		}
	}else{
		App.Logic.UasrtErrRestCnt = 0;
	}
	
	
	
	
}



//方波逻辑--主要控制模式：正转、反转、往复运动 、位置控制（单步、回位）
void Logic_Fangbo(void){
	int8_t i = 0 ;
	if(App.FB.Status != HS_BRA){
		App.FB.Spd_Set = App.Logic.Set_Spd ;
		App.FB.Rad_Set = App.FB.Spd_Set *(2.0f *M_PI)/60.0f;
	}
	if(App.Logic.CtlMode == 1){		//正转模式
		App.FB.Dir.Set = CW;	//方向CW
		//顺便判定启动条件	
		if(		App.FB.Err == E_NONE 									//无错误
			&&	App.FB.Spd_Set >= (MCPara[12]*10)			//设定转速大于最小转速
			&&	App.Logic.Motor_Kind ==1							//电机种类
			&&  App.Logic.Brake_Sign ==0							//不在刹车
			&&  App.Logic.Motor_Stop_StartFlag == 0		//停机时间
			&&  YSorWSFlag ==1												//确认是无刷电机
		){
			App.FB.Start =1;
		}else{
			App.FB.Start =0;
		}
	}
	else if(App.Logic.CtlMode == 2){	//反转模式
		App.FB.Dir.Set = CCW;	//方向CCW
			//顺便判定启动条件	
		if(		App.FB.Err == E_NONE 									//无错误
			&&	App.FB.Spd_Set >= (MCPara[12]*10)			//设定转速大于最小转速
			&&	App.Logic.Motor_Kind ==1							//电机种类
			&&  App.Logic.Brake_Sign ==0							//不在刹车
			&&  App.Logic.Motor_Stop_StartFlag == 0		//停机时间
			&&  YSorWSFlag ==1												//确认是无刷电机
		){
			App.FB.Start =1;
		}else{
			App.FB.Start =0;
		}
	}
	else if(App.Logic.CtlMode == 3){	//往返模式：规划正转、刹车、反转、刹车
		if(App.Logic.u8WfFru >= 1){
			App.Logic.u16WfCyc = 10000 / App.Logic.u8WfFru;	//根据频率规划
		}else {
			App.Logic.u16WfCyc = 1000;
		}
		if(App.Logic.Mode3Cha == 0){		//CW
			App.FB.Dir.Set = CW;
			if(App.Logic.Mode31msFg == 1){
				App.Logic.Mode31msFg = 0;
				
				App.Logic.Mode3TimCnt ++;
				if(App.Logic.Mode3TimCnt >=(App.Logic.u16WfCyc-10)){	//规划刹车
						App.Logic.Brake_Sign = 1;
						App.Logic.Brake_Kind = 5;
				}
				if(App.Logic.Mode3TimCnt >= App.Logic.u16WfCyc){	//切换方向
					App.Logic.Mode3TimCnt = 0;
					App.Logic.Mode3Cha = 1;
				}
			}
		}else if(App.Logic.Mode3Cha == 1){		//CCW
			App.FB.Dir.Set = CCW;
			if(App.Logic.Mode31msFg == 1){
				App.Logic.Mode31msFg = 0;
				App.Logic.Mode3TimCnt ++;
				if(App.Logic.Mode3TimCnt >=(App.Logic.u16WfCyc-10)){	//规划刹车
						App.Logic.Brake_Sign = 1;
						App.Logic.Brake_Kind = 5;
				}
				if(App.Logic.Mode3TimCnt >= App.Logic.u16WfCyc){	//切换方向
					App.Logic.Mode3TimCnt = 0;
					App.Logic.Mode3Cha = 0;
				}
			}
		}
		//顺便判定启动条件	
		if(		App.FB.Err == E_NONE 									//无错误
			&&	App.FB.Spd_Set >= (MCPara[12]*10)			//设定转速大于最小转速
			&&	App.Logic.Motor_Kind ==1							//电机种类
			&&  App.Logic.Brake_Sign ==0							//不在刹车
			&&  App.Logic.Motor_Stop_StartFlag == 0		//停机时间
			&&  YSorWSFlag ==1												//确认是无刷电机	
		){
			App.FB.Start =1;
		}else{
			App.FB.Start =0;
		}		
	}
	//位置单步运行，CW
	else if((App.Logic.CtlMode == 4)&&(App.FB.Err == E_NONE)){
		if((App.Pos.Start == 1)&&(App.Logic.Motor_Kind ==1)){			//有步数发送过来，并且是电机1，那么就允许启动
			App.FB.Start = 1;
			App.FB.Dir.Now = CW;
		}else{
			App.FB.Start = 0;
		}
	}
	//位置单步运行，CCW
	else if((App.Logic.CtlMode == 5)&&(App.FB.Err == E_NONE)){
		if((App.Pos.Start == 1)&&(App.Logic.Motor_Kind ==1)){			//有步数发送过来，并且是电机1，那么就允许启动
			App.FB.Start = 1;
			App.FB.Dir.Now = CW;
		}else{
			App.FB.Start = 0;
		}
	}		
	else if(App.Logic.CtlMode == 0){
		App.FB.Start = 0;
	}
	
	//模式4/5的PWM控制步数控制都在这里：分3步
	if((App.Logic.CtlMode == 4||App.Logic.CtlMode == 5)&&(App.Logic.Motor_Kind ==1)){			//模式4/5的PWM控制和步数控制在这里
		if(App.Pos.LockPosStatus == 1){									//第一步：设置步数，锁定步数，判断霍尔是否正确
			TMR_SetCompare4(TMR2, MCPara[56]);
			App.Pos.PlanSteps  = App.Pos.PosSetSteps ;		//锁定步数
			if(App.Pos.PosNow1 == 0x1F){									//之前没有锁定位置
				App.Pos.PosHallVal = gethall;								//读取Hall;
				for(i=0; i<7; i++ ){
					if(App.Pos.PosHallVal == RAMP_TABLE_Hall[i]){
						App.Pos.PosHalli = i;
						if(App.Pos.PosHalli == 0){							//往前位移一个数组
							App.Pos.PosHalli = 5;
						}else{
							App.Pos.PosHalli --;
						}
					}
				}
			}else{
				App.Pos.PosHalli = App.Pos.PosNow1;		//之前锁定的位置
			}
			Hall_PhaseChange(RAMP_TABLE_Hall[App.Pos.PosHalli],7000);		//90%,定位到目前获取到的Hall的中间位置
			Delay_ms(MCPara[54]);
			if((App.Pos.PosHallVal !=0)||(App.Pos.PosHallVal !=7)){			//判定霍尔的值不为0和7
				App.Pos.LockPosStatus = 2;				//跳转第二步
			}else {
				App.FB.Err = E_NOHALL;								//无Hall错误
			}
		}else if(App.Pos.LockPosStatus ==2){	//第二步：根据锁定的步数，去拖动执行
			TMR_SetCompare4(TMR2, MCPara[56]);
			//给固定PWM给BUCK电路
			if(( App.Pos.PlanSteps > 0)&&(App.Pos.DragFlash ==1)){
				App.Pos.DragFlash = 0;
				if(App.Logic.CtlMode == 4){
					App.Pos.PosHalli++;
					if(App.Pos.PosHalli >5){
						App.Pos.PosHalli = 0;
					}
				}else if(App.Logic.CtlMode == 5){
					App.Pos.PosHalli--;
					if(App.Pos.PosHalli < 0){						//给定期望位置
						App.Pos.PosHalli = 5;
					}
				}
				App.Pos.PosHallVal = RAMP_TABLE_Hall[App.Pos.PosHalli];
				
				Hall_PhaseChange(App.Pos.PosHallVal,7000);		//90%			定位到期望的位置
				
				Delay_ms(MCPara[54]);
				App.Pos.PlanSteps -- ;
			}else if(( App.Pos.PlanSteps > 0)&&(App.Pos.DragFlash ==0)){
				//间隔
			}else if (App.Pos.PlanSteps == 0){
				App.Pos.LockPosStatus = 3;		//去定位一段时间
				App.Pos.LocatedTimeCnt = 0;		//清
			}
		}else if (App.Pos.LockPosStatus ==3){				//第三步：定位一段时间
				if(App.Pos.LocatedTimeCnt < App.Pos.LocatedTim){			//到达定位时间
					//定位
				}else {
					App.Pos.LockPosStatus = 0;	//回到等待状态
					App.Pos.LocatedTimeCnt = 0;	//清
					//关闭
					TMR_SetCompare4(TMR2, 0);
					App.Pos.Start = 0;
					//记录位置
					App.Pos.PosNow1 = App.Pos.PosHalli;
						//记录这个位置，后续每次转动都需要停止在这个位置,上电默认此刻位置：
					App.Pos.OutSidePosition1 = 0;		
				}			
		}else if (App.Pos.LockPosStatus ==0){
			//等待状态
		}
	}
	
	if(App.FB.Status == HS_POSBRA){		//如果在位置刹车
		App.FB.Prot.HallDragCnt++;
		if(App.FB.Prot.HallDragCnt > MCPara[66]){
			App.FB.Prot.HallDragCnt = 0;
			App.FB.Err = E_PosDrag;
			PWM_3l_Stop();									
			//不需要定位或者定位结束
			TMR_SetCompare4(TMR2, 0);	
			//最后强定位时间
			App.FB.Status = HS_WAIT;
		}
		
		if(App.Logic.CtlMode == 0 ||App.Logic.CtlMode == 1||App.Logic.CtlMode == 2||App.Logic.CtlMode == 3){
			if(App.Pos.PlanPosStep1 > 0){				//如果现在的位置是正一圈，那么就反方向拉回来
				App.Pos.Tempi1 --;
				if(App.Pos.Tempi1 < 0){
					App.Pos.Tempi1 = 5;
				}
				App.Pos.PlanPosStep1 --;
				Hall_PhaseChange(RAMP_TABLE_Hall[App.Pos.Tempi1],7000);		//90%,定位到目前获取到的Hall的中间位置
				Delay_ms(MCPara[51]);					//步进间隔的时间
			}
			else if(App.Pos.PlanPosStep1 < 0){		//如果现在的位置是负一圈，那么就正方向拉回来
			if(App.Pos.PlanPosStep1 < 0){	
				App.Pos.Tempi1 ++;
				if(App.Pos.Tempi1 > 5){
					App.Pos.Tempi1 = 0;
				}
				App.Pos.PlanPosStep1 ++;
				Hall_PhaseChange(RAMP_TABLE_Hall[App.Pos.Tempi1],7000);		//90%,定位到目前获取到的Hall的中间位置
				Delay_ms(MCPara[51]);				//步进间隔时间
				}
			}
			else{
				PWM_3l_Stop();									//如果刚刚好就在起点附近，或者是已经拉到起点附近，
				//不需要定位或者定位结束
				TMR_SetCompare4(TMR2, 0);	
				//最后强定位时间
				App.FB.Status = HS_WAIT;
			}
		}
		
		if(App.FB.Err != E_NONE){
				PWM_3l_Stop();									//如果刚刚好就在起点附近，或者是已经拉到起点附近，
				//不需要定位或者定位结束
				TMR_SetCompare4(TMR2, 0);	
				//最后强定位时间
				App.FB.Status = HS_ERR;
		}
	}
	
}


//方波通道2逻辑控制
void Logic_Fangbo2(void){
	int8_t i = 0 ;
	if(App.FB2.Status != HS_BRA){					
		App.FB2.Spd_Set = App.Logic.Set_Spd ;//给定转速
		App.FB2.Rad_Set = App.FB2.Spd_Set *(2.0f *M_PI)/60.0f;	//转化为角速度
	}
	
	if(App.Logic.CtlMode == 1){			//正转模式
		App.FB2.Dir.Set = CW;
		//顺便判定启动条件
		if(		App.FB2.Err == E_NONE 									//无错误
			&&	App.FB2.Spd_Set >= (MCPara2[12]*10)		//设定转速大于最小转速
			&&	App.Logic.Motor_Kind ==2							//电机种类正确
			&&  App.Logic.Brake_Sign ==0							//不在刹车	
			&&  App.Logic.Motor_Stop_StartFlag == 0		//停机时间
			&&  YSorWSFlag ==1												//确认是无刷电机	
		){
			App.FB2.Start =1;
		}else{
			App.FB2.Start =0;
		}
	}
	else if(App.Logic.CtlMode == 2){	//反转模式
		App.FB2.Dir.Set = CCW;				
		//顺便判定启动条件
		if(		App.FB2.Err == E_NONE 									//无错误
			&&	App.FB2.Spd_Set >= (MCPara2[12]*10)		//设定转速大于最小转速
			&&	App.Logic.Motor_Kind ==2							//电机种类正确
			&&  App.Logic.Brake_Sign ==0							//不在刹车
			&&  App.Logic.Motor_Stop_StartFlag == 0		//停机时间
			&&  YSorWSFlag ==1												//确认是无刷电机	
		){
			App.FB2.Start =1;
		}else{
			App.FB2.Start =0;
		}		
	}
	else if(App.Logic.CtlMode == 3){		//往返模式：规划正转、刹车、反转、刹车
		if(App.Logic.u8WfFru >= 1){
			App.Logic.u16WfCyc = 10000 / App.Logic.u8WfFru;//根据频率规划
		}else {
			App.Logic.u16WfCyc = 1000;
		}

		if(App.Logic.Mode3Cha == 0){				//CW
			App.FB2.Dir.Set = CW;
			
			if(App.Logic.Mode31msFg == 1){
				App.Logic.Mode31msFg = 0;
				
				App.Logic.Mode3TimCnt ++;
				if(App.Logic.Mode3TimCnt >=(App.Logic.u16WfCyc-10)){	//规划刹车
						App.Logic.Brake_Sign = 1;
						App.Logic.Brake_Kind = 5;
				}
				if(App.Logic.Mode3TimCnt >= App.Logic.u16WfCyc){
					App.Logic.Mode3TimCnt = 0;
					App.Logic.Mode3Cha = 1;
				}
			}
		}else if(App.Logic.Mode3Cha == 1){	//CCW
			App.FB2.Dir.Set = CCW;
			if(App.Logic.Mode31msFg == 1){
				App.Logic.Mode31msFg = 0;
				App.Logic.Mode3TimCnt ++;
				if(App.Logic.Mode3TimCnt >=(App.Logic.u16WfCyc-10)){	//规划刹车
						App.Logic.Brake_Sign = 1;
						App.Logic.Brake_Kind = 5;
				}
				if(App.Logic.Mode3TimCnt >= App.Logic.u16WfCyc){
					App.Logic.Mode3TimCnt = 0;
					App.Logic.Mode3Cha = 0;
				}
			}
		}
		
		//顺便判定启动条件
		if(		App.FB2.Err == E_NONE 									//无错误
			&&	App.FB2.Spd_Set >= (MCPara2[12]*10)		//设定转速大于最小转速
			&&	App.Logic.Motor_Kind ==2							//电机种类正确
			&&  App.Logic.Brake_Sign ==0							//不在刹车	
			&&  App.Logic.Motor_Stop_StartFlag == 0		//停机时间
			&&  YSorWSFlag ==1												//确认是无刷电机	
		){
			App.FB2.Start =1;
		}else{
			App.FB2.Start =0;
		}		

	}
	else if((App.Logic.CtlMode == 4)&&(App.FB2.Err == E_NONE)){
		if((App.Pos.Start == 1)&&(App.Logic.Motor_Kind ==2)){			//有步数发送过来，并且是电机1，那么就允许启动
			App.FB2.Start = 1;
			App.FB2.Dir.Now = CW;
		}else{
			App.FB2.Start = 0;
		}
	}else if((App.Logic.CtlMode == 5)&&(App.FB2.Err == E_NONE)){
		if((App.Pos.Start == 1)&&(App.Logic.Motor_Kind ==2)){			//有步数发送过来，并且是电机1，那么就允许启动
			App.FB2.Start = 1;
			App.FB2.Dir.Now = CW;
		}else{
			App.FB2.Start = 0;
		}
	}
	else if(App.Logic.CtlMode == 0){
		App.FB2.Start = 0;
	}
	
	//模式4/5的PWM控制步数控制都在这里：分3步
	if((App.Logic.CtlMode == 4||App.Logic.CtlMode == 5)&&(App.Logic.Motor_Kind ==2)){			//模式4/5的PWM控制和步数控制在这里
		if(App.Pos.LockPosStatus == 1){								//第一步：设置步数，锁定步数，判断霍尔是否正确
			TMR_SetCompare4(TMR2, 800);
			App.Pos.PlanSteps  = App.Pos.PosSetSteps ;	//锁定步数
			if(App.Pos.PosNow2 == 0x1F){								//之前没有锁定位置
				App.Pos.PosHallVal = gethall2;						//读取Hall;
				for(i=0; i<7; i++ ){
					if(App.Pos.PosHallVal == RAMP_TABLE_Hall[i]){
						App.Pos.PosHalli = i;
						if(App.Pos.PosHalli == 0){						//往前位移一个数组
							App.Pos.PosHalli = 5;
						}else{
							App.Pos.PosHalli --;
						}
					}
				}
			}else{
				App.Pos.PosHalli = App.Pos.PosNow2;		//之前锁定的位置
			}
			Hall_PhaseChange2(RAMP_TABLE_Hall[App.Pos.PosHalli],7000);		//90%,定位到目前获取到的Hall的中间位置
			Delay_ms(MCPara2[54]);
			if((App.Pos.PosHallVal !=0)||(App.Pos.PosHallVal !=7)){			//判定霍尔的值不为0和7
				App.Pos.LockPosStatus = 2;						//跳转第二步
			}else {
				App.FB2.Err = E_NOHALL;
			}
		}else if(App.Pos.LockPosStatus ==2){			//第二步：根据锁定的步数，去拖动执行
			TMR_SetCompare4(TMR2, MCPara2[56]);
			//给固定PWM给BUCK电路
			if(( App.Pos.PlanSteps > 0)&&(App.Pos.DragFlash ==1)){
				App.Pos.DragFlash = 0;
				if(App.Logic.CtlMode == 4){
					App.Pos.PosHalli++;
					if(App.Pos.PosHalli >5){
						App.Pos.PosHalli = 0;
					}
				}else if(App.Logic.CtlMode == 5){
					App.Pos.PosHalli--;
					if(App.Pos.PosHalli < 0){						//给定期望位置
						App.Pos.PosHalli = 5;
					}
				}
				App.Pos.PosHallVal = RAMP_TABLE_Hall[App.Pos.PosHalli];
				Hall_PhaseChange2(App.Pos.PosHallVal,7000);		//90%			定位到期望的位置
				
				Delay_ms(MCPara2[54]);
				App.Pos.PlanSteps -- ;
			}else if(( App.Pos.PlanSteps > 0)&&(App.Pos.DragFlash ==0)){
				//间隔
			
			}else if (App.Pos.PlanSteps == 0){
				App.Pos.LockPosStatus = 3;		//去定位一段时间
				App.Pos.LocatedTimeCnt = 0;		//清
			}
		}else if (App.Pos.LockPosStatus ==3){
				if(App.Pos.LocatedTimeCnt < App.Pos.LocatedTim){
					//定位
				}else {
					App.Pos.LockPosStatus = 0;	//回到等待状态
					App.Pos.LocatedTimeCnt = 0;	//清
					//关闭
					TMR_SetCompare4(TMR2, 0);
					App.Pos.Start = 0;
					//记录位置
					App.Pos.PosNow2 = App.Pos.PosHalli;
				}			
				
				App.Pos.OutSidePosition2 = 0;	
		}else if (App.Pos.LockPosStatus ==0){
			//等待状态
		}
	}	
	
	if(App.FB2.Status == HS_POSBRA){		//如果在位置刹车
		if(App.Logic.CtlMode == 0 ||App.Logic.CtlMode == 1||App.Logic.CtlMode == 2||App.Logic.CtlMode == 3){
			
			App.FB2.Prot.HallDragCnt++;
			if(App.FB2.Prot.HallDragCnt > MCPara2[66]){
				App.FB2.Prot.HallDragCnt = 0;
				App.FB2.Err = E_PosDrag;
				PWM_3l_Stop2();
					//不需要定位或者定位结束
				TMR_SetCompare4(TMR2, 0);	
				App.FB2.Status = HS_WAIT;
			}

			if(App.Pos.PlanPosStep2 > 0){
				App.Pos.Tempi2 --;
				if(App.Pos.Tempi2 < 0){
					App.Pos.Tempi2 = 5;
				}
				App.Pos.PlanPosStep2 --;
				Hall_PhaseChange2(RAMP_TABLE_Hall[App.Pos.Tempi2],7000);		//90%,定位到目前获取到的Hall的中间位置
				Delay_ms(MCPara2[51]);	
			}
			else if(App.Pos.PlanPosStep2 < 0){
			if(App.Pos.PlanPosStep2 < 0){
				App.Pos.Tempi2 ++;
				if(App.Pos.Tempi2 > 5){
					App.Pos.Tempi2 = 0;
				}
				App.Pos.PlanPosStep2 ++;
				Hall_PhaseChange2(RAMP_TABLE_Hall[App.Pos.Tempi2],7000);		//90%,定位到目前获取到的Hall的中间位置
				Delay_ms(MCPara2[51]);	
				}
			}
			else{
			PWM_3l_Stop2();
				//不需要定位或者定位结束
			TMR_SetCompare4(TMR2, 0);	
			App.FB2.Status = HS_WAIT;
			}
		}
		
		if(App.FB2.Err != E_NONE){
				PWM_3l_Stop2();
					//不需要定位或者定位结束
				TMR_SetCompare4(TMR2, 0);	
				App.FB2.Status = HS_ERR;
		}
	}
	
	
}


//定位1ms控制函数
void Pos_1ms(void){
	//第二步，拖动时间
	if(App.Pos.LockPosStatus == 2){
		App.Pos.DragTimCnt ++;
		if(App.Pos.DragTimCnt >App.Pos.DragTim ){
			App.Pos.DragTimCnt = 0;
			App.Pos.DragFlash = 1;
		}
	}
	//第三步，定位时间
	else if (App.Pos.LockPosStatus == 3){
		App.Pos.LocatedTimeCnt ++;
	}
	//停机时间
	if(App.FB.Status == HS_STOP){
		App.Pos.WaitStopTimCnt1 ++;
		if(App.Pos.WaitStopTimCnt1 >1000){	//限幅
			App.Pos.WaitStopTimCnt1 = 0;
			//报错
		}
		
	}
	//停机时间
	if(App.FB2.Status == HS_STOP){
		App.Pos.WaitStopTimCnt2 ++;
		if(App.Pos.WaitStopTimCnt2 >1000){	//限幅
			App.Pos.WaitStopTimCnt2 = 0;
			//报错
		}
	}	
}


void Logic_1ms(void){		//1ms
	App.FB.CheckCyc = 1;//保护检测周期
	//软件触发ADC采样
	ADC_SoftwareStart;
}


void Logic_500ms(void){//500ms中断
	App.FB.StaLedCyc = 1;//LED灯
}

void Logic_Sotf_Init(void){//上电初始化
	//逻辑	
	App.Logic.CtlMode = STOP_STATUS;
	App.Logic.Motor_Kind = 0;					//默认电机1
	App.Logic.LogicHall_or_pll =2;				//默认的霍尔状态	

	App.Logic.Set_Spd = 0;
	
}


//状态灯控制，哪个电机正在运行哪个电机有错误就显示其错误灯，也有可能是系统错误！
void STA_Led(uint8_t fault_num){
#if _STA_LED == 1
	static uint8_t cnt = 0;
	if(App.FB.StaLedCyc == 1){
		App.FB.StaLedCyc = 0;
		
		if(fault_num > 0){
			cnt ++;
			if(cnt <= 2){//1 2 3
				STA_LED = 1;//关
			}else if(cnt <= (fault_num * 2 - 1) + 2){
				STA_LED = ~STA_LED;
			}
			else{
				cnt = 0;
				STA_LED = 1;//关
			}
		}else if((App.FB.AllRun == 0)&&(App.FB2.AllRun ==0)){
			STA_LED = 0;//开;//在等待状态下 状态指示灯常亮
			cnt = 0;
		}else if((App.FB.AllRun == 1)||(App.FB2.AllRun == 1)){
			STA_LED = 1;//关;//在等待状态下 状态指示灯常亮
			cnt = 0;
		}
	}
#endif
}


