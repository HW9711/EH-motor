#include "pospid.h"
#include "mcctl.h"
#include "userpara.h"
#include "enocfoc.h"


POSSpd_TYPEDEF 								PosSpd;
POSSpd_TYPEDEF 								PosSpd2;

//初始化路径中点
void Pos_SetSpd_Init(void){
	//记录起点和终点的中间位置
	PosSpd.OldNow = App.M1.Pos.NowCnt;//记录当前位置 用于和设定位置做对比
	//算出正的速度
	if(PosSpd.OldNow <= App.M1.Pos.SetCnt){//如果当前位置小于设定位置 0 -> 100 == 0-30up mid 30-100dw
		PosSpd.MidPos = PosSpd.OldNow + ((App.M1.Pos.SetCnt - PosSpd.OldNow) * MCPara[120] / 10)  ;//30% 为加速部分
	}
	
	//算出负的速度
	else{//如果当前位置大于设定位置 100 -> 0 == 100-70up mid 70-0dw
		PosSpd.MidPos = App.M1.Pos.SetCnt + ((PosSpd.OldNow - App.M1.Pos.SetCnt) * (10 - MCPara[120]) / 10);//30% 为加速部分
	}
	
	//初始化起点最小速度
	PosSpd.StaMinSpd = MCPara[121];
	//初始化终点最小速度
	PosSpd.EndMinSpd = MCPara[122];
	
	//初始化全程最大速度
	if(App.Ctl.SetWeiNum == 0){
		PosSpd.AllMaxSpd = MCPara[123];
	}else if(App.Ctl.SetWeiNum == 1){
		PosSpd.AllMaxSpd = MCPara[100];
	}else if(App.Ctl.SetWeiNum == 2){
		PosSpd.AllMaxSpd = MCPara[101];
	}else if(App.Ctl.SetWeiNum == 3){
		PosSpd.AllMaxSpd = MCPara[102];
	}else if(App.Ctl.SetWeiNum == 4){
		PosSpd.AllMaxSpd = MCPara[103];
	}
	
	//初始化加速 斜率
	PosSpd.UpS = MCPara[124];
	//初始化减速 斜率
	PosSpd.DownS = MCPara[125];
}


	//初始化路径中点
void Pos2_SetSpd_Init(void){
	//记录起点和终点的中间位置
	PosSpd2.OldNow = App.M2.Pos.NowCnt;//记录当前位置 用于和设定位置做对比
	//算出正的速度
	if(PosSpd2.OldNow <= App.M2.Pos.SetCnt){//如果当前位置小于设定位置 0 -> 100 == 0-30up mid 30-100dw
		PosSpd2.MidPos = PosSpd2.OldNow + ((App.M2.Pos.SetCnt - PosSpd2.OldNow) * MCPara2[120] / 10)  ;//30% 为加速部分
	}
	
	//算出负的速度
	else{//如果当前位置大于设定位置 100 -> 0 == 100-70up mid 70-0dw
		PosSpd2.MidPos = App.M2.Pos.SetCnt + ((PosSpd2.OldNow - App.M2.Pos.SetCnt) * (10 - MCPara2[120]) / 10)  ;//30% 为加速部分
	}
	
	//初始化起点最小速度
	PosSpd2.StaMinSpd = MCPara2[121];
	//初始化终点最小速度
	PosSpd2.EndMinSpd = MCPara2[122];
	//初始化全程最大速度
	PosSpd2.AllMaxSpd = MCPara2[123];
	//初始化加速 斜率
	PosSpd2.UpS = MCPara2[124];
	//初始化减速 斜率
	PosSpd2.DownS = MCPara2[125];
}





//运行中给定转速，运行中判断位置时候到位
int32_t Pos_SetSpd(int64_t nowpos,POSSpd_TYPEDEF posspd){
	float tempspd = 0;
	
	//在运行过程中，给定位置发生了变化,就重新初始化一遍
	if(App.M1.Pos.OldSetCnt != App.M1.Pos.SetCnt){
		App.M1.Pos.OldSetCnt = App.M1.Pos.SetCnt;//记录运行过程中，位置是否发生变化
		Pos_SetSpd_Init();
	}
	
	//算出正的速度
	if(PosSpd.OldNow <= App.M1.Pos.SetCnt){//如果当前位置小于设定位置 0 -> 100 == 0-30up mid 30-100dw
		//方向
		App.M1.Dir.TempSet = CW;
		
		if(nowpos <= PosSpd.MidPos){//0 -> 100 == 0-30up mid 30-100dw 加速段
			tempspd = (nowpos - PosSpd.OldNow) * PosSpd.UpS / 100  + MCPara[121];
			if(tempspd > PosSpd.AllMaxSpd){
				tempspd = PosSpd.AllMaxSpd;
			}else if(tempspd < PosSpd.StaMinSpd){
				tempspd = PosSpd.StaMinSpd;
			}
		}else if(nowpos > PosSpd.MidPos){//0 -> 100 == 0-30up mid 30-100dw 减速段
			tempspd = (App.M1.Pos.SetCnt - nowpos) * PosSpd.DownS / 100  + MCPara[122];
			if(tempspd > PosSpd.AllMaxSpd){
				tempspd = PosSpd.AllMaxSpd;
			}else if(tempspd < PosSpd.EndMinSpd){
				tempspd = PosSpd.EndMinSpd;
			}
		}
		
		//判断位置是否到位 
		if(nowpos - App.M1.Pos.SetCnt >= -MCPara[126]){
			App.M1.Pos.PosStatus = POS_STOP;//状态跳转到停止
			App.M1.Pos.PosArriFg = 1;
		}
	}
	
	//算出负的速度
	else{//如果当前位置大于设定位置 100 -> 0 == 100-70up mid 70-0dw
		
		//方向
		App.M1.Dir.TempSet = CCW;
		
		if(nowpos <= PosSpd.MidPos){//100 -> 0 == 100-70up mid 70-0dw 减速段
			tempspd = (App.M1.Pos.SetCnt - nowpos) * PosSpd.DownS / 100 - MCPara[122];
			if(tempspd < -PosSpd.AllMaxSpd){
				tempspd = -PosSpd.AllMaxSpd;
			}else if(tempspd > -PosSpd.EndMinSpd){
				tempspd = -PosSpd.EndMinSpd;
			}
		}else if(nowpos > PosSpd.MidPos){//100 -> 0 == 100-70up mid 70-0dw 加速段
			tempspd = (nowpos - PosSpd.OldNow) * PosSpd.UpS / 100 - MCPara[121];
			if(tempspd < -PosSpd.AllMaxSpd){
				tempspd = -PosSpd.AllMaxSpd;
			}else if(tempspd > -PosSpd.StaMinSpd){
				tempspd = -PosSpd.StaMinSpd;
			}
		}
		
		//判断位置是否到位
		if(App.M1.Pos.SetCnt - nowpos >= -MCPara[126]){
			App.M1.Pos.PosStatus = POS_STOP;//状态跳转到停止
			App.M1.Pos.PosArriFg = 1;
		}
	}
	
	return tempspd;
}



int32_t Pos_SetSpd_Lock(int64_t nowpos,POSSpd_TYPEDEF posspd){
	float tempspd = 0;
	
//	//在运行过程中，给定位置发生了变化,就重新初始化一遍
//	if(App.M1.Pos.OldSetCnt != App.M1.Pos.SetCnt){
//		App.M1.Pos.OldSetCnt = App.M1.Pos.SetCnt;//记录运行过程中，位置是否发生变化
//		Pos_SetSpd_Init();
//	}
	
	//算出正的速度
	if(nowpos <= App.M1.Pos.SetCnt){//如果当前位置小于设定位置 0 -> 100 == 0-30up mid 30-100dw
		//方向
		App.M1.Dir.TempSet = CW;
		
		tempspd = (App.M1.Pos.SetCnt - nowpos) * MCPara[128] / 100  + MCPara[122];
		if(tempspd > MCPara[129]){
			tempspd = MCPara[129];
		}else if(tempspd < MCPara[122]){
			tempspd = MCPara[122];
		}
		
		//判断位置是否到位 
		if(nowpos - App.M1.Pos.SetCnt >= -MCPara[126]){
			App.M1.Pos.PosStatus = POS_STOP;//状态跳转到停止
			App.M1.Pos.PosArriFg = 1;
		}
	}
	
	//算出负的速度
	else{//如果当前位置大于设定位置 100 -> 0 == 100-70up mid 70-0dw
		
		//方向
		App.M1.Dir.TempSet = CCW;
		
		tempspd = (App.M1.Pos.SetCnt - nowpos) * MCPara[128] / 100 - MCPara[122];
		if(tempspd < -MCPara[129]){
			tempspd = -MCPara[129];
		}else if(tempspd > -MCPara[122]){
			tempspd = -MCPara[122];
		}
		
		//判断位置是否到位
		if(App.M1.Pos.SetCnt - nowpos >= -MCPara[126]){
			App.M1.Pos.PosStatus = POS_STOP;//状态跳转到停止
			App.M1.Pos.PosArriFg = 1;
		}
	}
	
	return tempspd;
}




//运行中给定转速，运行中判断位置时候到位
int32_t Pos2_SetSpd(int64_t nowpos,POSSpd_TYPEDEF posspd){
	float tempspd = 0;
	
	//在运行过程中，给定位置发生了变化,就重新初始化一遍
	if(App.M2.Pos.OldSetCnt != App.M2.Pos.SetCnt){
		App.M2.Pos.OldSetCnt = App.M2.Pos.SetCnt;//记录运行过程中，位置是否发生变化
		Pos2_SetSpd_Init();
	}
	
	//算出正的速度
	if(PosSpd2.OldNow <= App.M2.Pos.SetCnt){//如果当前位置小于设定位置 0 -> 100 == 0-30up mid 30-100dw
		//方向
		App.M2.Dir.TempSet = CW;
		
		if(nowpos <= PosSpd2.MidPos){//0 -> 100 == 0-30up mid 30-100dw 加速段
			tempspd = (nowpos - PosSpd2.OldNow) * PosSpd2.UpS / 100  + MCPara2[121];
			if(tempspd > PosSpd2.AllMaxSpd){
				tempspd = PosSpd2.AllMaxSpd;
			}else if(tempspd < PosSpd2.StaMinSpd){
				tempspd = PosSpd2.StaMinSpd;
			}
		}else if(nowpos > PosSpd2.MidPos){//0 -> 100 == 0-30up mid 30-100dw 减速段
			tempspd = (App.M2.Pos.SetCnt - nowpos) * PosSpd2.DownS / 100  + MCPara2[122];
			if(tempspd > PosSpd2.AllMaxSpd){
				tempspd = PosSpd2.AllMaxSpd;
			}else if(tempspd < PosSpd2.EndMinSpd){
				tempspd = PosSpd2.EndMinSpd;
			}
		}
		
		//判断位置是否到位 
		if(nowpos - App.M2.Pos.SetCnt >= -MCPara2[126]){
			App.M2.Pos.PosStatus = POS_STOP;//状态跳转到停止
			App.M2.Pos.PosStart = 0;//清除使能位
			App.M2.Pos.PosArriFg = 1;
		}
	}
	
	//算出负的速度
	else{//如果当前位置大于设定位置 100 -> 0 == 100-70up mid 70-0dw
		
		//方向
		App.M2.Dir.TempSet = CCW;
		
		if(nowpos <= PosSpd2.MidPos){//100 -> 0 == 100-70up mid 70-0dw 减速段
			tempspd = (App.M2.Pos.SetCnt - nowpos) * PosSpd2.DownS / 100 - MCPara2[122];
			if(tempspd < -PosSpd2.AllMaxSpd){
				tempspd = -PosSpd2.AllMaxSpd;
			}else if(tempspd > -PosSpd2.EndMinSpd){
				tempspd = -PosSpd2.EndMinSpd;
			}
		}else if(nowpos > PosSpd2.MidPos){//100 -> 0 == 100-70up mid 70-0dw 加速段
			tempspd = (nowpos - PosSpd2.OldNow) * PosSpd2.UpS / 100 - MCPara2[121];
			if(tempspd < -PosSpd2.AllMaxSpd){
				tempspd = -PosSpd2.AllMaxSpd;
			}else if(tempspd > -PosSpd2.StaMinSpd){
				tempspd = -PosSpd2.StaMinSpd;
			}
		}
		
		//判断位置是否到位
		if(App.M2.Pos.SetCnt - nowpos >= -MCPara2[126]){
			App.M2.Pos.PosStatus = POS_STOP;//状态跳转到停止
			App.M2.Pos.PosStart = 0;//清除使能位
			App.M2.Pos.PosArriFg = 1;
		}
	}
	
	return tempspd;
}



//运行中给定转速，运行中判断位置时候到位
int32_t Pos2_SetSpd_Lock(int64_t nowpos,POSSpd_TYPEDEF posspd){
	float tempspd = 0;
	
//	//在运行过程中，给定位置发生了变化,就重新初始化一遍
//	if(App.M2.Pos.OldSetCnt != App.M2.Pos.SetCnt){
//		App.M2.Pos.OldSetCnt = App.M2.Pos.SetCnt;//记录运行过程中，位置是否发生变化
//		Pos2_SetSpd_Init();
//	}
	
	//算出正的速度
	if(nowpos <= App.M2.Pos.SetCnt){//如果当前位置小于设定位置 0 -> 100 == 0-30up mid 30-100dw
		//方向
		App.M2.Dir.TempSet = CW;
		//根据位置设定转速
		tempspd = (App.M2.Pos.SetCnt - nowpos) * MCPara2[128] / 100  + MCPara2[122];
		if(tempspd > MCPara2[122]){
			tempspd = MCPara2[122];
		}else if(tempspd < MCPara2[122]){
			tempspd = MCPara2[122];
		}
		
		//判断位置是否到位 
		if(nowpos - App.M2.Pos.SetCnt >= -MCPara2[126]){
			App.M2.Pos.PosStatus = POS_STOP;//状态跳转到停止
			App.M2.Pos.PosArriFg = 1;
		}
	}
	
	//算出负的速度
	else{//如果当前位置大于设定位置 100 -> 0 == 100-70up mid 70-0dw
		
		//方向
		App.M2.Dir.TempSet = CCW;
		
		tempspd = (App.M2.Pos.SetCnt - nowpos) * MCPara2[128] / 100 - MCPara2[122];
		if(tempspd < -MCPara2[129]){
			tempspd = -MCPara2[129];
		}else if(tempspd > -MCPara2[122]){
			tempspd = -MCPara2[122];
		}
		
		//判断位置是否到位
		if(App.M2.Pos.SetCnt - nowpos >= -MCPara2[126]){
			App.M2.Pos.PosStatus = POS_STOP;//状态跳转到停止
			App.M2.Pos.PosArriFg = 1;
		}
	}
	
	return tempspd;
}



#define turns_enoc_num 118374


void Pos_Sta(void){
	//位置控制状态机
	switch(App.M1.Pos.PosStatus){
		case POS_WAIT://等待
			App.M1.Spd.Set = 0;
		
			if(App.M1.Err == E_NONE){
				//位置偏移已经记录好了
				if(App.M1.Pos.PosLocaFg == 1){
					//如果设定了圈数，启动运行
					if(
						App.Ctl.SetTurns > App.Ctl.NowTurns //设定了圈数
						&& App.Ctl.SetWeiNum >= 1 //已经设置了圈数
						&& App.M2.Pos.PosLocaFg == 1 //已经归位完成
						&& App.M2.Err == E_NONE//威力电机不能出错
					){
						App.M1.Pos.PosStatus = CTLS_SPD;
						
						App.Ctl.OldPrsINSendCnt = App.Ctl.PrsINSendCnt;//记录当前发生的圈数
						App.Ctl.OldPos = App.M1.Pos.NowCnt;//记录当前位置
						
						App.Ctl.Old1PrsINSendCnt = App.Ctl.PrsINSendCnt;//记录当前发生的圈数
					}
					
					//没有设定圈数,就闭环到上次的位置
					else {
						//判定位置是否发生了偏移
						if( myabs( (App.M1.Pos.SetCnt - App.M1.Pos.NowCnt) ) > MCPara[127]){
							//重新计算加减速
							App.M1.Pos.OldSetCnt = App.M1.Pos.SetCnt - 1;
							App.M1.Pos.PosStatus = CTLS_POS;//跳转到运行模式
						}
						
					}

				}
				//供弹电机初始位置还未记录
				else{
					App.M1.Pos.PosStatus = POS_INIT;//调整到记录初始位置
				}
			}
			
			
					break;
		
		case POS_INIT://找零 门位置
			if(App.M1.Enoc.Sta == 1){
				App.M1.Pos.ZNowCnt = 0;//清除圈数，怕位置溢出
				App.M1.Pos.NowOffSet = read_enoc;
				App.M1.Pos.NowCnt = App.M1.Pos.ZNowCnt * MCPara[83] + read_enoc - App.M1.Pos.NowOffSet;
				
				App.M1.Pos.SetCnt = 0;
				
				App.M1.Pos.PosLocaFg = 1;
				App.M1.Pos.PosStatus = POS_WAIT;
			}
			
			if(App.M1.Err != E_NONE){
				App.M1.Pos.PosStatus = POS_STOP;
			}
			
					break;
		case CTLS_SPD:
			//设定转速
			if(App.Ctl.SetWeiNum == 1){
				App.M1.Spd.Set = MCPara[100];
			}else if(App.Ctl.SetWeiNum == 2){
				App.M1.Spd.Set = MCPara[101];
			}else if(App.Ctl.SetWeiNum == 3){
				App.M1.Spd.Set = MCPara[102];
			}else if(App.Ctl.SetWeiNum == 4){
				App.M1.Spd.Set = MCPara[103];
			}
			
		
			if(MCPara[30] == 0){
				App.M1.Dir.TempSet = CW;
			}else {
				App.M1.Dir.TempSet = CCW;
			}
			
			
			//检测是否转过一圈了还没检测到接近开关
			//圈数还未更新
			if(App.Ctl.OldPrsINSendCnt == App.Ctl.PrsINSendCnt){
				if( (myabs((App.M1.Pos.NowCnt - App.Ctl.OldPos)))   >= (turns_enoc_num + 10000) ){//% turns_enoc_num
					App.M1.Err = E_Pri;//接近开关长时间不触发
					App.BitErr |= ERR_GD_Pri;//供弹电机接近开关错误
				}
			}else if( (App.Ctl.PrsINSendCnt - App.Ctl.OldPrsINSendCnt) >= 2){
				App.M1.Err = E_Pri;//一圈触发多次
				App.BitErr |= ERR_GD_Pri;//供弹电机接近开关错误
			}
			else {//圈数更新了
				App.Ctl.OldPrsINSendCnt = App.Ctl.PrsINSendCnt;
				App.Ctl.OldPos = App.M1.Pos.NowCnt;
			}
			
			//如果接近开关已经检测到1次了，那么需要过了某个角度之后，用Can发送一次数据。
			if(App.Ctl.Old1PrsINSendCnt != App.Ctl.PrsINSendCnt){
				if(MCPara[30] == 0){
					if((App.Ctl.PrsINSendPos + MCPara[104] * 10 - 300) < App.M1.Pos.NowCnt){
						App.Ctl.Old1PrsINSendCnt = App.Ctl.PrsINSendCnt;
						App.Ctl.PrsINSendFg = 1;//发送一次接近开关位置
					}
					
				}else {
					if((App.Ctl.PrsINSendPos - MCPara[104] * 10 + 300) > App.M1.Pos.NowCnt){
						App.Ctl.Old1PrsINSendCnt = App.Ctl.PrsINSendCnt;
						App.Ctl.PrsINSendFg = 1;//发送一次接近开关位置
					}
				}
			}
			
			
			//检测到圈数到了-跳转到停止
			if(
				(App.Ctl.SetTurns != 0 && App.Ctl.SetTurns <= App.Ctl.NowTurns)
				//如果在击发过程中停止，那么需要转完本圈
				|| (App.Ctl.SetTurns == 0 && App.Ctl.SetTurns < App.Ctl.NowTurns)
			){
				
				//防止位置溢出，重新调整当前位置
				App.M1.Pos.ZNowCnt = 0;//清除圈数，怕位置溢出
				App.M1.Pos.NowOffSet = read_enoc;
				App.M1.Pos.NowCnt = App.M1.Pos.ZNowCnt * MCPara[83] + read_enoc - App.M1.Pos.NowOffSet;
				App.Ctl.RelayAng = App.M1.Pos.NowCnt;
				
				App.Ctl.PrsINSendPos = App.M1.Pos.NowCnt;//记录接近开关中断
				
				if(MCPara[30] == 0){
					App.M1.Pos.SetCnt = MCPara[104] * 10 + App.M1.Pos.NowCnt;//59187 在原点开关的另一侧
				}else {
					App.M1.Pos.SetCnt = App.M1.Pos.NowCnt - MCPara[104] * 10;//59187 在原点开关的另一侧
				}
				
				//重新计算加减速
				App.M1.Pos.OldSetCnt = App.M1.Pos.SetCnt - 1;
				//跳转到运行模式
				App.M1.Pos.PosStatus = CTLS_RUN;
			}
			
			#if _Relay_SW == 0
			//已经记录了接近开关位置
			if(App.Ctl.RelaySta == 1){
				if(App.M1.Dir.TempSet == CW){
					if(App.M1.Pos.NowCnt >= (App.Ctl.RelayAng + 328.818f * MCPara[105])){
						Relay_ON;
						App.Ctl.RelaySta = 2;
					}
				}else{
					if(App.M1.Pos.NowCnt <= (App.Ctl.RelayAng - 328.818f * MCPara[105])){
						Relay_ON;
						App.Ctl.RelaySta = 2;
					}
				}
			}else if(App.Ctl.RelaySta == 2){
				if(App.M1.Dir.TempSet == CW){
					if(App.M1.Pos.NowCnt >= (App.Ctl.RelayAng + 328.818f * (MCPara[105] + MCPara[106]))){
						Relay_OFF;
						App.Ctl.RelaySta = 0;
					}
				}else{
					if(App.M1.Pos.NowCnt <= (App.Ctl.RelayAng - 328.818f * (MCPara[105] + MCPara[106]))){
						Relay_OFF;
						App.Ctl.RelaySta = 0;
					}
					
				}
			}
			#endif
			
			
			//供弹电机错误，直接停止
			if(App.M1.Err != E_NONE){
				App.M1.Pos.PosStatus = POS_STOP;
			}
			//威力电机错误
			else if(App.M2.Err != E_NONE){
				
				//如果威力电机发生了错误，那么供弹电机就停止
				//App.Ctl.RelayAng = App.M1.Pos.NowCnt;
				
				if(MCPara[30] == 0){
					App.M1.Pos.SetCnt = MCPara[104] * 10 + App.Ctl.RelayAng;//59187 在原点开关的另一侧
				}else {
					App.M1.Pos.SetCnt = App.Ctl.RelayAng - MCPara[104] * 10;//59187 在原点开关的另一侧
				}
				//跳转到运行模式
				App.M1.Pos.PosStatus = CTLS_RUN;
				
			}
			
					break;
		case CTLS_RUN://运行到位置
			//根据当前位置设定转速
			//根据当前位置判定是否到位
			App.M1.Spd.Set = myabs( (Pos_SetSpd( App.M1.Pos.NowCnt,PosSpd)) );
			//上面步骤运行完就转到锁轴
		
			//如果接近开关已经检测到1次了，那么需要过了某个角度之后，用Can发送一次数据。
			if(App.Ctl.Old1PrsINSendCnt != App.Ctl.PrsINSendCnt){
				if(MCPara[30] == 0){
					if((App.Ctl.PrsINSendPos + MCPara[104] * 10 - 300) < App.M1.Pos.NowCnt){
						App.Ctl.Old1PrsINSendCnt = App.Ctl.PrsINSendCnt;
						App.Ctl.PrsINSendFg = 1;//发送一次接近开关位置
					}
					
				}else {
					if((App.Ctl.PrsINSendPos - MCPara[104] * 10 + 300) > App.M1.Pos.NowCnt){
						App.Ctl.Old1PrsINSendCnt = App.Ctl.PrsINSendCnt;
						App.Ctl.PrsINSendFg = 1;//发送一次接近开关位置
					}
				}
			}
		
		
		#if _Relay_SW == 0
			//已经记录了接近开关位置
			if(App.Ctl.RelaySta == 1){
				if(App.M1.Dir.TempSet == CW){
					if(App.M1.Pos.NowCnt >= (App.Ctl.RelayAng + 328.818f * MCPara[105])){
						Relay_ON;
						App.Ctl.RelaySta = 2;
					}
				}else{
					if(App.M1.Pos.NowCnt <= (App.Ctl.RelayAng - 328.818f * MCPara[105])){
						Relay_ON;
						App.Ctl.RelaySta = 2;
					}
				}
			}else if(App.Ctl.RelaySta == 2){
				if(App.M1.Dir.TempSet == CW){
					if(App.M1.Pos.NowCnt >= (App.Ctl.RelayAng + 328.818f * (MCPara[105] + MCPara[106]))){
						Relay_OFF;
						App.Ctl.RelaySta = 0;
					}
				}else{
					if(App.M1.Pos.NowCnt <= (App.Ctl.RelayAng - 328.818f * (MCPara[105] + MCPara[106]))){
						Relay_OFF;
						App.Ctl.RelaySta = 0;
					}
					
				}
			}
		#endif
			
			//错误也退出来
			if(App.M1.Err != E_NONE){
				#if _Relay_SW == 0
				Relay_OFF;
				#endif
				App.M1.Pos.PosStatus = POS_STOP;
			}
			
					break;
		case CTLS_POS://锁轴功能
			//在锁轴功能时，如果设定了圈数，则立马执行圈数
			if(
				App.Ctl.SetTurns > App.Ctl.NowTurns 
				&& App.Ctl.SetWeiNum >= 1
				&& App.M1.Err == E_NONE
				&& App.M2.Err == E_NONE
			){
				App.M1.Pos.PosStatus = POS_WAIT;
			}
			//否则执行锁轴功能
			else {
				App.M1.Spd.Set = myabs( (Pos_SetSpd_Lock( App.M1.Pos.NowCnt,PosSpd)) );
			}
			
		
			//错误也退出来
			if(App.M1.Err != E_NONE){
				#if _Relay_SW == 0
				Relay_OFF;
				#endif
				App.M1.Pos.PosStatus = POS_STOP;
			}
			
					break;
			
		case POS_STOP://停止
			App.M1.Spd.Set = 0;//电机停止
			//App.M1.Pos.PosStart = 0;//清除使能位
			App.M1.Pos.PosStatus = POS_WAIT;//状态跳转到等待
					break;
		
		
		default:           
					break;
	}
	
}





void Pos2_Sta(void){
	//位置控制状态机
	switch(App.M2.Pos.PosStatus){
		case POS_WAIT://等待
			App.M2.Spd.Set = 0;
			
			if(
				App.M2.Err == E_NONE
				//&& App.M1.Err == E_NONE
			){
				//已经定位完成  && App.Ctl.SetWeiNum >= 1
				if(App.M2.Pos.PosLocaFg == 1){
						//如果设置位置与当前位置差距过小，直接跳转到停止
						if( myabs( (App.M2.Pos.SetCnt - App.M2.Pos.NowCnt) ) < MCPara2[127]){
							App.M2.Pos.PosStatus = POS_STOP;
							App.M2.Pos.PosArriFg = 1;
						}else {
							//重新计算加减速
							App.M2.Pos.OldSetCnt = App.M2.Pos.SetCnt - 1;
							App.M2.Pos.PosStatus = CTLS_RUN;//跳转到运行模式
						}
				}
				//还未定位
				else {
					//开始回原
					if(App.M2.Pos.PosStart == 2){
						App.M2.Pos.CheckStallCnt = 0;
						App.M2.Pos.LocaTimCnt = 0;
						App.M2.Pos.PosStatus = POS_INIT;//跳转到回原
					}
				}
			}
			
					break;
		
		case POS_INIT://找零
			//找零判定：发生堵转，就判定为定位OK
			if(App.M2.Pos.CheckCur1msFg == 1){
				App.M2.Pos.CheckCur1msFg = 0;
				
				//还未发生堵转
				if(App.M2.Pos.PosLocaFg == 0){
					if(App.M2.Spd.Set < MCPara2[105]){
						App.M2.Spd.Set += 0.1f;//速度给定慢慢增加
					}
					
					if(MCPara2[106] == 0){// 正的方向
						App.M2.Dir.TempSet = CW;
					}else{//负的方向
						App.M2.Dir.TempSet = CCW;
					}
					
					if(App.M2.Cal.AllCurLPF >= MCPara2[107] / 100.0f){//堵转了 立马停止
						App.M2.Pos.CheckOCCnt ++;
						if(App.M2.Pos.CheckOCCnt >= MCPara2[108]){//参p
							App.M2.Pos.CheckOCCnt = 0;
							
							App.M2.Pos.PosLocaFg = 2;//已经发生了恒流堵转
							App.M2.Pos.LocaTimCnt = 0;
						}
					}else if(App.M2.Pos.CheckOCCnt > 0){
						App.M2.Pos.CheckOCCnt --;
					}

				}
				
				
				
				//发生了堵转
				if(App.M2.Pos.PosLocaFg == 2){//堵转之后，等待Nms，再归零一段时间，然后位置归零
					App.M2.Spd.Set = 0;
					App.M2.Pos.CheckStallCnt = 0;
					App.M2.Pos.LocaTimCnt ++;
					if(App.M2.Pos.LocaTimCnt >= MCPara2[109]){//堵转之后延时一会，在记录位置，防止抖动
						App.M2.Pos.LocaTimCnt = 0;
						
						//位置信息在这里完全清除，重新开始计算。
						App.M2.Pos.ZNowCnt = 0;//清除Z轴累加
						App.M2.Pos.NowOffSet = read_enoc2;//清除位置
						//App.M2.Pos.NowCnt = 0;
						
						
						
						
						App.M2.Pos.SetCnt = 0;
						//App.Ctl.SetWeiNum = 0;
						App.Ctl.SetWeiNum = 1;//设置挡位自动为1档
						
						App.M2.Pos.PosLocaFg = 1;//门的位置已经找零完成
						App.M2.Pos.PosStart = 0;//清除使能位
						App.M2.Pos.PosStatus = POS_STOP;
						
						
						//自动设置挡位1
						App.Ctl.SetWeiNum = 1;//设置挡位
						if(MCPara2[99] == 0){
							if(App.Ctl.SetWeiNum == 1){
								App.M2.Pos.SetCnt = MCPara2[100] * 10;
								App.M2.Pos.PosArriFg = 0;
								App.M2.Pos.PosStart = 1;
							}
						}else {
							if(App.Ctl.SetWeiNum == 1){
								App.M2.Pos.SetCnt = -MCPara2[100] * 10;
								App.M2.Pos.PosArriFg = 0;
								App.M2.Pos.PosStart = 1;
							}
						}
						
						
					}
				}
				
				
				//判断时间
				App.M2.Pos.CheckStallCnt ++;
				if(App.M2.Pos.CheckStallCnt >= MCPara2[110]){
					App.M2.Pos.CheckStallCnt = 0;//清除运动受阻判定
					App.M2.Pos.PosLocaFg = 0;//初始化失败
					App.M2.Err = E_PosInit;//位置找零时间过长，错误
					App.BitErr |= ERR_WL_PosLoca;//威力电机找零错误
				}
				
				
			}//1ms 结束
			
			
			if(App.M2.Err != E_NONE){
				App.M2.Pos.CheckStallCnt = 0;
				App.M2.Pos.PosStatus = POS_STOP;
			}

					break;
		case CTLS_RUN://运行到位置
			//根据当前位置设定转速
			//根据当前位置判定是否到位
			if(App.M2.Pos.PosStart == 1){
				App.M2.Spd.Set = myabs((Pos2_SetSpd(App.M2.Pos.NowCnt,PosSpd2)));
			}else {
				App.M2.Spd.Set = myabs((Pos2_SetSpd_Lock(App.M2.Pos.NowCnt,PosSpd2)));
			}
			
			//错误也退出来
			if(App.M2.Err != E_NONE){// || App.M2.Pos.PosStart != 1
				App.M2.Pos.PosStatus = POS_STOP;
			}
			
					break;
		case POS_STOP://停止
			App.M2.Spd.Set = 0;//电机停止
			//App.M2.Pos.PosStart = 0;//清除使能位
			App.M2.Pos.PosStatus = POS_WAIT;//状态跳转到等待
					break;
		
		
		default:           
					break;
	}
	
}












