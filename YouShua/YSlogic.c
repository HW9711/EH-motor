#include "YSlogic.h"
#include "youshua.h"
#include "logic.h"
#include "YSstatemachine.h"
#include "YSmcctl.h"
#include "mcuart.h"


//有刷逻辑控制
/*
 * 函数功能：规划有刷正反转和往复，外部停机期间只撤销启动许可
 * 输入参数：无。
 * 返回参数：无。
 */
void YSLog_Ctl(void){
    if (MotorCommand_IsStopRequested() != 0U)
    {
        App2.Ch1.Start = 0U; /* 停止优先于A路往复方向调度。 */
        App2.Ch2.Start = 0U; /* 同步撤销B路启动，不能因另一通道等待状态重新取得许可。 */
        App2.SysCtl.SpdSet = 0.0f; /* 普通调速目标保持零，制动斜率继续使用原参数。 */
        return; /* 停止完成前冻结往复相位及方向切换。 */
    }

	
	if(App2.Log.OldChannel != App2.Log.Channel){				//如果通发生变化
		App2.Log.OldChannel = App2.Log.Channel;
		//没有用先放着
	}

	if(App2.Log.Channel == 3){
		App2.Ch2.Start = 0;	//不允许电机2启动
		
		//模式0：停止不给控制
		if(App2.Log.CtlMode == 0){
			App2.Ch1.Start = 0;
			App2.Log.Mode3TimCnt = 0;
			App2.Log.Mode3Cha = 0;
		}
		//模式1 CW方向
		else if(App2.Log.CtlMode == 1){
			App2.Log.Mode3TimCnt = 0;
			App2.Log.Mode3Cha = 0;
			//给方向CW
			App2.Ch1.Dir.Set = CW;
			//串口指令给转速
			App2.SysCtl.SpdSet = App2.Log.u32SetSpd;
			//判断错误
			if(	
				App2.Err == E_NONE 
				&& App2.SysCtl.SpdSet >= YSPara1[2]*10						//启动条件
//				&& App2.SysPort.Mbus >= (YSPara1[3] / 10.0f)
				&& App2.Ch2.Status == CTLS_WAIT
				&& YSorWSFlag ==2
			){
				App2.Ch1.Start = 1;
			}else{
				App2.Ch1.Start = 0;
			}
		}
		//模式2 CCW方向
		else if(App2.Log.CtlMode == 2){
			App2.Log.Mode3TimCnt = 0;
			App2.Log.Mode3Cha = 0;
			//给方向CCW
			App2.Ch1.Dir.Set= CCW;
			//串口指令给转速
			App2.SysCtl.SpdSet = App2.Log.u32SetSpd;
			//判断错误
			if(
				App2.Err == E_NONE
				&& App2.SysCtl.SpdSet >= YSPara1[2]*10						//启动条件
//				&& App2.SysPort.Mbus  >= (MCPara[3] / 10.0f)
				&& App2.Ch2.Status == CTLS_WAIT		
				&& YSorWSFlag ==2			
			){
				App2.Ch1.Start = 1;
			}else{
				App2.Ch1.Start= 0;
			}
		}
		//模式3 往复模式
		else if(App2.Log.CtlMode == 3){
			
			if(App2.Log.u8WfFru >= 1){
				App2.Log.u16WfCyc = 10000 / App2.Log.u8WfFru;		//由频率规划正转、刹车、反转、刹车
			}else {
				App2.Log.u16WfCyc = 1000;
			}
			
			if(App2.Log.Mode3Cha == 0){													
				App2.Ch1.Dir.Set = CW;															//CW
				                                                    
				if(App2.Log.Mode31msFg == 1){                       
					App2.Log.Mode31msFg = 0;                          
					App2.Log.Mode3TimCnt ++;                          
					if(App2.Log.Mode3TimCnt >= App2.Log.u16WfCyc){	  
						App2.Log.Mode3TimCnt = 0;                       	//刹车
						                                                
						App2.Log.Mode3Cha = 1;                          
					}                                                 
				}                                                   
			}else if(App2.Log.Mode3Cha == 1){										  
				App2.Ch1.Dir.Set = CCW;                             	//CCW
				                                                    
				if(App2.Log.Mode31msFg == 1){                       
					App2.Log.Mode31msFg = 0;                          
					                                                  
					App2.Log.Mode3TimCnt ++;                          
					if(App2.Log.Mode3TimCnt >= App2.Log.u16WfCyc){	  
						App2.Log.Mode3TimCnt = 0;                       	//刹车
						
						App2.Log.Mode3Cha = 0;
					}
				}
				
			}	
			App2.SysCtl.SpdSet = App2.Log.u32SetSpd;
			if(
				App2.Err == E_NONE 
				&& App2.SysCtl.SpdSet >= YSPara1[2]*10						//启动条件
//				&& App2.SysPort.Mbus >= (MCPara[3] / 10.0f)
				&& App2.Ch2.Status == CTLS_WAIT		
				&& YSorWSFlag ==2			
			){
				App2.Ch1.Start = 1;
			}else{
				App2.Ch1.Start = 0;
			}
		}
	}
	else if(App2.Log.Channel == 4){			//电机2
		App2.Ch1.Start = 0;	//不允许电机1启动
		//模式0：停止不给控制
		if(App2.Log.CtlMode == 0){
			App2.Ch2.Start = 0;
			App2.Log.Mode3TimCnt = 0;
			App2.Log.Mode3Cha = 0;
		}
		//模式1 CW方向
		else if(App2.Log.CtlMode == 1){
			App2.Log.Mode3TimCnt = 0;
			App2.Log.Mode3Cha = 0;
			//给方向CW
			App2.Ch2.Dir.Set = CW;
			//串口指令给转速
			App2.SysCtl.SpdSet = App2.Log.u32SetSpd;
			//判断错误
			if(
				App2.Err == E_NONE
				&& App2.SysCtl.SpdSet >= YSPara1[2]*10						//启动条件
//				&& App2.SysPort.Mbus >= (MCPara[3] / 10.0f)
				&& App2.Ch1.Status == CTLS_WAIT	
				&& YSorWSFlag ==2
			){
				App2.Ch2.Start = 1;
			}else{
				App2.Ch2.Start = 0;
			}
		}
		//模式2 CCW方向
		else if(App2.Log.CtlMode == 2){
			App2.Log.Mode3TimCnt = 0;
			App2.Log.Mode3Cha = 0;
			//给方向CCW
			App2.Ch2.Dir.Set = CCW;
			//串口指令给转速
			App2.SysCtl.SpdSet = App2.Log.u32SetSpd;
			//判断错误
			if(
				App2.Err == E_NONE 
				&& App2.SysCtl.SpdSet >= YSPara1[2]*10						//启动条件
//				&& App2.SysPort.Mbus >= (MCPara[3] / 10.0f)
				&& App2.Ch1.Status == CTLS_WAIT		
				&& YSorWSFlag ==2			
			){
				App2.Ch2.Start = 1;
			}else{
				App2.Ch2.Start = 0;
			}
		}
		//模式3 往复模式
		else if(App2.Log.CtlMode == 3){
			
			if(App2.Log.u8WfFru >= 1){
				App2.Log.u16WfCyc = 10000 / App2.Log.u8WfFru;
			}else {
				App2.Log.u16WfCyc = 1000;
			}
			
			if(App2.Log.Mode3Cha == 0){
				App2.Ch2.Dir.Set = CW;															//CW
				                                                    
				if(App2.Log.Mode31msFg == 1){                       
					App2.Log.Mode31msFg = 0;                          
					App2.Log.Mode3TimCnt ++;                          
					if(App2.Log.Mode3TimCnt >= App2.Log.u16WfCyc){    
						App2.Log.Mode3TimCnt = 0;                       	//刹车
						                                                
						App2.Log.Mode3Cha = 1;                          
					}                                                 
				}                                                   
			}else if(App2.Log.Mode3Cha == 1){                     
				App2.Ch2.Dir.Set = CCW;                             	//CCW
				                                                    
				if(App2.Log.Mode31msFg == 1){                       
					App2.Log.Mode31msFg = 0;                          
					                                                  
					App2.Log.Mode3TimCnt ++;                          
					if(App2.Log.Mode3TimCnt >= App2.Log.u16WfCyc){    
						App2.Log.Mode3TimCnt = 0;                       	//刹车
						
						App2.Log.Mode3Cha = 0;
					}
				}
				
			}
			
			App2.SysCtl.SpdSet = App2.Log.u32SetSpd;
			
			if(
				App2.Err == E_NONE
				&& App2.SysCtl.SpdSet >= YSPara1[2]*10							//启动条件
//				&& App2.SysPort.Mbus >= (MCPara[3] / 10.0f)
				&& App2.Ch1.Status == CTLS_WAIT	
				&& YSorWSFlag ==2
			){
				App2.Ch2.Start = 1;
			}else{
				App2.Ch2.Start = 0;
			}
		}
		
	}
	
	STA_Led(App2.Err);			//错误指示灯
}










