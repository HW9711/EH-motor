#include "fangbo_nohall.h"
#include "fangbo.h"
#include "hallfoc.h"
#include "tim.h"
#include "mclib.h"
#include "gpio.h"
#include "math.h"
#include "mcfoc.h"
#include "mcflash.h"
#include "adc.h"

// 5-1-3-2-6-4		CCW
 // 5-4-6-2-3-1		CW			
//换相中断		 
//5BA-1CA-3CB-2AB-6AC-4BC

//通道1无霍尔换向+启动拖动检测CW
void OpenLoop_check_changeCW(uint8_t hall_val){
//CW
	volatile uint8_t val = 0;
	if((App.FB.OpenLoop.StartHallOkCnt > 2000)&&(App.FB.OpenLoop.OpenLoopTime>MCPara[30])){
		//切入闭环初始化
		App.FB2.Rad_Now = 100;
		mcApp_VoltageP_PIParam.qdSum = App.Logic.VBusNowPWM;
		App.Logic.VBusSetPWM = App.Logic.VBusNowPWM;
		App.FB.OpenLoop.StartHallOkFg = 1;	//检测成功
		App.FB.Status = HS_RUN ;						//切闭环
		App.FB.OpenLoop.StartHallOkCnt= 0;	//清
		App.FB.OpenLoop.ClossLoopFlg = 1;		//切闭环标志
		mcApp_VoltageP_PIParam.qOutMin = (float)MCPara[35];
		mcApp_VoltageP_PIParam.qOutMax = (float)MCPara[34];
	}
	switch(hall_val){
		case phase5:		//BA--检测C相是否发生改变
					val = get_EN1Z;
					if(val == 0){
								if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
								}else{
									App.FB.OpenLoop.HallVal =4;			
									Hall_PhaseChange(4,7000);
									App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;		//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
									App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清									
								}
							App.FB.OpenLoop.En1C_Val = get_EN1Z;								
					}
					break;
		case phase4:		//BC--检测A相是否发生改变
					val = get_EN1A;
					if(val == 1){			//如果发生变化
							if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB.OpenLoop.HallVal =6;
								Hall_PhaseChange(6,7000);	
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清										
							}						
							App.FB.OpenLoop.En1A_Val = get_EN1A;
					}		
					break;
		case phase6:		//AC--检测B相是否发生改变
					val = get_EN1B;
					if(val == 0){	
							if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB.OpenLoop.HallVal =2;
								Hall_PhaseChange(2,7000);
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;		//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清								
							}
							App.FB.OpenLoop.En1B_Val = get_EN1B;
					}
					break;
		case phase2:		//AB--检测C相是否发生改变
					val = get_EN1Z;
					if(val == 1){
							if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
								
							}else{
								App.FB.OpenLoop.HallVal =3;
								Hall_PhaseChange(3,7000);
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清			
							}	
				
							App.FB.OpenLoop.En1C_Val = get_EN1Z;							
					}					
					break;
		case phase3:		//CB--检测A相是否发生改变
					val = get_EN1A;
					if(val == 0){			//如果发生变化
						if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB.OpenLoop.HallVal =1;
								Hall_PhaseChange(1,7000);
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清
							}

							App.FB.OpenLoop.En1A_Val = get_EN1A;
					}					
					break;
		case phase1:		//CA--检测B相是否发生改变
					val = get_EN1B;
					if(val == 1){	
							if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB.OpenLoop.HallVal =5;
								Hall_PhaseChange(5,7000);
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清
							}

							App.FB.OpenLoop.En1B_Val = get_EN1B;
					}
					break;
		default: break;
	
	}

}


//通道2无霍尔换向+启动拖动检测CW
void OpenLoop_check_changeCW2(uint8_t hall_val){
//CW
	volatile uint8_t val = 0;
	if((App.FB2.OpenLoop.StartHallOkCnt > 2000)&&(App.FB2.OpenLoop.OpenLoopTime>MCPara2[30])){
//if(App.FB.OpenLoop.StartHallOkCnt > 1000){		
		//切入闭环初始化
		App.FB2.Rad_Now = 100;
		mcApp_VoltageP_PIParam2.qdSum = App.Logic.VBusNowPWM;
		App.Logic.VBusSetPWM = App.Logic.VBusNowPWM;
		App.FB2.OpenLoop.StartHallOkFg = 1;	//检测成功
		App.FB2.Status = HS_RUN ;						//切闭环
		App.FB2.OpenLoop.StartHallOkCnt= 0;	//清
		App.FB2.OpenLoop.ClossLoopFlg = 1;		//切闭环标志
		mcApp_VoltageP_PIParam2.qOutMin = (float)MCPara2[35];
		mcApp_VoltageP_PIParam2.qOutMax = (float)MCPara2[34];

	}
	switch(hall_val){
		case phase5:		//BA--检测C相是否发生改变
					val = get_EN2Z;
					if(val == 0){
								if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
								}else{
									App.FB2.OpenLoop.HallVal =4;
									Hall_PhaseChange2(4,7000);
									App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;		//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
									App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清									
								}
							App.FB2.OpenLoop.En1C_Val = get_EN2Z;								
					}
					break;
		case phase4:		//BC--检测A相是否发生改变
					val = get_EN2A;
					if(val == 1){			//如果发生变化
							if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =6;
								Hall_PhaseChange2(6,7000);	
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清										
							}						
							App.FB2.OpenLoop.En1A_Val = get_EN2A;
					}		
					break;
		case phase6:		//AC--检测B相是否发生改变
					val = get_EN2B;
					if(val == 0){	
							if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =2;
								Hall_PhaseChange2(2,7000);
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;		//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清								
							}
							App.FB2.OpenLoop.En1B_Val = get_EN2B;
					}
					break;
		case phase2:		//AB--检测C相是否发生改变
					val = get_EN2Z;
					if(val == 1){
							if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
								
							}else{
								App.FB2.OpenLoop.HallVal =3;
								Hall_PhaseChange2(3,7000);
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清			
							}	
				
							App.FB2.OpenLoop.En1C_Val = get_EN2Z;							
					}					
					break;
		case phase3:		//CB--检测A相是否发生改变
					val = get_EN2A;
					if(val == 0){			//如果发生变化
						if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =1;
								Hall_PhaseChange2(1,7000);
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清
							}

							App.FB2.OpenLoop.En1A_Val = get_EN2A;
					}					
					break;
		case phase1:		//CA--检测B相是否发生改变
					val = get_EN2B;
					if(val == 1){	
							if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =5;
								Hall_PhaseChange2(5,7000);
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清
							}

							App.FB2.OpenLoop.En1B_Val = get_EN2B;
					}
					break;
		default: break;
	
	}

}

//通道1无霍尔换向+启动拖动检测CCW
void OpenLoop_check_changeCCW(uint8_t hall_val){
//CW
	volatile uint8_t val = 0;
	if((App.FB.OpenLoop.StartHallOkCnt > 2000)&&(App.FB.OpenLoop.OpenLoopTime>MCPara[30])){
		//切入闭环初始化
		App.FB.Rad_Now = -100;
		mcApp_VoltageP_PIParam.qdSum = App.Logic.VBusNowPWM;
		App.Logic.VBusSetPWM = App.Logic.VBusNowPWM;
		App.FB.OpenLoop.StartHallOkFg = 1;	//检测成功
		App.FB.Status = HS_RUN ;						//切闭环
		App.FB.OpenLoop.StartHallOkCnt= 0;	//清
		App.FB.OpenLoop.ClossLoopFlg = 1;		//切闭环标志
		mcApp_VoltageP_PIParam.qOutMin = (float)MCPara[35];
		mcApp_VoltageP_PIParam.qOutMax = (float)MCPara[34];
	}		//5-4-6-2-3-1		->
	switch(hall_val){
		case phase5:		//BA--检测C相是否发生改变			
					val = get_EN1Z;
					if(val == 0){
								if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
								}else{
									App.FB.OpenLoop.HallVal =1;			
									Hall_PhaseChange(1,7000);
									App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;		//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
									App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清									
								}
							App.FB.OpenLoop.En1C_Val = get_EN1Z;								
					}
					break;
		case phase4:		//BC--检测A相是否发生改变
					val = get_EN1A;
					if(val == 1){			//如果发生变化
							if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB.OpenLoop.HallVal =5;
								Hall_PhaseChange(5,7000);	
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清										
							}						
							App.FB.OpenLoop.En1A_Val = get_EN1A;
					}		
					break;
		case phase6:		//AC--检测B相是否发生改变
					val = get_EN1B;
					if(val == 0){	
							if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB.OpenLoop.HallVal =4;
								Hall_PhaseChange(4,7000);
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;		//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清								
							}

							App.FB.OpenLoop.En1B_Val = get_EN1B;
					}
					break;
		case phase2:		//AB--检测C相是否发生改变
					val = get_EN1Z;
					if(val == 1){
							if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
								
							}else{
								App.FB.OpenLoop.HallVal =6;
								Hall_PhaseChange(6,7000);
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清			
							}	
				
							App.FB.OpenLoop.En1C_Val = get_EN1Z;							
					}					
					break;
		case phase3:		//CB--检测A相是否发生改变
					val = get_EN1A;
					if(val == 0){			//如果发生变化
						if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB.OpenLoop.HallVal =2;
								Hall_PhaseChange(2,7000);
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清
							}
							App.FB.OpenLoop.En1A_Val = get_EN1A;
					}					
					break;
		case phase1:		//CA--检测B相是否发生改变
					val = get_EN1B;
					if(val == 1){	
							if(App.FB.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB.OpenLoop.HallVal =3;
								Hall_PhaseChange(3,7000);
								App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清
							}

							App.FB.OpenLoop.En1B_Val = get_EN1B;
					}
					break;
		default: break;
	
	}

}


//通道2无霍尔换向+启动拖动检测CCW
void OpenLoop_check_changeCCW2(uint8_t hall_val){
//CW
	volatile uint8_t val = 0;
	if((App.FB2.OpenLoop.StartHallOkCnt > 2000)&&(App.FB2.OpenLoop.OpenLoopTime>MCPara2[30])){//if(App.FB.OpenLoop.StartHallOkCnt > 1000){
		//切入闭环初始化
		App.FB2.Rad_Now = -100;
		mcApp_VoltageP_PIParam2.qdSum = App.Logic.VBusNowPWM;
		App.Logic.VBusSetPWM = App.Logic.VBusNowPWM;
		//切入标志
		App.FB2.OpenLoop.StartHallOkFg = 1;	//检测成功
		App.FB2.Status = HS_RUN ;						//切闭环
		App.FB2.OpenLoop.StartHallOkCnt= 0;	//清
		App.FB2.OpenLoop.ClossLoopFlg = 1;		//切闭环标志
		mcApp_VoltageP_PIParam2.qOutMin = (float)MCPara2[35];
		mcApp_VoltageP_PIParam2.qOutMax = (float)MCPara2[34];
	}		//5-4-6-2-3-1		->
	switch(hall_val){
		case phase5:		//BA--检测C相是否发生改变			
					val = get_EN2Z;
					if(val == 0){
								if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
								}else{
									App.FB2.OpenLoop.HallVal =1;			
									Hall_PhaseChange2(1,7000);
									App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;		//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
									App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清									
								}
							App.FB2.OpenLoop.En1C_Val = get_EN2Z;								
					}
					break;
		case phase4:		//BC--检测A相是否发生改变
					val = get_EN2A;
					if(val == 1){			//如果发生变化
							if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =5;
								Hall_PhaseChange2(5,7000);	
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清										
							}						
							App.FB2.OpenLoop.En1A_Val = get_EN2A;
					}		
					break;
		case phase6:		//AC--检测B相是否发生改变
					val = get_EN2B;
					if(val == 0){	
							if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =4;
								Hall_PhaseChange2(4,7000);
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;		//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清								
							}

							App.FB2.OpenLoop.En1B_Val = get_EN2B;
					}
					break;
		case phase2:		//AB--检测C相是否发生改变
					val = get_EN2Z;
					if(val == 1){
							if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =6;
								Hall_PhaseChange2(6,7000);
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清			
							}	
							App.FB2.OpenLoop.En1C_Val = get_EN2Z;							
					}					
					break;
		case phase3:		//CB--检测A相是否发生改变
					val = get_EN2A;
					if(val == 0){			//如果发生变化
						if(App.FB2.OpenLoop.StartHallOkFg == 0){
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =2;
								Hall_PhaseChange2(2,7000);
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清
							}

							App.FB2.OpenLoop.En1A_Val = get_EN2A;
					}					
					break;
		case phase1:		//CA--检测B相是否发生改变
					val = get_EN2B;
					if(val == 1){	
							if(App.FB2.OpenLoop.StartHallOkFg == 0){		//如果还没有检测成功，那么就更新检测数
								App.FB2.OpenLoop.StartHallOkCnt++;
							}else{
								App.FB2.OpenLoop.HallVal =3;
								Hall_PhaseChange2(3,7000);
								App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
								App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清
							}

							App.FB2.OpenLoop.En1B_Val = get_EN2B;
					}
					break;
		default: break;
	
	}

}



//无霍尔闭环换相状态机
void OpenLoop_change(uint8_t hall_val){
	
	switch(hall_val){
		case phase1:
			
					break;
		case phase2:
			
					break;
		case phase3:
			
					break;
		case phase4:
			
					break;
		case phase5:
			
					break;
		case phase6:
			
					break;
		default: break;
	
	}
	
}




