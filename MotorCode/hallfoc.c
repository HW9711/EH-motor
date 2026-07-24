#include "hallfoc.h"
#include "tim.h"
#include "mclib.h"
#include "gpio.h"
#include "math.h"
#include "mcfoc.h"
#include "mcflash.h"
#include "enocfoc.h"

//                      20 - 40
//             Q轴偏移  相角度偏移   hall5对应的角度偏移  hall中间角度
#define Offset (90.0f   + (30.0f)      + 30.0f              -30.0f)       * ANGLE_2PI / 360.0f
#define hall0offset(Q)  ( (Q < 0) ? (Q + ANGLE_2PI) : (Q) )

//CW 513264
float hall_mid_val[8] = {
	-10,    //0
	hall0offset(1.047f * 1 - Offset),  //1
	hall0offset(1.047f * 3 - Offset),//2
	hall0offset(1.047f * 2 - Offset),//3
	hall0offset(1.047f * 5 - Offset),//4
	hall0offset(1.047f * 0 - Offset),//5
	hall0offset(1.047f * 4 - Offset),//6
	-70,//7
};







//根据hall切分角度
void hall_cal(void){
	uint8_t hallval;
	float hall_diff = 0;
	float rpm_abs = 0;
	
	hallval = gethall;//读hall
	
	rpm_abs = myabs((App.M1.CalSpd.NowRad * 60.0f/((2.0f * M_PI )*MCPara[1]))); 
	
	if((hallval == 0 || hallval == 7) && App.M1.AllowRun == 1){
		App.M1.Prot.HallErrCnt ++;
		if(App.M1.Prot.HallErrCnt >= 20){
			App.M1.Err = E_NOHALL;
			App.BitErr |= ERR_GD_Hall_Loss;//供弹电机霍尔错误
		}
	}else {
		App.M1.Prot.HallErrCnt --;
		if(App.M1.Prot.HallErrCnt < 0){
			App.M1.Prot.HallErrCnt = 0;
		}
	}
	
	//App.M1.Hall.ReadMidAng = hall_mid_val[ hallval ];//取角度
	App.M1.Hall.ReadMidAng = App.M1.Hall.SdSaveAng[ hallval ];
	App.M1.Hall.MidAng = App.M1.Hall.ReadMidAng;//记录此刻hall的角度
	
	if(App.M1.Hall.ReadMidAng > 0 && App.M1.Hpll.HallStadySta == 3){//读到的hall值是正确的  && App.M1.AllowRun == 1
		
		if(App.M1.Hall.ReadMidAngPrev < 0){//启动/之前读取的hall为 0 或者 7
			App.M1.Hall.ReadMidAngPrev = App.M1.Hall.ReadMidAng;
			App.M1.Hall.Ang = App.M1.Hall.ReadMidAng;//更新当前角度为当前hall中间值
		}else if(App.M1.Hall.ReadMidAng != App.M1.Hall.ReadMidAngPrev){//hall发生了变化
			
			App.M1.Hall.AngAvg = myabs((App.M1.Hall.ReadMidAng - App.M1.Hall.ReadMidAngPrev));//当前hall角度与上次的hall角度的中间值，就是当前的角度
			if(App.M1.Hall.AngAvg < M_PI){
				App.M1.Hall.AngAvg = (App.M1.Hall.ReadMidAng + App.M1.Hall.ReadMidAngPrev) / 2;
			}else if(App.M1.Hall.AngAvg != M_PI){
				App.M1.Hall.AngAvg = (App.M1.Hall.ReadMidAng + App.M1.Hall.ReadMidAngPrev) / 2 + M_PI;
			}
			
			while(App.M1.Hall.AngAvg > ANGLE_2PI){//规范角度
				App.M1.Hall.AngAvg -= ANGLE_2PI;
			}
			while(App.M1.Hall.AngAvg < 0) {//规范角度
				App.M1.Hall.AngAvg += ANGLE_2PI;
			}
			App.M1.Hall.Ang = App.M1.Hall.AngAvg;//赋值角度
		}
			App.M1.Hall.ReadMidAngPrev = App.M1.Hall.ReadMidAng;//上一相角度赋值为当前角度 用于判定hall是否变化了
		
		if(rpm_abs < MCPara[59]){//如果当前转速小于 N，那么hall直接等于当前角度，不进行累加了，相当于方波
			App.M1.Hall.Ang = App.M1.Hall.ReadMidAng;
		}else {
			hall_diff = App.M1.Hall.Ang - App.M1.Hall.MidAng;//计算当前累加角度是否超过了角度的范围
			
			while (hall_diff > M_PI) {//得到真正的角度差值
				hall_diff -= ANGLE_2PI;
			}
			while (hall_diff < -M_PI) {
				hall_diff += ANGLE_2PI;
			}
			
			if( myabs(hall_diff) < (ANGLE_2PI / 12.0f) ){//正确的					//在15°以内的误差是对的
				App.M1.Hall.Ang += App.M1.CalSpd.NowRad * App.M1.CalSpd.Dt;
			}else {
				App.M1.Hall.Ang -= hall_diff / 100.0f;//离差值太远了
			}
		}
		while (App.M1.Hall.Ang >  ANGLE_2PI) {
			App.M1.Hall.Ang -= ANGLE_2PI;
		}
		while (App.M1.Hall.Ang < 0) {
			App.M1.Hall.Ang += ANGLE_2PI;
		}
	}else {//角度为错误的
		App.M1.Hall.ReadMidAngPrev = -1;
	}
}






//根据hall切分角度
void hall_cal2(void){
	uint8_t hallval;
	float hall_diff = 0;
	float rpm_abs  = 0;
	
	hallval = gethall2;//读hall
	rpm_abs = myabs((App.M2.CalSpd.NowRad * 60.0f / ((2.0f * M_PI)  * MCPara2[1] )));//计算速度 取绝对值
	if((hallval == 0 || hallval == 7) && App.M2.AllowRun == 1){
		App.M2.Prot.HallErrCnt ++;
		if(App.M2.Prot.HallErrCnt >= 20){
			App.M2.Err = E_NOHALL;
			App.BitErr |= ERR_WL_Hall_Loss;//威力电机霍尔错误
		}
	}else {
		App.M2.Prot.HallErrCnt --;
		if(App.M2.Prot.HallErrCnt < 0){
			App.M2.Prot.HallErrCnt = 0;
		}
	}
 
	//App.M2.Hall.ReadMidAng = hall_mid_val[ hallval ];//取角度
	App.M2.Hall.ReadMidAng = App.M2.Hall.SdSaveAng[ hallval ];
	App.M2.Hall.MidAng = App.M2.Hall.ReadMidAng;//记录此刻hall的角度
	if(App.M2.Hall.ReadMidAng > 0 && App.M2.Hpll.HallStadySta == 3){//读到的hall值是正确的  && App.M2.AllowRun == 1
		if(App.M2.Hall.ReadMidAngPrev < 0){//启动/之前读取的hall为 0 或者 7
			App.M2.Hall.ReadMidAngPrev = App.M2.Hall.ReadMidAng;
			App.M2.Hall.Ang = App.M2.Hall.ReadMidAng;//更新当前角度为当前hall中间值
		}
		else if(App.M2.Hall.ReadMidAng != App.M2.Hall.ReadMidAngPrev){//hall发生了变化
			App.M2.Hall.Hall_Change_Flag =1;
			App.M2.Hall.AngAvg = myabs((App.M2.Hall.ReadMidAng - App.M2.Hall.ReadMidAngPrev));//当前hall角度与上次的hall角度的中间值，就是当前的角度
			if(App.M2.Hall.AngAvg < M_PI){
				App.M2.Hall.AngAvg = (App.M2.Hall.ReadMidAng + App.M2.Hall.ReadMidAngPrev) / 2;
			}else if(App.M2.Hall.AngAvg != M_PI){
				App.M2.Hall.AngAvg = (App.M2.Hall.ReadMidAng + App.M2.Hall.ReadMidAngPrev) / 2 + M_PI;
			}
			while(App.M2.Hall.AngAvg > ANGLE_2PI){//规范角度
				App.M2.Hall.AngAvg -= ANGLE_2PI;
			}
			while(App.M2.Hall.AngAvg < 0) {//规范角度
				App.M2.Hall.AngAvg += ANGLE_2PI;
			}
			App.M2.Hall.Ang = App.M2.Hall.AngAvg;//赋值角度
		}
			App.M2.Hall.ReadMidAngPrev = App.M2.Hall.ReadMidAng;//上一相角度赋值为当前角度 用于判定hall是否变化了
		
		if(rpm_abs < MCPara2[59]){//如果当前转速小于 N，那么hall直接等于当前角度，不进行累加了，相当于方波
			App.M2.Hall.Ang = App.M2.Hall.ReadMidAng;
		}else {
			hall_diff = App.M2.Hall.Ang - App.M2.Hall.MidAng;//计算当前累加角度是否超过了角度的范围
			
			while (hall_diff > M_PI) {//得到真正的角度差值
				hall_diff -= ANGLE_2PI;
			}
			while (hall_diff < -M_PI) {
				hall_diff += ANGLE_2PI;
			}
			
			if( myabs(hall_diff) < (ANGLE_2PI / 12.0f) ){//正确的					//在15°以内的误差是对的
				App.M2.Hall.Ang += App.M2.CalSpd.NowRad * App.M2.CalSpd.Dt;
			}else {
				App.M2.Hall.Ang -= hall_diff / 100.0f;//离差值太远了
			}
		}
		while (App.M2.Hall.Ang >  ANGLE_2PI) {
			App.M2.Hall.Ang -= ANGLE_2PI;
		}
		while (App.M2.Hall.Ang < 0) {
			App.M2.Hall.Ang += ANGLE_2PI;
		}
	}else {//角度为错误的
		App.M2.Hall.ReadMidAngPrev = -1;
	}
}





//开闭环 电流控制
void Hall_FOC_DoControl(void){
		float DoControl_Temp1, DoControl_Temp2;
    if(App.M1.OpenLoop.OpenLoop == 1){                                                 //开环运行
        if(App.M1.OpenLoop.ChangeMode == 1){//执行开环的一些量的初始化
            App.M1.OpenLoop.ChangeMode = 0;//只执行一次
					
            mcApp_ControlParam.IqRef = 0;
            mcApp_ControlParam.IdRef = 0;
						
						//开环角度参数初始化
		       	App.M1.OpenLoop.LocateTimCnt = 0;
		      	App.M1.OpenLoop.RampAngleRadsPerSec = 0;
						App.M1.OpenLoop.Ang = 0;
        }
				
				//开环给定Q值
				mcApp_ControlParam.IqRef = App.M1.OpenLoop.RampupCurQ;
        mcApp_ControlParam.IdRef = App.M1.OpenLoop.RampupCurD;
				
        // PI control for Q
        mcApp_Q_PIParam.qInMeas = mcApp_I_DQParam.q;
        mcApp_Q_PIParam.qInRef  = mcApp_ControlParam.IqRef;
        mcLib_CalcPI(&mcApp_Q_PIParam);
        mcApp_V_DQParam.q = mcApp_Q_PIParam.qOut;
        // PI control for D
        mcApp_D_PIParam.qInMeas = mcApp_I_DQParam.d;
        mcApp_D_PIParam.qInRef  = mcApp_ControlParam.IdRef;
        mcLib_CalcPI(&mcApp_D_PIParam);
        mcApp_V_DQParam.d = mcApp_D_PIParam.qOut;
    }else{//闭环控制
	    	if(App.M1.OpenLoop.ChangeMode == 1){//闭环量初始化
            App.M1.OpenLoop.ChangeMode = 0;
						
						mcApp_Speed_PIParam.qdSum = 0;
						mcApp_Speed_PIParam.qOut = 0;
						
						mcApp_D_PIParam.qdSum = 0;
						mcApp_D_PIParam.qOut = 0;
						
						mcApp_Q_PIParam.qdSum = 0;
						mcApp_Q_PIParam.qOut = 0;
						
						mcApp_V_DQParam.q = 0;
						mcApp_V_DQParam.d = 0;
					
						mcApp_ControlParam.IqRef = 0;
				
						mcApp_ControlParam.IdRef = 2.0f;
					
						mcApp_ControlParam.VelRef = App.M1.CalSpd.NowRad;

						App.M1.Spd.RadSet = ((App.M1.Spd.Set / 60.0f) * (2.0f*M_PI)) * MCPara[1];
        }
				
				Id_Ctl();
				Spd_PI_Out();
				
        mcApp_D_PIParam.qInMeas = mcApp_I_DQParam.d;          // This is in Amps
        mcApp_D_PIParam.qInRef  = mcApp_ControlParam.IdRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_D_PIParam);
        mcApp_V_DQParam.d    =  mcApp_D_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)
				
        DoControl_Temp2 = mcApp_D_PIParam.qOut * mcApp_D_PIParam.qOut;
        DoControl_Temp1 = sqrtMax - DoControl_Temp2;
        mcApp_Q_PIParam.qOutMax = sqrt(DoControl_Temp1);        
		
        mcApp_Q_PIParam.qInMeas = mcApp_I_DQParam.q;          // This is in Amps
        mcApp_Q_PIParam.qInRef  = mcApp_ControlParam.IqRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_Q_PIParam);
        mcApp_V_DQParam.q    = mcApp_Q_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)       
    }

}



//开闭环 电流控制
void Hall_FOC_DoControl2(void){
		float DoControl_Temp1, DoControl_Temp2;
    if(App.M2.OpenLoop.OpenLoop == 1){                                                 //开环运行
        if(App.M2.OpenLoop.ChangeMode == 1){//执行开环的一些量的初始化
            App.M2.OpenLoop.ChangeMode = 0;//只执行一次
					
            mcApp_ControlParam2.IqRef = 0;
            mcApp_ControlParam2.IdRef = 0;
						
						//开环角度参数初始化
		       	App.M2.OpenLoop.LocateTimCnt = 0;
		      	App.M2.OpenLoop.RampAngleRadsPerSec = 0;
						App.M2.OpenLoop.Ang = 0;
        }
				
				//开环给定Q值
				mcApp_ControlParam2.IqRef = App.M2.OpenLoop.RampupCurQ;
        mcApp_ControlParam2.IdRef = App.M2.OpenLoop.RampupCurD;
				
        // PI control for Q
        mcApp_Q2_PIParam.qInMeas = mcApp_I2_DQParam.q;
        mcApp_Q2_PIParam.qInRef  = mcApp_ControlParam2.IqRef;
        mcLib_CalcPI(&mcApp_Q2_PIParam);
        mcApp_V2_DQParam.q = mcApp_Q2_PIParam.qOut;
        // PI control for D
        mcApp_D2_PIParam.qInMeas = mcApp_I2_DQParam.d;
        mcApp_D2_PIParam.qInRef  = mcApp_ControlParam2.IdRef;
        mcLib_CalcPI(&mcApp_D2_PIParam);
        mcApp_V2_DQParam.d = mcApp_D2_PIParam.qOut;
    }else{//闭环控制
	    	if(App.M2.OpenLoop.ChangeMode == 1){//闭环量初始化
            App.M2.OpenLoop.ChangeMode = 0;
						
						mcApp_Speed2_PIParam.qdSum = 0;
						mcApp_Speed2_PIParam.qOut = 0;
						
						mcApp_D2_PIParam.qdSum = 0;
						mcApp_D2_PIParam.qOut = 0;
						
						mcApp_Q2_PIParam.qdSum = 0;
						mcApp_Q2_PIParam.qOut = 0;
						
						mcApp_V2_DQParam.q = 0;
						mcApp_V2_DQParam.d = 0;
					
						mcApp_ControlParam2.IqRef = 0;
            mcApp_ControlParam2.IdRef = 0;
					
						mcApp_ControlParam2.VelRef = App.M2.CalSpd.NowRad;			//参考速度初始化！也就是现在目前的速度。

						App.M2.Spd.RadSet = ((App.M2.Spd.Set / 60.0f) * (2.0f*M_PI)) * MCPara2[1];
        }
				
				Id_Ctl2();
				Spd_PI_Out2();
				
        mcApp_D2_PIParam.qInMeas = mcApp_I2_DQParam.d;          // This is in Amps
		
        mcApp_D2_PIParam.qInRef  = mcApp_ControlParam2.IdRef;      // This is in Amps

        mcLib_CalcPI(&mcApp_D2_PIParam);
        mcApp_V2_DQParam.d    =  mcApp_D2_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)
				
        DoControl_Temp2 = mcApp_D2_PIParam.qOut * mcApp_D2_PIParam.qOut;
        DoControl_Temp1 = sqrtMax - DoControl_Temp2;
        mcApp_Q2_PIParam.qOutMax = sqrt(DoControl_Temp1);        
		
        mcApp_Q2_PIParam.qInMeas = mcApp_I2_DQParam.q;          // This is in Amps
        mcApp_Q2_PIParam.qInRef  = mcApp_ControlParam2.IqRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_Q2_PIParam);
        mcApp_V2_DQParam.q    = mcApp_Q2_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)       
    }

}












//开环定位和角度累加
//闭环初始角度偏移
void Hall_FOC_CalculateParkAngle(void){
	if(App.M1.OpenLoop.OpenLoop == 1){                                                 //开环运行
		if(App.M1.OpenLoop.LocateTimCnt < App.M1.OpenLoop.LocateTimSum){                 //启动前定位 时间 
			App.M1.OpenLoop.LocateTimCnt ++;
		}else if (App.M1.OpenLoop.RampAngleRadsPerSec < App.M1.OpenLoop.RampupRad2SecLooptime){//定位完成，在设定时间内加速到设定速度
			App.M1.OpenLoop.RampAngleRadsPerSec += App.M1.OpenLoop.RampupInc;
		}else{                                                                          //跳转到闭环
			//hall自学习
			Hall_Stady();
		}
		App.M1.OpenLoop.Ang += App.M1.OpenLoop.RampAngleRadsPerSec;
    if(App.M1.OpenLoop.Ang > ANGLE_2PI){App.M1.OpenLoop.Ang = App.M1.OpenLoop.Ang - ANGLE_2PI;}
  }
}


//开环定位和角度累加
//闭环初始角度偏移
void Hall_FOC_CalculateParkAngle2(void){
	if(App.M2.OpenLoop.OpenLoop == 1){                                                 //开环运行
		if(App.M2.OpenLoop.LocateTimCnt < App.M2.OpenLoop.LocateTimSum){                 //启动前定位 时间 
			App.M2.OpenLoop.LocateTimCnt ++;
		}else if (App.M2.OpenLoop.RampAngleRadsPerSec < App.M2.OpenLoop.RampupRad2SecLooptime){//定位完成，在设定时间内加速到设定速度
			App.M2.OpenLoop.RampAngleRadsPerSec += App.M2.OpenLoop.RampupInc;
		}else{                                                                          //跳转到闭环
			//hall自学习
			Hall_Stady2();
		}
		App.M2.OpenLoop.Ang += App.M2.OpenLoop.RampAngleRadsPerSec;
    if(App.M2.OpenLoop.Ang > ANGLE_2PI){
		App.M2.OpenLoop.Ang = App.M2.OpenLoop.Ang - ANGLE_2PI;
	}
  }
}


void Hall_Stady(void){
	//hall 位置记录
	App.M1.Hall.Val = gethall;
		if(App.M1.Hall.OldVal != App.M1.Hall.Val ){
			//记录角度
			if(App.M1.Hall.SdSunXuCnt >= 6){//电机电角度已经转完一圈
				//记录当前角度
				App.M1.Hall.SdRecAng[App.M1.Hall.Val] = App.M1.OpenLoop.Ang;//( App.M1.Hall.SdRecAng[App.M1.Hall.Val] + App.M1.OpenLoop.Ang ) / 2.0f;// / 2

				//算出Hall的中间值
				App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] = myabs((App.M1.Hall.SdRecAng[App.M1.Hall.Val] - App.M1.Hall.SdRecAng[App.M1.Hall.OldVal]));
				if(App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] < M_PI){
					App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] = (App.M1.Hall.SdRecAng[App.M1.Hall.Val] + App.M1.Hall.SdRecAng[App.M1.Hall.OldVal]) / 2.0f;//(App.M1.Hall.SdRecAng[App.M1.Hall.Val] + App.M1.Hall.SdRecAng[App.M1.Hall.OldVal]) / 2;
				}else if(App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] != M_PI){
					App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] = (App.M1.Hall.SdRecAng[App.M1.Hall.Val] + App.M1.Hall.SdRecAng[App.M1.Hall.OldVal]) / 2.0f + M_PI;//(App.M1.Hall.SdRecAng[App.M1.Hall.Val] + App.M1.Hall.SdRecAng[App.M1.Hall.OldVal]) / 2 + M_PI;
				}
				while(App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] > ANGLE_2PI){//规范角度
					App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] -= ANGLE_2PI;
				}
				while(App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] < 0) {//规范角度
					App.M1.Hall.SdSaveAng[App.M1.Hall.OldVal] += ANGLE_2PI;
				}
				
			}else {//电机电角度未转完一圈
				App.M1.Hall.SdRecAng[App.M1.Hall.Val] = App.M1.OpenLoop.Ang;//当前hall值赋值为开环角度
			}

			App.M1.Hall.SdSunXuCnt ++;
			
			if(App.M1.Hall.SdSunXuCnt >= MCPara[56]){//hall角度学习次数
				App.M1.Hall.SdSunXuCnt = 0;
				
				App.M1.Hpll.HallStadySta = 2;//已经开环转动完成
				App.M1.AllowRun = 0;
				App.M1.Hpll.HallAllRun = 0;//不允许启动，等待保存完成
				TIME_PWM_Stop_3Channel(TMR1);
				App.M1.Status = CTLS_STOP;//直接停止
				App.M1.OpenLoop.OpenLoop = 0;//转到闭环
			}

			App.M1.Hall.OldVal = App.M1.Hall.Val;
			
			//错误了如何处理
			if(App.M1.Err != E_NONE){
				TIME_PWM_Stop_3Channel(TMR1);
				App.M1.OpenLoop.OpenLoop = 0;//转到闭环
				
				
				App.M1.Hall.SdSunXuCnt = 0;
				App.M1.Hpll.HallAllRun = 0;
				App.M1.Hpll.HallStadySta = 0;
				
			}
			
		}
	
	
}


void Hall_Stady2(void){
	//hall 位置记录
	App.M2.Hall.Val = gethall2;
	
	if(App.M2.Hall.OldVal != App.M2.Hall.Val){
		//记录角度
		if(App.M2.Hall.SdSunXuCnt >= 6){//电机电角度已经转完一圈
			//记录当前角度
			App.M2.Hall.SdRecAng[App.M2.Hall.Val] = App.M2.OpenLoop.Ang;//( App.M2.Hall.SdRecAng[App.M2.Hall.Val] + App.M2.OpenLoop.Ang ) / 2.0f;// / 2

			//算出Hall的中间值
			App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] = myabs((App.M2.Hall.SdRecAng[App.M2.Hall.Val] - App.M2.Hall.SdRecAng[App.M2.Hall.OldVal]));
			if(App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] < M_PI){
				App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] = (App.M2.Hall.SdRecAng[App.M2.Hall.Val] + App.M2.Hall.SdRecAng[App.M2.Hall.OldVal]) / 2.0f;//(App.M2.Hall.SdRecAng[App.M2.Hall.Val] + App.M2.Hall.SdRecAng[App.M2.Hall.OldVal]) / 2;
			}else if(App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] != M_PI){
				App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] = (App.M2.Hall.SdRecAng[App.M2.Hall.Val] + App.M2.Hall.SdRecAng[App.M2.Hall.OldVal]) / 2.0f + M_PI;//(App.M2.Hall.SdRecAng[App.M2.Hall.Val] + App.M2.Hall.SdRecAng[App.M2.Hall.OldVal]) / 2 + M_PI;
			}
			while(App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] > ANGLE_2PI){//规范角度
				App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] -= ANGLE_2PI;
			}
			while(App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] < 0) {//规范角度
				App.M2.Hall.SdSaveAng[App.M2.Hall.OldVal] += ANGLE_2PI;
			}
			
		}else {//电机电角度未转完一圈
			App.M2.Hall.SdRecAng[App.M2.Hall.Val] = App.M2.OpenLoop.Ang;//当前hall值赋值为开环角度
		}

		App.M2.Hall.SdSunXuCnt ++;
		
		if(App.M2.Hall.SdSunXuCnt >= MCPara2[56]){//hall角度学习次数
			App.M2.Hall.SdSunXuCnt = 0;
			
			App.M2.Hpll.HallStadySta = 2;//已经开环转动完成
			App.M2.AllowRun = 0;
			App.M2.Hpll.HallAllRun = 0;//不允许启动，等待保存完成
			TIME_PWM_Stop_3Channel(TMR8);
			App.M2.Status = CTLS_STOP;//直接停止
			App.M2.OpenLoop.OpenLoop = 0;//转到闭环
		}

		App.M2.Hall.OldVal = App.M2.Hall.Val;
		
		//错误了如何处理
		if(App.M2.Err != E_NONE){
			TIME_PWM_Stop_3Channel(TMR8);
			App.M2.OpenLoop.OpenLoop = 0;//转到闭环
			
			App.M2.Hall.SdSunXuCnt = 0;
			App.M2.Hpll.HallAllRun = 0;
			App.M2.Hpll.HallStadySta = 0;
		}
	}
}














float angoff = -10;//学习完了之后，偏移3个角度

void HallStady_Check(void){
	float hallerr[8] = {0};
	uint8_t i = 0;
	uint8_t j = 0;
	float temp = 0;
	
	if(App.M1.AllowRun == 0){
		
		if(App.M1.Hpll.HallStadySta == 0){//还未学习过
			
			App.M1.Hpll.HallAllRun = 0;//还未学习，不允许启动
			
			if(App.M1.Key.StadyFg == 1){//按键指令允许学习
				App.M1.Hpll.HallStadySta = 1;
			}
			
		}else if(App.M1.Hpll.HallStadySta == 1){//正在进行开环转动
			App.M1.Hpll.HallAllRun = 2;//允许启动去学习
		}else if(App.M1.Hpll.HallStadySta == 2){//已经开环转动过，现在需要判定学习是否正确
			App.M1.Hpll.HallAllRun = 0;//不允许启动
			
			
			
			//对数据取余 得出范围 0-1047
				for(i = 1;i < 7;i ++){
					//对数据进行一定的偏移
					App.M1.Hall.SdSaveAng[i] = App.M1.Hall.SdSaveAng[i] - (angoff * ANGLE_2PI / 360.0f);//偏移一定的角度
					if(App.M1.Hall.SdSaveAng[i] < 0){
						App.M1.Hall.SdSaveAng[i] += ANGLE_2PI;
					}else if(App.M1.Hall.SdSaveAng[i] > ANGLE_2PI){
						App.M1.Hall.SdSaveAng[i] -= ANGLE_2PI;
					}
					
					//转为定点数运算
					App.M1.Hall.SdSaveAng_Q15[i] = App.M1.Hall.SdSaveAng[i] * 1000.0f;
					//取余
					App.M1.Hall.SdAngQuYu[i] = App.M1.Hall.SdSaveAng_Q15[i] % 1047;//对学习的角度进行取余 ，求平均值  1047 是6.28 * 1000 / 6
				}
				
				//算出是否有交叉的取余值
				App.M1.Hall.SdJiaoChaFg = 0;
				
				for(i = 1; i < 7 ; i ++){
					for(j = (i + 1);j < 7; j ++){
						if( myabs(App.M1.Hall.SdAngQuYu[i] - App.M1.Hall.SdAngQuYu[j]) > 523){
							App.M1.Hall.SdJiaoChaFg = 1;//有交叉数据
							break;
						}
					}
				}
				
				
				App.M1.Hall.SdAngQuYuPJ = 0;//清除平均数
				//求平均数
				if(App.M1.Hall.SdJiaoChaFg == 0){//数据无交叉
					for(i = 1; i < 7 ; i ++){
						//计数取余之后的平均数
						App.M1.Hall.SdAngQuYuPJ += App.M1.Hall.SdAngQuYu[i];
					}
				}else {//数据有交叉 也就是 1 2 3 1046 1045 1044
					for(i = 1; i < 7 ; i ++){
						if(App.M1.Hall.SdAngQuYu[i] > 523){//1047 / 2
							App.M1.Hall.SdAngQuYu[i] = App.M1.Hall.SdAngQuYu[i] - 1047;
						}
						//计数取余之后的平均数
						App.M1.Hall.SdAngQuYuPJ += App.M1.Hall.SdAngQuYu[i];
					}
				}
				
				//求取余之后的平均值
				App.M1.Hall.SdAngQuYuPJ = App.M1.Hall.SdAngQuYuPJ / 6;
				
				//算出取余值和平均取余值的差值
				App.M1.Hall.SdAngQuYu_AllPJ = 0;
				for(i = 1;i < 7;i ++){
					App.M1.Hall.SdAngQuYu_PJ[i] = App.M1.Hall.SdAngQuYu[i] - App.M1.Hall.SdAngQuYuPJ;
					App.M1.Hall.SdAngQuYu_AllPJ += App.M1.Hall.SdAngQuYu_PJ[i];
				}
				
//				if(myabs(App.M1.Hall.SdAngQuYu_AllPJ) > 2){
//					App.M1.Err = E_SDAHALL;
//				}
				
				//算出均匀划分的学习角度
				for(i = 1;i < 7;i ++){
					temp = App.M1.Hall.SdSaveAng_Q15[i] - App.M1.Hall.SdAngQuYu_PJ[i];
					
					while (temp >  6283) {
						temp -= 6283;
					}
					while (temp < 0) {
						temp += 6283;
					}
					
					App.M1.Hall.SdSaveAng_Q15[i] = temp;
				}
				
				//更新平滑之后的角度
				for(i = 1;i < 7;i ++){
					App.M1.Hall.SdSaveAng[i] = App.M1.Hall.SdSaveAng_Q15[i] / 1000.0f;
				}
				
				for(i = 1;i < 7;i ++){
					hallerr[i] = App.M1.Hall.SdSaveAng[i] * 1000.0f;
				}
				hallerr[7] = 65535;//冒泡排序法

				//冒泡排序法
				for(i = 0;i < 7;i ++){//控制比较轮次，一共 n-1 趟
					for(j = 0; j < 7 - i ; j ++){//控制两个挨着的元素进行比较
						if(hallerr[j] > hallerr[j+1]){
							temp = hallerr[j];
							hallerr[j] = hallerr[j+1];
							hallerr[j+1] = temp;
						}
					}
				}

				for(i = 2;i < 7;i ++){
					if(hallerr[i] - hallerr[i - 1] < MCPara[57]){
						App.M1.Err = E_SDAHALL;
						App.BitErr |= ERR_GD_Angle_Learn;//供弹电机学习错误
					}
				}
				
				if(App.M1.Err == E_NONE){
					//保存hall学习之后的角度
					FLASH_Write(TEST_FLASH_ADDRESS_START_HALL,App.M1.Hall.SdSaveAng_Q15,8);
				}else {
					App.M1.Hpll.HallStadySta = 0;//学习失败
				}
				
				if(App.M1.Err != E_SAVE){
					App.M1.Hpll.HallStadySta = 4;//学习完成
				}else {
					App.M1.Hpll.HallStadySta = 0;//学习失败
				}
				
			App.M1.Hall.SdSaveAng[0] = -1;
			App.M1.Hall.SdSaveAng[7] = -1;
		}else if(App.M1.Hpll.HallStadySta == 3){//已经学习完成
			App.M1.Hpll.HallAllRun = 1;//允许正常去启动
			if(App.M1.Key.StadyFg == 1){//如果重新有指令去学习，那么需要重新学习
				App.M1.Hpll.HallStadySta = 1;//去开环拖
			}
		}else if(App.M1.Hpll.HallStadySta == 4){//已经学习完成
			App.M1.Hpll.HallAllRun = 0;//当前等待学习按键往回拨，才允许启动
			if(App.M1.Key.StadyFg == 0){//学习按键弹起时，才能正常去启动
				App.M1.Hpll.HallStadySta = 3;//学习完成
			}
		}
	}
}


void HallStady_Check2(void){
	float hallerr[8] = {0};
	uint8_t i = 0;
	uint8_t j = 0;
	float temp = 0;
	
	if(App.M2.AllowRun == 0){
		
		if(App.M2.Hpll.HallStadySta == 0){//还未学习过
			
			App.M2.Hpll.HallAllRun = 0;//还未学习，不允许启动
			
			if(App.M2.Key.StadyFg == 1){//按键指令允许学习
				App.M2.Hpll.HallStadySta = 1;
			}
			
		}else if(App.M2.Hpll.HallStadySta == 1){//正在进行开环转动
			App.M2.Hpll.HallAllRun = 2;//允许启动去学习
		}else if(App.M2.Hpll.HallStadySta == 2){//已经开环转动过，现在需要判定学习是否正确
			App.M2.Hpll.HallAllRun = 0;//不允许启动
			
			
			
			//对数据取余 得出范围 0-1047
				for(i = 1;i < 7;i ++){
					//对数据进行一定的偏移
					App.M2.Hall.SdSaveAng[i] = App.M2.Hall.SdSaveAng[i] - (angoff * ANGLE_2PI / 360.0f);//偏移一定的角度
					if(App.M2.Hall.SdSaveAng[i] < 0){
						App.M2.Hall.SdSaveAng[i] += ANGLE_2PI;
					}else if(App.M2.Hall.SdSaveAng[i] > ANGLE_2PI){
						App.M2.Hall.SdSaveAng[i] -= ANGLE_2PI;
					}
					
					//转为定点数运算
					App.M2.Hall.SdSaveAng_Q15[i] = App.M2.Hall.SdSaveAng[i] * 1000.0f;
					//取余
					App.M2.Hall.SdAngQuYu[i] = App.M2.Hall.SdSaveAng_Q15[i] % 1047;//对学习的角度进行取余 ，求平均值  1047 是6.28 * 1000 / 6
				}
				
				//算出是否有交叉的取余值
				App.M2.Hall.SdJiaoChaFg = 0;
				
				for(i = 1; i < 7 ; i ++){
					for(j = (i + 1);j < 7; j ++){
						if( myabs(App.M2.Hall.SdAngQuYu[i] - App.M2.Hall.SdAngQuYu[j]) > 523){
							App.M2.Hall.SdJiaoChaFg = 1;//有交叉数据
							break;
						}
					}
				}
				
				
				App.M2.Hall.SdAngQuYuPJ = 0;//清除平均数
				//求平均数
				if(App.M2.Hall.SdJiaoChaFg == 0){//数据无交叉
					for(i = 1; i < 7 ; i ++){
						//计数取余之后的平均数
						App.M2.Hall.SdAngQuYuPJ += App.M2.Hall.SdAngQuYu[i];
					}
				}else {//数据有交叉 也就是 1 2 3 1046 1045 1044
					for(i = 1; i < 7 ; i ++){
						if(App.M2.Hall.SdAngQuYu[i] > 523){//1047 / 2
							App.M2.Hall.SdAngQuYu[i] = App.M2.Hall.SdAngQuYu[i] - 1047;
						}
						//计数取余之后的平均数
						App.M2.Hall.SdAngQuYuPJ += App.M2.Hall.SdAngQuYu[i];
					}
				}
				
				//求取余之后的平均值
				App.M2.Hall.SdAngQuYuPJ = App.M2.Hall.SdAngQuYuPJ / 6;
				
				//算出取余值和平均取余值的差值
				App.M2.Hall.SdAngQuYu_AllPJ = 0;
				for(i = 1;i < 7;i ++){
					App.M2.Hall.SdAngQuYu_PJ[i] = App.M2.Hall.SdAngQuYu[i] - App.M2.Hall.SdAngQuYuPJ;
					App.M2.Hall.SdAngQuYu_AllPJ += App.M2.Hall.SdAngQuYu_PJ[i];
				}
				
//				if(myabs(App.M2.Hall.SdAngQuYu_AllPJ) > 2){
//					App.M2.Err = E_SDAHALL;
//				}
				
				//算出均匀划分的学习角度
				for(i = 1;i < 7;i ++){
					temp = App.M2.Hall.SdSaveAng_Q15[i] - App.M2.Hall.SdAngQuYu_PJ[i];
					
					while (temp >  6283) {
						temp -= 6283;
					}
					while (temp < 0) {
						temp += 6283;
					}
					
					App.M2.Hall.SdSaveAng_Q15[i] = temp;
				}
				
				//更新平滑之后的角度
				for(i = 1;i < 7;i ++){
					App.M2.Hall.SdSaveAng[i] = App.M2.Hall.SdSaveAng_Q15[i] / 1000.0f;
				}
				
				for(i = 1;i < 7;i ++){
					hallerr[i] = App.M2.Hall.SdSaveAng[i] * 1000.0f;
				}
				hallerr[7] = 65535;//冒泡排序法

				//冒泡排序法
				for(i = 0;i < 7;i ++){//控制比较轮次，一共 n-1 趟
					for(j = 0; j < 7 - i ; j ++){//控制两个挨着的元素进行比较
						if(hallerr[j] > hallerr[j+1]){
							temp = hallerr[j];
							hallerr[j] = hallerr[j+1];
							hallerr[j+1] = temp;
						}
					}
				}

				for(i = 2;i < 7;i ++){
					if(hallerr[i] - hallerr[i - 1] < MCPara2[57]){
						App.M2.Err = E_SDAHALL;
						App.BitErr |= ERR_WL_Angle_Learn;//威力电机学习错误
					}
				}
				
				if(App.M2.Err == E_NONE){
					//保存hall学习之后的角度
					FLASH_Write(TEST_FLASH_ADDRESS_START_HALL2,App.M2.Hall.SdSaveAng_Q15,8);
				}else {
					App.M2.Hpll.HallStadySta = 0;//学习失败
				}
				
				if(App.M2.Err != E_SAVE){
					App.M2.Hpll.HallStadySta = 4;//学习完成
				}else {
					App.M2.Hpll.HallStadySta = 0;//学习失败
				}
				
			App.M2.Hall.SdSaveAng[0] = -1;
			App.M2.Hall.SdSaveAng[7] = -1;
		}else if(App.M2.Hpll.HallStadySta == 3){//已经学习完成
			App.M2.Hpll.HallAllRun = 1;//允许正常去启动
			if(App.M2.Key.StadyFg == 1){//如果重新有指令去学习，那么需要重新学习
				App.M2.Hpll.HallStadySta = 1;//去开环拖
			}
		}else if(App.M2.Hpll.HallStadySta == 4){//已经学习完成
			App.M2.Hpll.HallAllRun = 0;//当前等待学习按键往回拨，才允许启动
			if(App.M2.Key.StadyFg == 0){//学习按键弹起时，才能正常去启动
				App.M2.Hpll.HallStadySta = 3;//学习完成
			}
		}
	}
}





