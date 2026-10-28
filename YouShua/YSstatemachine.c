#include "YSstatemachine.h"
#include "youshua.h"
#include "YSmcctl.h"
#include "mcuart.h"
#include "fangbo.h" /* 复用桥臂全关动作，故障时不只清有刷比较值。 */

/*
 * 函数功能：清理本路历史估速及共用输出缓存，使启动或故障关断不继承上一轮调速状态。
 * 输入参数：channel指向本次启动或故障的有刷通道；调用前必须撤销该通道运行许可。
 * 返回参数：无。
 */
static void YS_ResetControlState(CHANNEL__TYPEDEF *channel)
{
    channel->Emf.qEmf = 0.0f; /* 清本路上一轮反电动势，避免故障估算值带入新启动。 */
    channel->Emf.qEmfLPF = 0.0f; /* 清停机期间不会自行衰减的滤波状态。 */
    channel->Emf.qIerr = 0.0f; /* 清旧电流差，首轮估速不能复用故障前的电感压降。 */
    channel->Emf.qLastI = channel->ADC.CurNow; /* 以当前采样建立差分基准，不能置零后制造虚假的电流突变。 */
    App2.SysCtl.SpdNow = 0.0f; /* 清共用转速反馈，停机及启动准备阶段不回传旧速度。 */
    App2.SysCtl.SpdNowLPF = 0.0f; /* 速度环等待本次采样重新生成反馈。 */
    App2.SysCtl.SpdSetSet = 0.0f; /* 目标爬坡从零开始，不能接续报警前的目标速度。 */
    App2.SysCtl.PwmSet = 0; /* 清兼容PWM目标，保持软件停机状态一致。 */
    App2.SysCtl.PwmNow = 0; /* 清兼容PWM输出，避免旧状态被其它维护入口读到。 */
    App2.Log.VbusRef = 0.0f; /* 清速度环留下的电压目标，下一轮由新误差重新计算。 */
    App2.Log.VbusSetPWM = 0; /* 清电压环目标占空比，不能恢复旧BUCK输出。 */
    App2.Log.VbusNowPWM = 0; /* 清实际爬坡缓存，与已清零的硬件BUCK保持一致。 */
}

//通道1等待状态
void Ctl_Wait(void){
	if((App2.Ch1.Start == 1)&&(App2.Log.BreakSta == 0)){
		App2.Ch1.Status = CTLS_INIT;	//允许启动，进入初始化
	}
	if(App2.Err > 0){
		App2.Ch1.Status = CTLS_ERROR;	//错误，进入错误处理
	}
}

//通道2等待状态
void Ctl_Wait2(void){
	if((App2.Ch2.Start == 1)&&(App2.Log.BreakSta == 0)){
		App2.Ch2.Status = CTLS_INIT;	//允许启动，进入初始化
	}
	if(App2.Err > 0){
		App2.Ch2.Status = CTLS_ERROR;	//错误，进入错误处理
	}
}


//通道1初始化状态
/*
 * 函数功能：在启动仍有效时清理A路历史估速和输出，再按零占空比顺序初始化参数与硬件。
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_Init(void){
    if ((App2.Ch1.Start == 0U) || (MotorCommand_IsStopRequested() != 0U) ||
        (MotorCommand_IsOutputLocked() != 0U) || (App2.Err != E_NONE))
    {
        App2.Ch1.AllowRun = 0U; /* 松脚或故障先撤销本路功率许可。 */
        PWM_3l_Stop(); /* 启动取消或锁定时完整关闭本路桥臂，不能留下桥臂使能。 */
        Vbus_Tim_OFF(); /* 撤销共用BUCK，不能继续执行初始化中的开启动作。 */
        App2.Ch1.Status = (App2.Err == E_NONE) ? CTLS_WAIT : CTLS_ERROR; /* 返回等待或原故障状态。 */
        return; /* 不再进入有刷开环启动。 */
    }


    App2.Ch1.AllowRun = 0U; /* 参数和估速复位期间禁止ADC普通调速写回输出。 */
    Vbus_Tim_OFF(); /* 先清硬件BUCK占空比，桥臂不得接入报警前残留的驱动输出。 */
    YS_ResetControlState(&App2.Ch1); /* 清本路历史估速和共用爬坡缓存，保持原目标参数不变。 */
    App2.Log.RunTimCnt1 = 0U; /* 本次启动重新执行本通道的采样等待，不能复用旧运行计时。 */

	//通道1有刷刀头电阻电感初始化
	if(App2.Log.Drill == 1){		//通道1刀头1
		//刀头1电阻电感和转换系数和保护电流
		App2.Ch1.Emf.qRs = (float)YSPara1[15] / 1000.0f;
		App2.Ch1.Emf.qLsDtq = (float)YSPara1[16] * PWM_FREQUENCY / 1000000.0f;
		App2.Ch1.Emf.qSpdxisu = (float)YSPara1[17];
		App2.Ch1.Emf.qRsxisu = (float)YSPara1[18] / 100.0f;
		//刀头1过流保护值
		if(App2.Log.CtlMode == 3){				//往复运动的过流保护值
			App2.Ch1.Prot.OverCurrent = App2.Ch1.Prot.OverCurrent2;
			App2.Ch1.Prot.CurCnt			= App2.Ch1.Prot.CurCnt2;
		}else{														//单向运动的过流保护值
			App2.Ch1.Prot.OverCurrent = App2.Ch1.Prot.OverCurrent1;
			App2.Ch1.Prot.CurCnt			= App2.Ch1.Prot.CurCnt1;
		}
	}
	else if(App2.Log.Drill == 2){//通道1刀头2
		//刀头2电阻电感和转换系数
		App2.Ch1.Emf.qRs = (float)YSPara1[27] / 1000.0f;
		App2.Ch1.Emf.qLsDtq = (float)YSPara1[28] * PWM_FREQUENCY / 1000000.0f;
		App2.Ch1.Emf.qSpdxisu = (float)YSPara1[29];
		App2.Ch1.Emf.qRsxisu = (float)YSPara1[30] / 100.0f;
		//刀头2过流保护值
		if(App2.Log.CtlMode == 3){			//往复运动的过流保护值
			App2.Ch1.Prot.OverCurrent = App2.Ch1.Prot.OverCurrent4;
			App2.Ch1.Prot.CurCnt			= App2.Ch1.Prot.CurCnt4;
		}else{													//单向运动的过流保护值
			App2.Ch1.Prot.OverCurrent = App2.Ch1.Prot.OverCurrent3;
			App2.Ch1.Prot.CurCnt			= App2.Ch1.Prot.CurCnt3;
		}		
	}
	//Log/PWM参数初始化
	App2.SysCtl.MaxSpdSet = (float)( PWM_PERIOD_COUNT * YSPara1[12] /1000.0f);	//最大PWM
	App2.SysCtl.MinSpdSet = 0;
	App2.Ch1.Dir.Now = App2.Ch1.Dir.Set;		//确认启动方向
	App2.Log.BreakSta = 0;									//初始化刹车为正常运行状态
	App2.Ch1.Tim.StopTimCnt = 0;
	//PID参数初始化
	YSPI_Init();
	Vbus_Tim_ON(); /* 先启用零占空比的BUCK，保持初始化过程无驱动输出。 */
	MCPwm_Init1(); /* 估速、PI和爬坡已复位后才接通本路桥臂。 */
	//允许启动
	App2.Ch1.Status = CTLS_OPENLOOP_RUN;		//跳转到开环
	App2.Ch1.AllowRun = 1;
	if(App2.Err > 0){
		App2.Ch1.Status= CTLS_ERROR; 
	}
	
}

//通道2初始化状态
/*
 * 函数功能：在启动仍有效时清理B路历史估速和输出，再按零占空比顺序初始化参数与硬件。
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_Init2(void){
    if ((App2.Ch2.Start == 0U) || (MotorCommand_IsStopRequested() != 0U) ||
        (MotorCommand_IsOutputLocked() != 0U) || (App2.Err != E_NONE))
    {
        App2.Ch2.AllowRun = 0U; /* 松脚或故障先撤销本路功率许可。 */
        PWM_3l_Stop2(); /* 启动取消或锁定时完整关闭本路桥臂，不能留下桥臂使能。 */
        Vbus_Tim_OFF(); /* 撤销共用BUCK，不能继续执行初始化中的开启动作。 */
        App2.Ch2.Status = (App2.Err == E_NONE) ? CTLS_WAIT : CTLS_ERROR; /* 返回等待或原故障状态。 */
        return; /* 不再进入有刷开环启动。 */
    }

	
    App2.Ch2.AllowRun = 0U; /* 参数和估速复位期间禁止ADC普通调速写回输出。 */
    Vbus_Tim_OFF(); /* 先清硬件BUCK占空比，桥臂不得接入报警前残留的驱动输出。 */
    YS_ResetControlState(&App2.Ch2); /* 清本路历史估速和共用爬坡缓存，保持原目标参数不变。 */
    App2.Log.RunTimCnt2 = 0U; /* 本次启动重新执行本通道的采样等待，不能复用旧运行计时。 */

	//通道2有刷刀头电阻电感初始化
	if(App2.Log.Drill == 1){		//通道2刀头1
		//通道2刀头1电阻电感和转换系数、、保护电流
		App2.Ch2.Emf.qRs = (float)YSPara2[15] / 1000.0f;
		App2.Ch2.Emf.qLsDtq = (float)YSPara2[16] * PWM_FREQUENCY / 1000000.0f;
		App2.Ch2.Emf.qSpdxisu = (float)YSPara2[17];
		App2.Ch2.Emf.qRsxisu = (float)YSPara2[18] / 100.0f;
		if(App2.Log.CtlMode == 3){				//往复运动的过流保护值
			App2.Ch2.Prot.OverCurrent = App2.Ch2.Prot.OverCurrent2;
			App2.Ch2.Prot.CurCnt			 = App2.Ch2.Prot.CurCnt2;
		}else{													//单向运动的过流保护值
			App2.Ch2.Prot.OverCurrent = App2.Ch2.Prot.OverCurrent1;
			App2.Ch2.Prot.CurCnt			 = App2.Ch2.Prot.CurCnt1;
		}
	}
	else if(App2.Log.Drill == 2){	//通道2刀头2
		//通道2刀头2电阻电感和转换系数、保护电流
		App2.Ch2.Emf.qRs = (float)YSPara2[27] / 1000.0f;
		App2.Ch2.Emf.qLsDtq = (float)YSPara2[28] * PWM_FREQUENCY / 1000000.0f;
		App2.Ch2.Emf.qSpdxisu = (float)YSPara2[29];
		App2.Ch2.Emf.qRsxisu = (float)YSPara2[30] / 100.0f;
		if(App2.Log.CtlMode == 3){				//往复运动的过流保护值
			App2.Ch2.Prot.OverCurrent = App2.Ch2.Prot.OverCurrent4;
			App2.Ch2.Prot.CurCnt			 = App2.Ch2.Prot.CurCnt4;
		}else{													//单向运动的过流保护值
			App2.Ch2.Prot.OverCurrent = App2.Ch2.Prot.OverCurrent3;
			App2.Ch2.Prot.CurCnt			 = App2.Ch2.Prot.CurCnt3;
		}		
	}
	//Log/PWM参数初始化
	App2.SysCtl.MaxSpdSet = (float)( PWM_PERIOD_COUNT * YSPara2[12] /1000.0f);	//最大PWM
	App2.SysCtl.MinSpdSet = 0;
	App2.Ch2.Dir.Now = App2.Ch2.Dir.Set;		//确认启动方向
	App2.Log.BreakSta = 0;					//初始化刹车为正常运行状态
	App2.Ch2.Tim.StopTimCnt = 0;
	//PID参数初始化
	YSPI_Init();
	Vbus_Tim_ON(); /* 先启用零占空比的BUCK，保持初始化过程无驱动输出。 */
	MCPwm_Init2(); /* 估速、PI和爬坡已复位后才接通本路桥臂。 */
	//允许启动
	App2.Ch2.AllowRun = 1;
	App2.Ch2.Status = CTLS_OPENLOOP_RUN;

	if(App2.Err > 0){
		App2.Ch2.Status = CTLS_ERROR; 
	}
	
}



//顺逆风检测
void Ctl_Check(void){
	
}

//顺逆风检测
void Ctl_Check2(void){
	
}
//刹车状态
void Ctl_Brake(void){
	
}
//刹车状态
void Ctl_Brake2(void){
	
}
//开环函数
/*
 * 函数功能：有刷A路开环转闭环前再次确认停止状态
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_OpenLoop(void)
{
    if ((App2.Ch1.Start == 0U) || (MotorCommand_IsStopRequested() != 0U))
    {
        App2.Ch1.AllowRun = 0U; /* 尚未进入闭环即松脚时，不再建立运行许可。 */
        TIME1_PWM_Stop_3Channel(); /* 清本通道相输出。 */
        Vbus_Tim_OFF(); /* 同步关闭共用BUCK。 */
        App2.Ch1.Status = CTLS_WAIT; /* 结束本次启动，等待新的合法命令。 */
        return; /* 停止不能被下面的闭环状态覆盖。 */
    }
    App2.Ch1.Status = CTLS_CLOSELOOP_RUN; /* 有效启动沿用原有直接进入闭环的流程。 */
}
//开环函数
/*
 * 函数功能：有刷B路开环转闭环前再次确认停止状态
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_OpenLoop2(void)
{
    if ((App2.Ch2.Start == 0U) || (MotorCommand_IsStopRequested() != 0U))
    {
        App2.Ch2.AllowRun = 0U; /* 尚未进入闭环即松脚时，不再建立运行许可。 */
        TIME8_PWM_Stop_3Channel(); /* 清本通道相输出。 */
        Vbus_Tim_OFF(); /* 同步关闭共用BUCK。 */
        App2.Ch2.Status = CTLS_WAIT; /* 结束本次启动，等待新的合法命令。 */
        return; /* 停止不能被下面的闭环状态覆盖。 */
    }
    App2.Ch2.Status = CTLS_CLOSELOOP_RUN; /* 有效启动沿用原有直接进入闭环的流程。 */
}


//通道1闭环状态
void Ctl_CloseLoop(void){
	TMR_CCxCmd(TMR1, TMR_Channel_3, TMR_CCx_Enable);  
	TMR_CCxNCmd(TMR1, TMR_Channel_3, TMR_CCxN_Enable); 
	TMR_Cmd(TMR1, ENABLE);
	//判断是否需要刹车
	if(	App2.Ch1.Start == 0 || (App2.Ch1.Dir.Now != App2.Ch1.Dir.Set)|| App2.Log.BreakSta >= 1 ){			//不允许启动，方向改变，
		//进入刹车状态
		if(App2.Log.BreakSta == 0){
			App2.Log.BreakSta = 1;			//进入刹车状态
		}
		//停止时间计时
		if(App2.Ch1.Tim.StopTimFg == 1){	
			App2.Ch1.Tim.StopTimFg = 0;		//刹车停止时间计数
			App2.Ch1.Tim.StopTimCnt ++;
		}
		//根据类型判断刹车
		if((	App2.Ch1.Start == 0 && App2.Ch1.Tim.StopTimCnt >= YSPara1[59])													//停止指令 或者报错停止的 刹车时间
			|| (App2.Ch1.Start == 1 && App2.Ch1.Tim.StopTimCnt >= YSPara1[60] && App2.Log.CtlMode !=3 )	//正常单向切换转向的刹车时间-->单向切换转相
			|| (App2.Ch1.Start == 1 && App2.Ch1.Tim.StopTimCnt >= YSPara1[61] && App2.Log.CtlMode == 3)	//往复模式下的刹车时间-->往复运动
			){
			App2.Ch1.Tim.StopTimCnt = 0;
			App2.Log.RunTimCnt1 = 0;
			App2.Ch1.AllowRun = 0;
			App2.Log.BreakSta = 0;				//刹车完成，进入正常状态
			TIME1_PWM_Stop_3Channel();
			Vbus_Tim_OFF();								//关Buck
			App2.Ch1.Status = CTLS_STOP;
		}
		
	}else if(App2.Ch1.Start == 1){
		App2.Ch1.Tim.StopTimCnt = 0;
	}
	
	if(App2.Err > 0){
		App2.Ch1.Status = CTLS_ERROR; 
	}
}

//通道2闭环状态
/*
 * 函数功能：执行有刷B路运行及按停止、换向或往复原因制动
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_CloseLoop2(void){
	TMR_CCxCmd(TMR8, TMR_Channel_3, TMR_CCx_Enable);
	TMR_CCxNCmd(TMR8, TMR_Channel_3, TMR_CCxN_Enable);
	TMR_Cmd(TMR8, ENABLE); /* B路运行和制动只操作TMR8，不能误开A路定时器。 */
	//判断是否需要刹车
	if(App2.Ch2.Start == 0 || (App2.Ch2.Dir.Now != App2.Ch2.Dir.Set)|| App2.Log.BreakSta >= 1){							//执行完一次刹车
		//进入刹车状态
		if(App2.Log.BreakSta == 0){
			App2.Log.BreakSta = 1;//进入刹车状态
		}
		//停止时间计时
		if(App2.Ch2.Tim.StopTimFg == 1){
			App2.Ch2.Tim.StopTimFg = 0;
			App2.Ch2.Tim.StopTimCnt ++;	
		}
		//根据类型判断刹车
		if((	App2.Ch2.Start == 0 && App2.Ch2.Tim.StopTimCnt >= YSPara2[59])		//停止指令 或者报错停止的 刹车时间
			|| (App2.Ch2.Start == 1 && App2.Ch2.Tim.StopTimCnt >= YSPara2[60] && App2.Log.CtlMode !=3 )	//正常单向切换转向的刹车时间-->单向切换转相
			|| (App2.Ch2.Start == 1 && App2.Ch2.Tim.StopTimCnt >= YSPara2[61] && App2.Log.CtlMode == 3)	//往复模式下的刹车时间-->往复运动
			){
			App2.Ch2.Tim.StopTimCnt = 0;
			App2.Log.RunTimCnt2 = 0;
			App2.Ch2.AllowRun = 0;
			App2.Log.BreakSta = 0;
			//刹车完成，进入正常状态
			TIME8_PWM_Stop_3Channel();
			Vbus_Tim_OFF();
			App2.Ch2.Status = CTLS_STOP;
		}
	}else if(App2.Ch2.Start == 1){
		App2.Ch2.Tim.StopTimCnt = 0;
	}
	if(App2.Err > 0){
		App2.Ch2.Status = CTLS_ERROR; 
	}
}


//通道1停止状态
void Ctl_Stop(void){
	App2.Ch1.AllowRun = 0;					//不允许运行
	TIME1_PWM_Stop_3Channel();			//关闭MOS
	App2.Ch1.Status = CTLS_WAIT;		//重新进入空闲状态
	if(App2.Err > 0){								//有错误则进入错误状态,
		App2.Ch1.Status = CTLS_ERROR; 
	}
}

//通道2停止状态
void Ctl_Stop2(void){
	App2.Ch2.AllowRun = 0;						//不允许运行
	TIME8_PWM_Stop_3Channel();        //关闭MOS
	App2.Ch2.Status = CTLS_WAIT;      //重新进入空闲状态
	if(App2.Err > 0){                 //有错误则进入错误状态,
		App2.Ch2.Status = CTLS_ERROR; 
	}
}

//通道1错误状态
/*
 * 函数功能：关闭有刷A路故障输出并清理重启状态；保留错误码，收到合法清错后回到等待态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_Error(void)
{
    App2.Ch1.AllowRun = 0U; /* 先撤销本路调速许可，故障关断期间ADC不能重新抬高输出。 */
    PWM_3l_Stop(); /* 关闭本路全部上下桥并清比较值，避免仅清比较值仍保留桥臂使能。 */
    if (App2.Err != E_NONE) /* 仅实际有刷故障清共用母线，旧错误态退出不能干扰另一通道的新运行。 */
    {
        Vbus_Tim_OFF(); /* 本地报警立即清共用BUCK，不等待下一帧主控零速命令。 */
        YS_ResetControlState(&App2.Ch1); /* 清估速、目标爬坡及软件PWM，故障后等待多久都不会保留旧值。 */
        App2.Ch1.Start = 0U; /* 撤销本路旧启动请求，恢复后必须重新经过启动状态机。 */
        App2.Log.u32SetSpd = 0U; /* 清有刷旧命令目标，禁止错误清除后沿用报警前转速。 */
        App2.SysCtl.SpdSet = 0.0f; /* 同步撤销速度环输入目标，下一次有效命令重新赋值。 */
        App2.Log.RunTimCnt1 = 0U; /* 故障终止本路运行计时，下次启动必须重新等待采样。 */
        App2.Ch1.Tim.StopTimCnt = 0U; /* 清本路旧制动计时，不将故障过程计入下一次停止。 */
        App2.Log.BreakSta = 0U; /* 输出已全关，不再继续执行普通减速制动。 */
        App2.Ch1.Status = CTLS_ERROR; /* 保护函数可直接调用本入口，立即发布错误态并保留原错误码。 */
    }
    else
    {
        App2.Ch1.Status = CTLS_WAIT; /* 合法零速清除错误后，只返回等待态，不自动恢复运行。 */
        App2.Log.BreakSta = 0U; /* 清理错误态遗留的制动标记，允许后续新命令正常起步。 */
    }
}

//通道2错误状态
/*
 * 函数功能：关闭有刷B路故障输出并清理重启状态；保留错误码，收到合法清错后回到等待态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_Error2(void)
{
    App2.Ch2.AllowRun = 0U; /* 先撤销本路调速许可，故障关断期间ADC不能重新抬高输出。 */
    PWM_3l_Stop2(); /* 关闭本路全部上下桥并清比较值，避免仅清比较值仍保留桥臂使能。 */
    if (App2.Err != E_NONE) /* 仅实际有刷故障清共用母线，旧错误态退出不能干扰另一通道的新运行。 */
    {
        Vbus_Tim_OFF(); /* 本地报警立即清共用BUCK，不等待下一帧主控零速命令。 */
        YS_ResetControlState(&App2.Ch2); /* 清估速、目标爬坡及软件PWM，故障后等待多久都不会保留旧值。 */
        App2.Ch2.Start = 0U; /* 撤销本路旧启动请求，恢复后必须重新经过启动状态机。 */
        App2.Log.u32SetSpd = 0U; /* 清有刷旧命令目标，禁止错误清除后沿用报警前转速。 */
        App2.SysCtl.SpdSet = 0.0f; /* 同步撤销速度环输入目标，下一次有效命令重新赋值。 */
        App2.Log.RunTimCnt2 = 0U; /* 故障终止本路运行计时，下次启动必须重新等待采样。 */
        App2.Ch2.Tim.StopTimCnt = 0U; /* 清本路旧制动计时，不将故障过程计入下一次停止。 */
        App2.Log.BreakSta = 0U; /* 输出已全关，不再继续执行普通减速制动。 */
        App2.Ch2.Status = CTLS_ERROR; /* 保护函数可直接调用本入口，立即发布错误态并保留原错误码。 */
    }
    else
    {
        App2.Ch2.Status = CTLS_WAIT; /* 合法零速清除错误后，只返回等待态，不自动恢复运行。 */
        App2.Log.BreakSta = 0U; /* 清理错误态遗留的制动标记，允许后续新命令正常起步。 */
    }
}

//有刷状态机
void YSMotor_Ctl1(void){
	switch(App2.Ch1.Status){//按照程序运行过程中执行的频率排序
		case CTLS_CLOSELOOP_RUN:		
				Ctl_CloseLoop();		//闭环
					break;
		case CTLS_WAIT:
				Ctl_Wait();					//等待
					break;
		case CTLS_OPENLOOP_RUN:
				Ctl_OpenLoop();    	//开环         
					break;
		case CTLS_ERROR:
				Ctl_Error();				//错误
					break;
		case CTLS_INIT:
				Ctl_Init();					//初始化
					break;
		case CTLS_STOP:              
				Ctl_Stop();					//停止
					break;
		case CTLS_CHECK:
				Ctl_Check();				//检测
					break;
		case CTLS_BRAKE:
				Ctl_Brake();				//刹车
					break;
		default:
			  Ctl_Wait(); 				//等待            
					break;
	}
}




void YSMotor_Ctl2(void){
		switch(App2.Ch2.Status){//按照程序运行过程中执行的频率排序
			case CTLS_CLOSELOOP_RUN:
					Ctl_CloseLoop2();				//闭环
						break;                
			case CTLS_WAIT:             
					Ctl_Wait2();            //等待
						break;                
			case CTLS_OPENLOOP_RUN:     
					Ctl_OpenLoop2();        //开环       
						break;                
			case CTLS_ERROR:            
					Ctl_Error2();           //错误
						break;                
			case CTLS_INIT:             
					Ctl_Init2();            //初始化
						break;                
			case CTLS_STOP:                   
					Ctl_Stop2();            //停止
						break;                
			case CTLS_CHECK:            
					Ctl_Check2();           //检测
						break;                
			case CTLS_BRAKE:            
					Ctl_Brake2();           //刹车
						break;                
			default:                    
					Ctl_Wait2();            //等待   
						break;
		}
}




//电机1刹车 关闭PWM输出
void TIME1_PWM_Stop_3Channel(void){
	TMR_SetCompare1(TMR1,0);
	TMR_SetCompare2(TMR1,0);

}
//电机2刹车 关闭PWM输出
void TIME8_PWM_Stop_3Channel(void){
	TMR_SetCompare1(TMR8,0);
	TMR_SetCompare2(TMR8,0);

}

//BUCK电路开启
void Vbus_Tim_ON(void){
	TMR_Cmd(TMR2, ENABLE);
	TMR_SetCompare4(TMR2,0);
	PVCC_PWM_ON;
}

//BUCK电路关闭
void Vbus_Tim_OFF(void){
	TMR_SetCompare4(TMR2,0);
}



