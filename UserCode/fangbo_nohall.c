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
#include "mcuart.h" /* 启动闭环切换必须服从外部停止约束。 */

// 5-1-3-2-6-4		CCW
 // 5-4-6-2-3-1		CW			
//换相中断		 
//5BA-1CA-3CB-2AB-6AC-4BC

//通道1无霍尔换向+启动拖动检测CW
/*
 * 函数功能：处理无霍尔A路CW换相，限于有效启动阶段发布闭环运行
 * 输入参数：hall_val为当前检测相序。
 * 返回参数：无。
 */
void OpenLoop_check_changeCW(uint8_t hall_val){
//CW
	volatile uint8_t val = 0;
	if((App.FB.Status == HS_START) && (App.FB.Start != 0U) && (App.Logic.Brake_Sign == 0U) &&
       (MotorCommand_IsStopRequested() == 0U) &&
       (App.FB.OpenLoop.StartHallOkCnt > 2000) && (App.FB.OpenLoop.OpenLoopTime > MCPara[30])) /* 只有本次有效启动可以切闭环，制动期间的旧检测结果不得恢复RUN。 */{
		//切入闭环初始化
		App.FB.Rad_Now = 100; /* A路闭环初值只写A路，禁止改动B路估计状态。 */
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


#define NOHALL2_PHASE_BLANK_TICKS 4U /* 换相后20us内忽略比较器，避免桥臂切换瞬态被当成反电动势边沿。 */
#define NOHALL2_PHASE_STABLE_TICKS 2U /* 两次5us采样确认一个电平，滤除单次毛刺且保留高速换相余量。 */
#define NOHALL2_START_REQUIRED_SECTORS 6U /* 启动必须连续取得完整六扇区边沿，固定电平不能累计成功。 */

typedef struct
{
    uint8_t phase; /* 当前已经开始观测的驱动扇区，0表示尚未建立本轮起点。 */
    uint8_t blank_ticks; /* 本扇区尚未结束的5us消隐节拍。 */
    uint8_t opposite_ticks; /* 目标电平出现前，反向电平连续稳定的次数。 */
    uint8_t target_ticks; /* 已经见到反向电平后，目标电平连续稳定的次数。 */
    uint8_t armed; /* 先确认反向电平后才允许接受目标边沿。 */
    uint8_t accepted; /* 每个驱动扇区只接受一次边沿，重复采样不能重复计数。 */
} NoHall2PhaseState;

static volatile NoHall2PhaseState s_nohall2_phase; /* 只维护B路换相资格，独立于全部过流和堵转保护计数。 */

/*
 * 函数功能：在新的B路启动前清除边沿资格，避免继承上次故障或方向的采样状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void NoHall2_ResetStartup(void)
{
    s_nohall2_phase.phase = 0U; /* 第一条有效驱动扇区重新建立起点。 */
    s_nohall2_phase.blank_ticks = 0U; /* 不继承上次换相的消隐进度。 */
    s_nohall2_phase.opposite_ticks = 0U; /* 新扇区必须重新确认反向电平。 */
    s_nohall2_phase.target_ticks = 0U; /* 清除上一轮目标电平的稳定次数。 */
    s_nohall2_phase.armed = 0U; /* 禁止旧边沿资格直接触发新启动的换相。 */
    s_nohall2_phase.accepted = 0U; /* 本轮还没有接受任何实际边沿。 */
}

/*
 * 函数功能：按原相表检测B路有效边沿；启动收齐连续六扇区后接管，运行只接受消隐后的稳定边沿。
 * 输入参数：hall_val为当前驱动扇区1至6；clockwise为1时沿原CW相表前进，为0时沿原CCW相表前进。
 * 返回参数：无。
 */
static void NoHall2_CheckPhase(uint8_t hall_val, uint8_t clockwise)
{
    uint8_t value; /* 当前悬空相比较器的实际输入电平。 */
    uint8_t expected; /* 本扇区原代码期望的过零后电平，保持硬件极性不变。 */
    uint8_t next_phase; /* 沿原方向表得到的下一驱动扇区。 */
    uint32_t saved_basepri; /* 只在确认边沿后的短提交区阻止500us状态机抢占。 */
    uint8_t expected_phase; /* 用于检查开环强制换相是否连续推进。 */

    if ((hall_val < 1U) || (hall_val > 6U))
    {
        return; /* 首次强制换相前没有合法扇区，不能读取相表或发布启动成功。 */
    }
    if (App.FB2.Err != E_NONE)
    {
        return; /* 已有故障只交给原状态机关闭输出，不再登记或发出新的换相。 */
    }
    if ((App.FB2.OpenLoop.StartHallOkFg == 0U) &&
        ((App.FB2.Status != HS_START) || (App.FB2.Start == 0U) ||
         (App.Logic.Brake_Sign != 0U) || (MotorCommand_IsStopRequested() != 0U)))
    {
        return; /* 只有本次合法启动允许积累接管资格，制动不能重新进入运行。 */
    }
    next_phase = (clockwise != 0U) ? NextCWhall[hall_val] : NextCCWhall[hall_val]; /* 完全沿用原CW/CCW换相顺序。 */
    if (s_nohall2_phase.phase != hall_val)
    {
        expected_phase = (clockwise != 0U) ? NextCWhall[s_nohall2_phase.phase] : NextCCWhall[s_nohall2_phase.phase]; /* 0索引为原表的无效起点。 */
        if ((App.FB2.OpenLoop.StartHallOkFg == 0U) &&
            ((s_nohall2_phase.accepted == 0U) || (expected_phase != hall_val)))
        {
            App.FB2.OpenLoop.StartHallOkCnt = 0U; /* 漏掉扇区边沿或相序不连续，启动资格必须重新累计。 */
        }
        s_nohall2_phase.phase = hall_val; /* 记录本次实际驱动扇区。 */
        s_nohall2_phase.blank_ticks = NOHALL2_PHASE_BLANK_TICKS; /* 每次换相重新执行短消隐。 */
        s_nohall2_phase.opposite_ticks = 0U; /* 必须在新扇区重新见到反向电平。 */
        s_nohall2_phase.target_ticks = 0U; /* 旧扇区目标电平不能跨扇区累计。 */
        s_nohall2_phase.armed = 0U; /* 撤销旧扇区边沿资格。 */
        s_nohall2_phase.accepted = 0U; /* 本扇区尚未接受实际边沿。 */
        return; /* 换相当次的开关瞬态不能参与启动或速度反馈。 */
    }
    if (s_nohall2_phase.blank_ticks != 0U)
    {
        --s_nohall2_phase.blank_ticks; /* 只消耗新增采样消隐计数，不改变原无换相计时。 */
        return; /* 消隐期间等待硬件电平稳定。 */
    }
    if (s_nohall2_phase.accepted != 0U)
    {
        return; /* 开环仍停在同一扇区时，重复目标电平不能增加启动次数。 */
    }
    switch (hall_val)
    {
        case phase1: value = get_EN2B; expected = 1U; break; /* 原CA扇区检测B相变高。 */
        case phase2: value = get_EN2Z; expected = 1U; break; /* 原AB扇区检测C相变高。 */
        case phase3: value = get_EN2A; expected = 0U; break; /* 原CB扇区检测A相变低。 */
        case phase4: value = get_EN2A; expected = 1U; break; /* 原BC扇区检测A相变高。 */
        case phase5: value = get_EN2Z; expected = 0U; break; /* 原BA扇区检测C相变低。 */
        default: value = get_EN2B; expected = 0U; break; /* 已校验剩余只能是AC扇区6，检测B相变低。 */
    }
    if (s_nohall2_phase.armed == 0U)
    {
        if (value != expected)
        {
            ++s_nohall2_phase.opposite_ticks; /* 先证明本扇区实际处于过零前的电平。 */
            if (s_nohall2_phase.opposite_ticks >= NOHALL2_PHASE_STABLE_TICKS)
            {
                s_nohall2_phase.armed = 1U; /* 反向电平稳定后，才允许识别后续目标边沿。 */
            }
        }
        else
        {
            s_nohall2_phase.opposite_ticks = 0U; /* 扇区进入时就匹配的静态电平不算转子转动。 */
        }
        return; /* 建立起点的同一采样不能同时算成换相边沿。 */
    }
    if (value != expected)
    {
        s_nohall2_phase.target_ticks = 0U; /* 目标电平中断时重新消抖，单次毛刺不得触发换相。 */
        return; /* 继续等待本扇区真正的目标边沿。 */
    }
    ++s_nohall2_phase.target_ticks; /* 累计目标电平的连续稳定采样。 */
    if (s_nohall2_phase.target_ticks < NOHALL2_PHASE_STABLE_TICKS)
    {
        return; /* 至少两次采样确认后才接受本次边沿。 */
    }
    saved_basepri = __get_BASEPRI(); /* 保存进入前的中断优先级门限。 */
    __set_BASEPRI_MAX(3U << (8U - __NVIC_PRIO_BITS)); /* 仅阻止优先级3的500us状态机改写扇区；ADC1和硬件保护0仍可抢占。 */
    if ((App.FB2.OpenLoop.HallVal != hall_val) || (App.FB2.Err != E_NONE) ||
        (App.FB2.AllRun != 1U) || (App.Logic.Motor_Kind != 2U) ||
        (App.FB2.Dir.Now != ((clockwise != 0U) ? CW : CCW)) ||
        ((App.FB2.OpenLoop.StartHallOkFg == 0U) &&
         ((App.FB2.Status != HS_START) || (App.FB2.Start == 0U) ||
          (App.Logic.Brake_Sign != 0U) || (MotorCommand_IsStopRequested() != 0U) ||
          (App.FB2.OpenLoop.OpenLoopTime >= NOHALL2_START_TIMEOUT_MS))) ||
        ((App.FB2.OpenLoop.StartHallOkFg != 0U) &&
         (App.FB2.Status != HS_RUN) && (App.FB2.Status != HS_BRA)))
    {
        __set_BASEPRI(saved_basepri); /* 检测期间若已停止、报错或强制换相，恢复门限并丢弃旧边沿。 */
        return; /* 不能用旧扇区结果覆盖新的BRA/ERR状态或恢复运行。 */
    }
    s_nohall2_phase.accepted = 1U; /* 本扇区已接受一次稳定边沿，后续相同电平不得重复计数。 */
    if (App.FB2.OpenLoop.StartHallOkFg == 0U)
    {
        if (App.FB2.OpenLoop.StartHallOkCnt < NOHALL2_START_REQUIRED_SECTORS)
        {
            ++App.FB2.OpenLoop.StartHallOkCnt; /* B路按连续有效扇区计数，不再按5us静态电平次数计数。 */
        }
        if ((App.FB2.OpenLoop.StartHallOkCnt < NOHALL2_START_REQUIRED_SECTORS) ||
            (App.FB2.OpenLoop.OpenLoopTime <= MCPara2[30]))
        {
            __set_BASEPRI(saved_basepri); /* 尚未接管也必须恢复原中断门限。 */
            return; /* 资格不足或原最短开环时间未到，继续原限流与PWM上限内的强制拖动。 */
        }
        NoHall2_ResetSpeedValidity(); /* 切闭环后重新用实测完整周期放行PI，强制拖动区间不作反馈。 */
        mcApp_VoltageP_PIParam2.qOutMin = (float)MCPara2[35]; /* 接管继续使用原闭环最小PWM。 */
        mcApp_VoltageP_PIParam2.qOutMax = (float)MCPara2[34]; /* 接管继续使用原闭环最大PWM。 */
        App.Logic.VBusSetPWM = App.Logic.VBusNowPWM; /* 当前实际PWM保持连续，首次有效PI再反算积分。 */
        App.FB2.OpenLoop.StartHallOkCnt = 0U; /* 本轮启动资格已经消费。 */
        App.FB2.OpenLoop.ClossLoopFlg = 1U; /* 保留原闭环接管标志供其他模块使用。 */
        App.FB2.OpenLoop.StartHallOkFg = 1U; /* 从此只接受比较器确认的换相，不再强制拖动。 */
        App.FB2.Status = HS_RUN; /* 所有接管准备完成后最后发布运行状态。 */
    }
    App.FB2.OpenLoop.HallVal = next_phase; /* 采用原方向表推进到下一驱动扇区。 */
    Hall_PhaseChange2(next_phase, 7000); /* 保留原桥臂换相动作和PWM幅值，内部停止门禁仍有效。 */
    App.FB2.OpenLoop.SpdCnt_Arr[next_phase - 1U] = App.FB2.OpenLoop.Spd_NotHallCnt; /* 沿用六项速度数组及原5us单位。 */
    NoHall2_RecordSpeedInterval(next_phase, App.FB2.OpenLoop.Spd_NotHallCnt); /* 只有确认过的边沿才能登记有效测速。 */
    App.FB2.OpenLoop.Spd_NotHallCnt = 0U; /* 与原真实换相路径一样清计时；原无换相保护中断和阈值不变。 */
    __set_BASEPRI(saved_basepri); /* 换相和样本完整发布后恢复门限，原保护任务继续调度。 */
}

/*
 * 函数功能：按原CW相表执行B路无霍尔边沿确认和完整周期启动接管。
 * 输入参数：hall_val为当前驱动扇区1至6。
 * 返回参数：无。
 */
void OpenLoop_check_changeCW2(uint8_t hall_val)
{
    NoHall2_CheckPhase(hall_val, 1U); /* CW仅选择原正向相表，保护与输出仍走原B路路径。 */
}

//通道1无霍尔换向+启动拖动检测CCW
/*
 * 函数功能：处理无霍尔A路CCW换相，限于有效启动阶段发布闭环运行
 * 输入参数：hall_val为当前检测相序。
 * 返回参数：无。
 */
void OpenLoop_check_changeCCW(uint8_t hall_val){
//CW
	volatile uint8_t val = 0;
	if((App.FB.Status == HS_START) && (App.FB.Start != 0U) && (App.Logic.Brake_Sign == 0U) &&
       (MotorCommand_IsStopRequested() == 0U) &&
       (App.FB.OpenLoop.StartHallOkCnt > 2000) && (App.FB.OpenLoop.OpenLoopTime > MCPara[30])) /* 只有本次有效启动可以切闭环，制动期间的旧检测结果不得恢复RUN。 */{
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


/*
 * 函数功能：按原CCW相表执行B路无霍尔边沿确认和完整周期启动接管。
 * 输入参数：hall_val为当前驱动扇区1至6。
 * 返回参数：无。
 */
void OpenLoop_check_changeCCW2(uint8_t hall_val)
{
    NoHall2_CheckPhase(hall_val, 0U); /* CCW仅选择原反向相表，消隐和边沿规则与CW一致。 */
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




