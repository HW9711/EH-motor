#include <math.h>
#include "mcctl.h"
#include "mcuart.h"
#include "mcflash.h"
#include "adc.h"
#include "tim.h"
#include "at32_board.h"
#include "fangbo.h"
#include "youshua.h"
#include "YSmcctl.h"
#include "YSstatemachine.h"
#include "driver_param.h"
#include "protect.h" /* 所有参数22/24入口共用合法性检查和运行时装载规则。 */
#define MOTOR_PROTECTION_FRAME_ADDRESS 0xABU /* 新运行帧与旧0xAA明确区分，避免旧驱动误执行。 */
#define MOTOR_PROTECTION_FRAME_LENGTH 15U /* 新帧byte9~12为两项大端毫秒，CRC覆盖前13字节。 */
#define IDNE_TIM    20
#define MOTOR_COMMAND_FRAME_LENGTH 11U /* 手柄控制帧固定为11字节，末2字节必须是CRC16。 */
#define MOTOR_COMMAND_FREQUENCY_MAX 100U /* 往复频率命令上限为100，防止乘2后发生8位回绕。 */

static volatile uint8_t s_motor_command_zero_required = 1U; /* 上电或通信超时后必须先收到合法零速帧，禁止旧RUN直接恢复。 */


#define MOTOR_STOP_MARGIN_TICKS 100U /* 停机预算额外保留50ms调度裕量，500us任务每次计一个tick。 */
#define MOTOR_STOP_MAX_TICKS 10000U /* 拒绝超过5s的软件停机预算，防止异常Flash参数形成无界输出。 */
static volatile uint8_t s_motor_stop_pending = 0U; /* 仅外部零速或跨驱动类型切换置位，往复内部换向不使用此标志。 */
static volatile uint8_t s_motor_stop_fault = 0U; /* 停机超时锁存到驱动复位，周期零速帧不能自动解除。 */
static volatile uint32_t s_motor_stop_ticks = 0U; /* 从首次停止请求累计的500us节拍，重复零速帧不得清零。 */
static volatile uint32_t s_motor_stop_limit = 0U; /* 本次停止按实际通道、减速步长和定位参数计算的最大节拍数。 */
static MCUART_Type s_motor_rx_frame; /* 主循环私有解析快照，与串口DMA接收区及中断完成区隔离。 */

/*
 * 函数功能：查询外部停止或停机故障是否仍在约束运行，供启动和普通调速入口使用。
 * 输入参数：无。
 * 返回参数：非零表示不得重新启动或恢复普通运行输出；零表示没有外部停机约束。
 */
uint8_t MotorCommand_IsStopRequested(void)
{
    return (uint8_t)(s_motor_stop_pending | s_motor_stop_fault); /* 保留内部往复制动与外部停止的区别。 */
}

/*
 * 函数功能：查询功率输出是否因停机故障、上电或失联门禁而完全禁止。
 * 输入参数：无。
 * 返回参数：非零表示包括换相在内的输出入口必须保持关闭。
 */
uint8_t MotorCommand_IsOutputLocked(void)
{
    return (uint8_t)(s_motor_stop_fault | s_motor_command_zero_required); /* 正常受控制动期间不锁死必要制动输出。 */
}

/*
 * 函数功能：检查指定参数组是否具备可结束的停机递减和合法PWM范围。
 * 输入参数：motor_kind为协议电机类型，1/2无刷，3/4有刷。
 * 返回参数：参数满足停机基本约束返回1，否则返回0并禁止以该参数组启动。
 */
uint8_t MotorCommand_AreStopParametersValid(uint8_t motor_kind)
{
    const uint16_t *para; /* 按当前实际通道选参数，不能让B路继承A路的停机限制。 */
    if ((motor_kind == 1U) || (motor_kind == 2U))
    {
        para = (motor_kind == 1U) ? MCPara : MCPara2; /* 无刷两路分别读取本通道参数。 */
        return ((para[44] > 0U) && (para[44] <= 32767U) && (para[45] > 0U) &&
                (para[34] <= PWM_PERIOD_COUNT) && (para[35] <= para[34]) &&
                (para[28] <= PWM_PERIOD_COUNT) && (para[27] <= para[28])) ? 1U : 0U; /* 停机不再受运行最小PWM限制。 */
    }
    if ((motor_kind == 3U) || (motor_kind == 4U))
    {
        para = (motor_kind == 3U) ? YSPara1 : YSPara2; /* 有刷使用本通道停止斜率和停止时间。 */
        return ((para[56] > 0U) && (para[56] <= 32767U) && (para[59] > 0U) &&
                ((uint32_t)para[59] * 2U + MOTOR_STOP_MARGIN_TICKS <= MOTOR_STOP_MAX_TICKS)) ? 1U : 0U; /* 禁止零步长或异常超长停机。 */
    }
    return 0U; /* 未定义类型没有合法的输出和停止对象。 */
}

/*
 * 函数功能：按当前通道的正常制动及必要定位流程计算本次停机期限。
 * 输入参数：motor_kind、mode、run_type分别为电机类型、当前模式和反馈类型。
 * 返回参数：允许的500us节拍数；返回0表示该配置不能建立有限停机期限。
 */
static uint32_t MotorCommand_GetStopLimit(uint8_t motor_kind, uint8_t mode, uint8_t run_type)
{
    const uint16_t *para; /* 只引用实际通道参数，不修改已经校准的制动或保护配置。 */
    uint32_t ticks; /* 使用32位计算，避免参数乘法使停机期限回绕。 */
    uint8_t position_locked; /* 仅已建立定位基准的有霍尔运行模式需要附加定位时间。 */
    if (MotorCommand_AreStopParametersValid(motor_kind) == 0U)
    {
        return 0U; /* 参数无效时不能启动，也不能继续依赖普通减速。 */
    }
    if (motor_kind >= 3U)
    {
        para = (motor_kind == 3U) ? YSPara1 : YSPara2; /* 保留有刷原停止制动时间。 */
        return (uint32_t)para[59] * 2U + MOTOR_STOP_MARGIN_TICKS; /* 时间制动到期后仍留50ms完成输出关闭。 */
    }
    para = (motor_kind == 1U) ? MCPara : MCPara2; /* 无刷递减在500us控制任务中执行。 */
    ticks = (PWM_PERIOD_COUNT + (uint32_t)para[44] - 1U) / para[44] + MOTOR_STOP_MARGIN_TICKS; /* 从硬件允许的最大PWM计算上界。 */
    position_locked = (motor_kind == 1U) ? App.Pos.LockFlag1 : App.Pos.LockFlag2; /* A/B定位基准分别判断。 */
    if ((run_type == 2U) && (mode <= 3U) && (position_locked != 0U))
    {
        if ((para[50] >= 1000U) || (para[66] == 0U))
        {
            return 0U; /* 等待计数在1000ms回绕，非法等待值不能进入定位停机。 */
        }
        ticks += (uint32_t)para[50] * 2U + ((uint32_t)para[66] + 1U) * ((uint32_t)para[51] * 2U + 1U); /* 保留原定位步距及步数上限，覆盖每步调度间隔。 */
    }
    return (ticks <= MOTOR_STOP_MAX_TICKS) ? ticks : 0U; /* 超出可接受上界时拒绝配置，不截短正常制动流程。 */
}

/*
 * 函数功能：判断全部电机状态机是否已经退出启动、运行、制动或定位过程。
 * 输入参数：无。
 * 返回参数：全部处于等待或已关断的错误状态且运行许可均撤销时返回1。
 */
static uint8_t MotorCommand_AreChannelsQuiescent(void)
{
    return ((App.FB.AllRun == 0U) && (App.FB2.AllRun == 0U) &&
            ((App.FB.Status == HS_WAIT) || (App.FB.Status == HS_ERR)) &&
            ((App.FB2.Status == HS_WAIT) || (App.FB2.Status == HS_ERR)) &&
            (App2.Ch1.AllowRun == 0U) && (App2.Ch2.AllowRun == 0U) &&
            ((App2.Ch1.Status == CTLS_WAIT) || (App2.Ch1.Status == CTLS_ERROR)) &&
            ((App2.Ch2.Status == CTLS_WAIT) || (App2.Ch2.Status == CTLS_ERROR))) ? 1U : 0U; /* 不能只用AllRun判断预充电和定位阶段。 */
}

/*
 * 函数功能：关闭有刷、无刷两路全部运行输出，并统一清理可导致旧动作恢复的状态。
 * 输入参数：error为需要上报的错误码；0表示正常结束并保留已有保护错误。
 * 返回参数：无；调用方必须已禁止中断，保证状态清理与硬件关断作为同一次操作完成。
 */
static void MotorCommand_CloseOutputs(uint8_t error)
{
    App.Logic.Set_Spd = 0U; /* 撤销无刷共享运行目标。 */
    App.Logic.CtlMode = 0U; /* 停止普通运行、往复及独立定位调度。 */
    App.Logic.Brake_Sign = 0U; /* 当前停机已经由此关断出口接管。 */
    App.Logic.Brake_Kind = 0U; /* 不允许恢复时沿用上一种制动原因。 */
    App.Logic.Brake_StopPwmOK = 0U; /* 完成记录只属于已经结束的制动，不能带到下次启动。 */
    App.Logic.Brake_StopTimOK = 0U; /* 同步清理旧换型和换霍尔时间结果。 */
    App.Logic.Brake_TimeCnt = 0U; /* 清除上一轮往复制动计时。 */
    App.Logic.VBusSetPWM = 0; /* 普通速度环不能继承旧母线目标。 */
    App.Logic.VBusNowPWM = 0; /* 软件当前输出与硬件零输出保持一致。 */
    App.Pos.Start = 0U; /* 取消未完成的独立定位命令。 */
    App.Pos.PosFlag = 0U; /* 取消已经结束或异常中止的停机定位请求。 */
    App.Pos.LockPosStatus = 0U; /* 复位定位执行阶段，保留已校准的位置基准。 */
    App.FB.Start = 0U; /* 撤销无刷A路启动许可。 */
    App.FB2.Start = 0U; /* 撤销无刷B路启动许可。 */
    App.FB.AllRun = 0U; /* 无刷A路不能继续运行环或换相。 */
    App.FB2.AllRun = 0U; /* 无刷B路不能继续运行环或换相。 */
    App.FB.Spd_Set = 0.0f; /* 清A路速度环目标。 */
    App.FB2.Spd_Set = 0.0f; /* 清B路速度环目标。 */
    App.FB.Spd_Now = 0.0f; /* 清已停止更新的旧估计值；该值不替代机械停转测量。 */
    App.FB2.Spd_Now = 0.0f; /* B路同样不回传上一运行周期冻结的估计值。 */
    App.FB.Rad_Now = 0.0f; /* 下次启动不能继承旧角速度滤波结果。 */
    App.FB2.Rad_Now = 0.0f; /* 清B路旧角速度滤波结果。 */
    App2.Log.CtlMode = 0U; /* 撤销有刷往复和方向调度。 */
    App2.Log.u32SetSpd = 0U; /* 清有刷串口目标。 */
    App2.Log.BreakSta = 0U; /* 全关断之后才退出有刷制动阶段。 */
    App2.Log.VbusSetPWM = 0; /* 清有刷真正使用的母线目标。 */
    App2.Log.VbusNowPWM = 0; /* 清有刷真正使用的当前母线输出。 */
    App2.SysCtl.SpdSet = 0.0f; /* 撤销有刷速度环目标。 */
    App2.SysCtl.SpdSetSet = 0.0f; /* 清有刷爬坡后的旧目标。 */
    App2.SysCtl.SpdNow = 0.0f; /* 清停止更新的有刷旧速度估计。 */
    App2.SysCtl.SpdNowLPF = 0.0f; /* 防止待机反馈继续报告上一周期估计值。 */
    App2.SysCtl.PwmSet = 0; /* 清兼容PWM目标状态。 */
    App2.SysCtl.PwmNow = 0; /* 清兼容PWM输出状态。 */
    App2.Ch1.Start = 0U; /* 撤销有刷A路启动许可。 */
    App2.Ch2.Start = 0U; /* 撤销有刷B路启动许可。 */
    App2.Ch1.AllowRun = 0U; /* 禁止有刷A路普通调速重新写输出。 */
    App2.Ch2.AllowRun = 0U; /* 禁止有刷B路普通调速重新写输出。 */
    if (error != E_NONE)
    {
        App.FB.Err = error; /* 无刷A路反馈明确错误原因。 */
        App.FB2.Err = error; /* 无刷B路反馈相同板级停机错误。 */
        App2.Err = error; /* 有刷反馈同一板级错误，避免类型切换隐藏故障。 */
    }
    App.FB.Status = (App.FB.Err == E_NONE) ? HS_WAIT : HS_ERR; /* 先关闭再次换相的状态入口。 */
    App.FB2.Status = (App.FB2.Err == E_NONE) ? HS_WAIT : HS_ERR; /* B路采用相同状态约束。 */
    App2.Ch1.Status = (App2.Err == E_NONE) ? CTLS_WAIT : CTLS_ERROR; /* 有刷A路等待新命令或故障复位。 */
    App2.Ch2.Status = (App2.Err == E_NONE) ? CTLS_WAIT : CTLS_ERROR; /* 有刷B路等待新命令或故障复位。 */
    PWM_3l_Stop(); /* 实际关闭A路全部上下桥并清相PWM。 */
    PWM_3l_Stop2(); /* 实际关闭B路全部上下桥并清相PWM。 */
    TIME1_PWM_Stop_3Channel(); /* 同步清有刷A路使用的比较寄存器。 */
    TIME8_PWM_Stop_3Channel(); /* 同步清有刷B路使用的比较寄存器。 */
    Vbus_Tim_OFF(); /* 最后清共用BUCK输出，正常停机和故障关断都必须到达此处。 */
}

/*
 * 函数功能：锁定一次外部停止请求，重复零速帧只维持停止，不能重置进度。
 * 输入参数：无，使用当前实际运行的通道和模式。
 * 返回参数：无；调用方已禁止中断，防止普通ADC调速插入停止状态发布过程。
 */
static void MotorCommand_BeginStop(void)
{
    uint8_t kind; /* 停止对象必须取当前实际驱动，不能被新帧的另一通道覆盖。 */
    uint8_t mode; /* 保存原运行模式，保留正常单向及往复停止差异。 */
    uint8_t run_type; /* 区分无霍尔普通制动与有霍尔必要定位。 */
    if ((s_motor_stop_pending != 0U) || (s_motor_stop_fault != 0U))
    {
        return; /* 已经停止中或故障锁定时，不重新清标记和计时。 */
    }
    if (MotorCommand_AreChannelsQuiescent() != 0U)
    {
        MotorCommand_CloseOutputs(E_NONE); /* 空闲零速也实际核对并关闭输出，不沿用旧完成标记。 */
        return;
    }
    kind = (YSorWSFlag == 2U) ? App2.Log.Channel : App.Logic.Motor_Kind; /* 读取当前有刷或无刷的实际通道。 */
    mode = (YSorWSFlag == 2U) ? App2.Log.CtlMode : App.Logic.CtlMode; /* 截止时间由当前制动流程决定。 */
    run_type = (YSorWSFlag == 2U) ? 3U : App.Logic.LogicHall_or_pll; /* 有刷不使用无刷定位附加时间。 */
    s_motor_stop_limit = MotorCommand_GetStopLimit(kind, mode, run_type); /* 首次请求固定预算，后续帧不能延长。 */
    if (s_motor_stop_limit == 0U)
    {
        s_motor_stop_fault = 1U; /* 运行中发现无法保证停机的配置时锁定异常。 */
        MotorCommand_CloseOutputs(E_StopTimeout); /* 参数异常不能阻止实际关断。 */
        return;
    }
    s_motor_stop_ticks = 0U; /* 只在这一笔新的停止请求开始时清计时。 */
    s_motor_stop_pending = 1U; /* 先发布停止约束，所有启动及普通调速入口必须服从。 */
    App.Logic.Set_Spd = 0U; /* 撤销无刷共享目标。 */
    App2.Log.u32SetSpd = 0U; /* 撤销有刷共享目标。 */
    App.FB.Start = 0U; /* 立即撤销A路启动许可，覆盖预充电和开环阶段。 */
    App.FB2.Start = 0U; /* B路采用同样的停止优先级。 */
    App2.Ch1.Start = 0U; /* 有刷A路开始按停止原因制动。 */
    App2.Ch2.Start = 0U; /* 有刷B路开始按停止原因制动。 */
    App.Logic.Brake_StopPwmOK = 0U; /* 新制动绝不能使用上次的完成结果。 */
    App.Logic.Brake_StopTimOK = 0U; /* 清旧切换等待结果，避免提前确认本次制动。 */
    App.Logic.Brake_TimeCnt = 0U; /* 往复的旧计时不能使本次外部停止直接完成。 */
    if (YSorWSFlag == 1U)
    {
        if ((mode == 4U) || (mode == 5U))
        {
            MotorCommand_CloseOutputs(E_NONE); /* 独立步进定位被零速取消时直接退出，不能执行剩余旧步数。 */
            s_motor_stop_pending = 0U; /* 此定位请求已由实际全关断完成。 */
            return;
        }
        App.Logic.Change_Kind = App.Logic.Motor_Kind; /* 停止期间保留实际电机类型。 */
        App.Logic.Change_Hall = App.Logic.LogicHall_or_pll; /* 停止期间保留当前反馈类型。 */
        App.Logic.Change_Mode = 0U; /* 只在本次制动完成后应用停止模式。 */
        App.Logic.Brake_Kind = 1U; /* 外部松脚覆盖往复或换向制动原因。 */
        App.Logic.Brake_Sign = 1U; /* 发布本次新的无刷制动请求。 */
        App.Pos.PosFlag = (run_type == 2U) ? 1U : 0U; /* 保留原有霍尔定位停机，无霍尔不进入定位。 */
        if (mode != 3U)
        {
            App.Logic.Motor_Stop_StartFlag = 1U; /* 保留原单向停机后的最短启动间隔。 */
            App.Logic.Motor_Stop_Start_Tim_Cnt = 0U; /* 新一次外部停止从当前时刻计算间隔。 */
        }
    }
    else if ((App2.Ch1.AllowRun != 0U) || (App2.Ch2.AllowRun != 0U))
    {
        App2.Log.BreakSta = 1U; /* 有刷仍允许运行时立即封住普通速度环，等待原制动算法递减输出。 */
    }
}

/*
 * 函数功能：在500us定时任务中独立监督外部停机，截止时间不依赖模式和串口保活。
 * 输入参数：无。
 * 返回参数：1表示故障锁定、调用方应跳过正常控制；0表示可以继续执行原状态机。
 */
uint8_t MotorCommand_ServiceStop500us(void)
{
    uint32_t primask; /* 保存进入前的中断状态，只在最终关断时建立短临界区。 */
    if (s_motor_stop_fault != 0U)
    {
        primask = __get_PRIMASK(); /* 不破坏调用方已有的中断屏蔽状态。 */
        __disable_irq(); /* 避免ADC或换相插入全关断过程。 */
        MotorCommand_CloseOutputs(E_StopTimeout); /* 锁定期间重复维护真实输出为零。 */
        __set_PRIMASK(primask); /* 恢复调用方的中断状态。 */
        return 1U; /* 故障状态不再推进启动、往复或定位。 */
    }
    if (s_motor_stop_pending == 0U)
    {
        return 0U; /* 正常运行和内部往复制动完全沿用原调度。 */
    }
    if (MotorCommand_AreChannelsQuiescent() != 0U)
    {
        primask = __get_PRIMASK(); /* 只有全部状态机退出后才发布本次停止完成。 */
        __disable_irq(); /* 状态完成与实际关输出保持原子关系。 */
        MotorCommand_CloseOutputs(E_NONE); /* 清零共用母线及两路桥臂，避免只改软件状态。 */
        s_motor_stop_pending = 0U; /* 实际关闭后才允许下一份合法启动命令。 */
        __set_PRIMASK(primask); /* 恢复原中断使能状态。 */
        return 0U;
    }
    ++s_motor_stop_ticks; /* 停止期间每个500us节拍只递增一次，不被重复零速或RUN覆盖。 */
    if (s_motor_stop_ticks >= s_motor_stop_limit)
    {
        primask = __get_PRIMASK(); /* 到期立即通过统一出口关断。 */
        __disable_irq(); /* 先封住其它输出执行者，再修改故障状态。 */
        s_motor_stop_fault = 1U; /* 此错误只能通过驱动复位解除。 */
        MotorCommand_CloseOutputs(E_StopTimeout); /* 即使模式或状态错误，也覆盖所有实际输出。 */
        __set_PRIMASK(primask); /* 关断完成后恢复中断，让通信仍可上报故障。 */
        return 1U;
    }
    return 0U; /* 未到期时继续原制动流程，不改变正常减速力度。 */
}

extern uint16_t SPStatus[SpeNum];
extern void TIM_Motor_PWM(TMR_Type* TMRx);
volatile uint8_t YSorWSFlag = 1;		//有刷还是无刷 1-无刷 2有刷
	
/*
 * 函数功能：校验11/15字节控制帧的地址长度、模式和电机类型，时间非法由装载处独立回退。
 * 输入参数：seruart 为已经完成长度和CRC计算的串口接收上下文。
 * 返回参数：字段语义有效返回1，否则返回0且本帧不得刷新通信租约。
 */
static uint8_t MotorCommand_IsSemanticallyValid(const MCUART_Type *seruart)
{
	uint8_t motor_kind; /* 保存协议byte3的电机类型，1/2为无刷，3/4为有刷。 */
	uint8_t run_type; /* 保存协议byte6的运行反馈类型，必须与电机类型匹配。 */

	if((seruart == 0) || (((seruart->Addr != 0xAAU) || (seruart->RxLen != MOTOR_COMMAND_FRAME_LENGTH)) &&
        ((seruart->Addr != MOTOR_PROTECTION_FRAME_ADDRESS) || (seruart->RxLen != MOTOR_PROTECTION_FRAME_LENGTH))))
	{
		return 0U; /* 空上下文或非固定长度不能作为手柄控制命令。 */
	}
	if((seruart->R_DATA[1] < 1U) || (seruart->R_DATA[1] > 5U))
	{
		return 0U; /* 控制模式仅允许正转、反转、往复和两个定位方向。 */
	}
	if(seruart->R_DATA[2] > MOTOR_COMMAND_FREQUENCY_MAX)
	{
		return 0U; /* 异常频率不能进入后续乘2逻辑，避免回绕成不可预测值。 */
	}

	motor_kind = seruart->R_DATA[3]; /* 读取电机类型后按无刷和有刷分别验证byte6。 */
	run_type = seruart->R_DATA[6]; /* byte6在无刷表示有无Hall，在有刷表示刀头类型。 */
	if((motor_kind < 1U) || (motor_kind > 4U))
	{
		return 0U; /* 只允许板上两路无刷和两路有刷电机。 */
	}
	if(motor_kind <= 2U)
	{
		return ((run_type == 1U) || (run_type == 2U)) ? 1U : 0U; /* 无刷只接受无Hall或有Hall类型。 */
	}

	return ((run_type == 3U) || (run_type == 4U)) ? 1U : 0U; /* 有刷只接受刀头1或刀头2类型。 */
}

/*
 * 函数功能：手柄控制租约超时后立即清零无刷和有刷目标并关闭全部PWM，进入通信故障态。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorCommand_ForceCommunicationStop(void)
{
    uint32_t primask = __get_PRIMASK(); /* 保留调用前中断状态，避免关断途中被换相重新打开输出。 */
    __disable_irq(); /* 通信故障状态和全部功率关断一次发布。 */
    s_motor_command_zero_required = 1U; /* 恢复通信后仍须先收到合法零速帧。 */
    s_motor_stop_pending = 0U; /* 通信故障直接关断，不再等待普通制动进度。 */
    MotorCommand_CloseOutputs((s_motor_stop_fault != 0U) ? E_StopTimeout : E_HandShake); /* 通信错误不能覆盖已锁存的停机错误。 */
    __set_PRIMASK(primask); /* 输出关闭后恢复中断，允许继续接收零速或上报故障。 */
}

//所有串口数据处理
void UartDealResponse(void){
	#if _Uart1 == 1
	Modbus_Ctl(USART1,&SerUart1);
	#endif
	
	#if _Uart2 == 1
	Modbus_Ctl(USART2,&SerUart2);
	#endif
	
	#if _Uart3 == 1
	Modbus_Ctl(USART3,&SerUart3);
	#endif
	
	#if _Uart4 == 1
	Modbus_Ctl(UART4,&SerUart4);
	#endif
	
	#if _Uart5 == 1
	Modbus_Ctl(UART5,&SerUart5);
	#endif
	
	#if _Uart6 == 1
	Modbus_Ctl(USART6,&SerUart6);
	#endif
	
	}

	
//串口Modbus协议解析
/*
 * 函数功能：复制完整串口帧后校验并处理命令，外部停止优先于普通运行和类型切换。
 * 输入参数：USARTx为回应串口，seruart为中断发布的接收上下文。
 * 返回参数：无；控制状态在短临界区内一次发布，串口回应在恢复中断后发送。
 */
void Modbus_Ctl(USART_Type* USARTx, MCUART_Type *seruart){
    uint16_t i = 0;
    uint8_t temp = 0;
    float temp1 = 0;
    uint32_t command_primask; /* 保存短临界区前的中断状态，不能在串口发送期间屏蔽中断。 */
    if (seruart->RxEnd == 1U){
        command_primask = __get_PRIMASK(); /* 防止UART空闲中断在复制完成区时再次发布新帧。 */
        __disable_irq(); /* 仅保护长度和最多70字节的复制，不包含CRC和协议处理。 */
        s_motor_rx_frame.RxLen = seruart->RxLen; /* 快照中的长度与R_DATA使用同一发布版本。 */
        if ((s_motor_rx_frame.RxLen < 2U) || (s_motor_rx_frame.RxLen > MAX_RXBUF)){
            seruart->RxEnd = 0U; /* 丢弃非法长度，不能用它续租或修改运行状态。 */
            __set_PRIMASK(command_primask); /* 异常返回也必须恢复原中断状态。 */
            return;
        }
        for (i = 0U; i < s_motor_rx_frame.RxLen; ++i){
            s_motor_rx_frame.R_DATA[i] = seruart->R_DATA[i]; /* CRC及后续字段解析只读取这份不再变化的内容。 */
        }
        seruart->RxEnd = 0U; /* 已消费完成区，后续中断可以发布下一帧。 */
        __set_PRIMASK(command_primask); /* 复制完成立即恢复定时控制和ADC中断。 */
        seruart = &s_motor_rx_frame; /* 下方沿用原协议解析，但彻底隔离实时接收区。 */
		seruart->Addr = seruart->R_DATA[0];
		seruart->FunCode = seruart->R_DATA[1];
		seruart->RxCRCH = seruart->R_DATA[seruart->RxLen - 1] << 8;
		seruart->RxCRCL = seruart->R_DATA[seruart->RxLen - 2];
		seruart->RxCRC = seruart->RxCRCH + seruart->RxCRCL;		
		seruart->CalcCRC = CRC_Calc(seruart->R_DATA,seruart->RxLen - 2);	
		if(DriverParam_TryHandle(USARTx,seruart) != 0U){
			return;
		}
		//从模式（上位机发送指令，当前板子接收并处理）
		if(
			(seruart->Addr == MCPara[0] || seruart->Addr == 0xEE) 
			&& (seruart->CalcCRC == seruart->RxCRC || seruart->RxCRC == 0xAABB)
			&& seruart->RxLen <= 8
		){
			switch(seruart->FunCode){
				case 3://查询
					seruart->Re3num = seruart->R_DATA[5];		//查询个数
					seruart->ReAddrH = seruart->R_DATA[2];	//查询内容类型
					seruart->ReAddrL = seruart->R_DATA[3]; 	//查询起始地址

					if(seruart->ReAddrH == 0x10){//查询 MCPara[]			//查询0x10xx，无刷1参数
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= ParaNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数

							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCPara[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCPara[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}

							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);

							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}else if(seruart->ReAddrH == 0x11){//查询 MCPara2[]			//查询0x11xx，无刷2参数
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= ParaNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数

							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCPara2[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCPara2[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}

							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);

							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

					else if(seruart->ReAddrH == 0x20){//查询 MCStatus[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= StatusNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{

							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数
							Modbus_20_Return();
							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCStatus[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCStatus[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}
							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

					else if(seruart->ReAddrH == 0x60 || seruart->ReAddrH == 0x30){//查询 SPStatus[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= SpeNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数
							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = SPStatus[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = SPStatus[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}
							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

				break;
				case 6://更改
                    if ((MotorCommand_AreChannelsQuiescent() == 0U) || (MotorCommand_IsStopRequested() != 0U) ||
                        (App.Logic.Set_Spd != 0U) || (App2.Log.u32SetSpd != 0U))
                    {
                        ModbusErrorResponse(USARTx, seruart, 0x86U, 0x06U); /* 旧维护入口也禁止在启动、运行、定位或停止过程中改参数。 */
                        break; /* 不执行后续RAM或Flash写入。 */
                    }
					seruart->UpdateRegH = seruart->R_DATA[2];
					seruart->UpdateRegL = seruart->R_DATA[3];
					seruart->UpdatePara = (seruart->R_DATA[4] << 8) + (seruart->R_DATA[5]);
					//更改参数
					if(
						seruart->UpdateRegH == 0x10				//更改0x10xx，无刷1参数
					){
						if(seruart->UpdateRegL >= ParaNum){
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else {
							if((seruart->UpdatePara > 32767) || (((seruart->UpdateRegL == 22U) || (seruart->UpdateRegL == 24U)) &&
                                (seruart->UpdatePara != Protect_ValidateTime(seruart->UpdatePara)))){ /* 旧维护地址也不得写入越界保护时间。 */
								ModbusErrorResponse(USARTx,seruart,0x83,3);
							}else {
								MCPara[seruart->UpdateRegL] = seruart->UpdatePara;
								ModbusResponse(USARTx,seruart);
								Pos_Uart_Init();	//更新一下最新刀头减速比数据
							}
						}
						
					}else	if(
						seruart->UpdateRegH == 0x11				//更改0x11xx，无刷2参数
					){
						if(seruart->UpdateRegL >= ParaNum){
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else {
							if((seruart->UpdatePara > 32767) || (((seruart->UpdateRegL == 22U) || (seruart->UpdateRegL == 24U)) &&
                                (seruart->UpdatePara != Protect_ValidateTime(seruart->UpdatePara)))){ /* 旧维护地址也不得写入越界保护时间。 */
								ModbusErrorResponse(USARTx,seruart,0x83,3);
							}else {
								MCPara2[seruart->UpdateRegL] = seruart->UpdatePara;
								ModbusResponse(USARTx,seruart);
								Pos_Uart_Init();	//更新一下最新刀头减速比数据

							}
							
						}
					}
					//特殊指令
					else if(
						seruart->UpdateRegH == 0x60
						|| seruart->UpdateRegH == 0x30
					){    
						switch(seruart->UpdateRegL){
							case 0x00://启停

								ModbusResponse(USARTx,seruart);
							break;
							case 0x01://方向
								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x02://清除错误
								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x03://保存
								if((App.FB.AllRun== 0)&&(App.FB2.AllRun== 0)){
									if(seruart->UpdatePara == 1){
										FLASH_Write(TEST_FLASH_ADDRESS_START,MCPara,ParaNum);
									}else if(seruart->UpdatePara == 3){
										F4Flash_SMCPara_Erase(TEST_FLASH_ADDRESS_START);
									}
									ModbusResponse(USARTx,seruart);
								}else {
									ModbusErrorResponse(USARTx,seruart,0x83,5);
								}
								
							break;
								
							case 0x04://退出指令控制

								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x0A:
								
								ModbusResponse(USARTx,seruart);
							break;
							
							
							default:
								ModbusErrorResponse(USARTx,seruart,0x83,4);
							break;	
						}
					} 
				break;
				default:
					ModbusErrorResponse(USARTx,seruart,0x83,1);
				break;			 
			}
		}
		if(
			(seruart->Addr == MCPara2[0] || seruart->Addr == 0xEE) 
			&& (seruart->CalcCRC == seruart->RxCRC || seruart->RxCRC == 0xAABB)
			&& seruart->RxLen <= 8
		){
			switch(seruart->FunCode){
				case 3://查询
					seruart->Re3num = seruart->R_DATA[5];		//查询个数
					seruart->ReAddrH = seruart->R_DATA[2];	//查询内容类型
					seruart->ReAddrL = seruart->R_DATA[3]; 	//查询起始地址

					if(seruart->ReAddrH == 0x10){//查询 MCPara[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= ParaNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数

							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCPara2[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCPara2[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}

							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);

							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

					else if(seruart->ReAddrH == 0x20){//查询 MCStatus[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= StatusNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{

							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数
							Modbus_20_Return();
							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCStatus[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCStatus[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}
							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

					else if(seruart->ReAddrH == 0x60 || seruart->ReAddrH == 0x30){//查询 SPStatus[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= SpeNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数
							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = SPStatus[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = SPStatus[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}
							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

				break;
				case 6://更改				
					seruart->UpdateRegH = seruart->R_DATA[2];
					seruart->UpdateRegL = seruart->R_DATA[3];
					seruart->UpdatePara = (seruart->R_DATA[4] << 8) + (seruart->R_DATA[5]);
					//更改参数
					if(
						seruart->UpdateRegH == 0x10
					){
						if(seruart->UpdateRegL >= ParaNum){
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else {
							if((seruart->UpdatePara > 32767) || (((seruart->UpdateRegL == 22U) || (seruart->UpdateRegL == 24U)) &&
                                (seruart->UpdatePara != Protect_ValidateTime(seruart->UpdatePara)))){ /* 旧维护地址也不得写入越界保护时间。 */
								ModbusErrorResponse(USARTx,seruart,0x83,3);
							}else {
								MCPara2[seruart->UpdateRegL] = seruart->UpdatePara;
								ModbusResponse(USARTx,seruart);
							}
						}
					}
					//特殊指令
					else if(
						seruart->UpdateRegH == 0x60
						|| seruart->UpdateRegH == 0x30
					){    
						switch(seruart->UpdateRegL){
							case 0x00://启停

								ModbusResponse(USARTx,seruart);
							break;
							case 0x01://方向
								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x02://清除错误
								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x03://保存
								if((App.FB.AllRun== 0)&&(App.FB2.AllRun== 0)){
									if(seruart->UpdatePara == 1){
										FLASH_Write(TEST_FLASH_ADDRESS_START2,MCPara2,ParaNum);
									}else if(seruart->UpdatePara == 3){
										F4Flash_SMCPara_Erase(TEST_FLASH_ADDRESS_START2);
									}
									ModbusResponse(USARTx,seruart);
								}else {
									ModbusErrorResponse(USARTx,seruart,0x83,5);
								}
								
							break;
								
							case 0x04://退出指令控制

								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x0A:
								
								ModbusResponse(USARTx,seruart);
							break;
							
							
							default:
								ModbusErrorResponse(USARTx,seruart,0x83,4);
							break;	
						}
					} 
				break;
				default:
					ModbusErrorResponse(USARTx,seruart,0x83,1);
				break;			 
			}
		}		
		
		else if(	//私有协议：判断有无刷、有无Hall、刹车类型、保护电流
			(((seruart->Addr == 0xAAU) && (seruart->RxLen == MOTOR_COMMAND_FRAME_LENGTH)) ||
             ((seruart->Addr == MOTOR_PROTECTION_FRAME_ADDRESS) && (seruart->RxLen == MOTOR_PROTECTION_FRAME_LENGTH))) /* 地址和长度必须严格配对。 */
            && (seruart->CalcCRC == seruart->RxCRC) /* 两项时间也必须通过CRC，坏帧不能续租或更改保护。 */
		){

			if(MotorCommand_IsSemanticallyValid(seruart) == 0U)
			{
				return; /* CRC虽正确但字段组合非法时整帧丢弃，不能续租也不能改变电机状态。 */
			}

            command_primask = __get_PRIMASK(); /* 从发布目标到发布模式均在同一短临界区内，不能混用新旧控制字段。 */
            __disable_irq(); /* 不允许ADC或500us状态机看到半份运行命令。 */
            Protect_ApplyCommandTimes(seruart->R_DATA[3],
                (seruart->Addr == MOTOR_PROTECTION_FRAME_ADDRESS) ? (uint16_t)(((uint16_t)seruart->R_DATA[9] << 8U) | seruart->R_DATA[10]) : MOTOR_OVERCURRENT_TIME_DEFAULT_MS,
                (seruart->Addr == MOTOR_PROTECTION_FRAME_ADDRESS) ? (uint16_t)(((uint16_t)seruart->R_DATA[11] << 8U) | seruart->R_DATA[12]) : MOTOR_OVERCURRENT_TIME_DEFAULT_MS,
                MotorCommand_AreChannelsQuiescent()); /* 新帧与运动状态同一临界区发布；旧帧明确500/500，重复帧不清计数。 */
            if (s_motor_stop_fault != 0U)
            {
                App.Logic.HandShakeProtCnt = 0U; /* 合法帧仍允许故障通信，但不能解除停机锁存。 */
                MotorCommand_CloseOutputs(E_StopTimeout); /* 不论帧中要求零速还是RUN，保持实际输出关闭。 */
                goto motor_command_reply; /* 恢复中断后上报锁存故障。 */
            }
            App.Logic.Set_Spd = (uint32_t)(((seruart->R_DATA[4] << 8) + seruart->R_DATA[5]) * 10); /* 在续租前解析目标值，用于故障后零速解锁。 */
			if((s_motor_command_zero_required != 0U) && (App.Logic.Set_Spd != 0U))
			{
				App.Logic.Set_Spd = 0U; /* 拒绝恢复前的非零命令时立即清回零速，防止返回前短暂发布旧 RUN 目标。 */
				goto motor_command_reply; /* 未完成零速门禁时只回应状态，恢复中断后禁止旧RUN输出。 */
			}
			if(App.Logic.Set_Spd == 0U)
			{
				s_motor_command_zero_required = 0U; /* 合法零速帧完成重新武装，后续新的非零控制帧才允许启动。 */
			}

			if(App.FB.Err == E_HandShake || App.FB2.Err == E_HandShake || App2.Err == E_HandShake){
				App2.Err = E_NONE; /* 只在通过CRC、语义和零速门禁后清有刷通信故障。 */
				App.FB.Err = E_NONE; /* 清无刷通道1通信故障，状态机随后回到等待态。 */
				App.FB2.Err = E_NONE; /* 清无刷通道2通信故障。 */
			}
			App.Logic.HandShakeProtCnt = 0U; /* 只有完整合法控制帧才能刷新200ms控制租约。 */
			if(App.Logic.Set_Spd == 0U){
				App2.Err = E_NONE; /* 保持原协议：合法零速帧允许清除可恢复错误。 */
				App.FB.Err = E_NONE; /* 零速帧清无刷通道1错误。 */
				App.FB2.Err = E_NONE; /* 零速帧清无刷通道2错误。 */
			}
            if (s_motor_stop_pending != 0U)
            {
                App.Logic.Set_Spd = 0U; /* 当前停机未完成，后来的RUN也不能覆盖零速目标。 */
                goto motor_command_reply; /* 重复零速不重置制动标记、计时或正在执行的定位。 */
            }
            if (App.Logic.Set_Spd == 0U)
            {
                if (MotorCommand_AreChannelsQuiescent() != 0U)
                {
                    YSorWSFlag = (seruart->R_DATA[3] <= 2U) ? 1U : 2U; /* 空闲零速可以为即将启动的驱动类型重新武装。 */
                    App.Logic.Motor_Kind = seruart->R_DATA[3]; /* 只在实际空闲时更新停止反馈的通道。 */
                    App.Logic.LogicHall_or_pll = seruart->R_DATA[6]; /* 保留下一次启动需要的反馈类型。 */
                    App2.Log.Channel = seruart->R_DATA[3]; /* 有刷空闲反馈使用本次选择的物理通道。 */
                    App2.Log.Drill = (seruart->R_DATA[6] == 4U) ? 2U : 1U; /* 保留有刷刀头选择含义。 */
                }
                MotorCommand_BeginStop(); /* 所有外部零速只通过这个入口创建或完成停机。 */
                goto motor_command_reply; /* 不能再落入旧零速分支清除正在使用的刹车状态。 */
            }
            if ((((YSorWSFlag == 1U) && (seruart->R_DATA[3] >= 3U)) ||
                 ((YSorWSFlag == 2U) && (seruart->R_DATA[3] <= 2U))) &&
                (MotorCommand_AreChannelsQuiescent() == 0U))
            {
                MotorCommand_BeginStop(); /* 有刷与无刷切换先完成原通道停机，后续周期再接受新类型RUN。 */
                goto motor_command_reply; /* 当前帧不能同时关闭旧电机和打开新电机。 */
            }
            if ((YSorWSFlag == 2U) && (seruart->R_DATA[3] >= 3U) &&
                (App2.Log.Channel != seruart->R_DATA[3]) && (MotorCommand_AreChannelsQuiescent() == 0U))
            {
                MotorCommand_BeginStop(); /* 有刷切换物理通道先停当前通道，不能让旧输出改用新通道参数。 */
                goto motor_command_reply; /* 下一个合法周期在停止完成后再启动新通道。 */
            }
            if (MotorCommand_GetStopLimit(seruart->R_DATA[3], seruart->R_DATA[1], seruart->R_DATA[6]) == 0U)
            {
                MotorCommand_CloseOutputs(E_StopParam); /* 无法保证停机的参数组不得产生启动输出。 */
                goto motor_command_reply; /* 通过原12字节反馈上报配置错误，不更改帧长。 */
            }
			if(YSorWSFlag == 1){						//无刷
				if(seruart->R_DATA[3] != 1 && seruart->R_DATA[3] != 2){
					App.Logic.Set_Spd = 0;
					if(App.FB.Status == HS_WAIT && App.FB2.Status == HS_WAIT &&(seruart->R_DATA[3] == 3|| seruart->R_DATA[3] == 4)){
						YSorWSFlag = 2;
						App.Logic.Set_Spd =(uint32_t)(((seruart->R_DATA[4]<<8)+seruart->R_DATA[5])*10);		//设置转速
					}
				}
			}else if (YSorWSFlag == 2){			//有刷
				if(seruart->R_DATA[3] != 3 && seruart->R_DATA[3] != 4)
				{
					App.Logic.Set_Spd = 0;
					if(App2.Ch1.Status == CTLS_WAIT && App2.Ch2.Status == CTLS_WAIT  &&(seruart->R_DATA[3] == 1|| seruart->R_DATA[3] == 2)  ){
						YSorWSFlag = 1;
						App.Logic.Set_Spd =(uint32_t)(((seruart->R_DATA[4]<<8)+seruart->R_DATA[5])*10);		//设置转速
					}
				}
			}
			//设置保护电流
			seruart->tempcurrent = (float)(((seruart->R_DATA[7]<<8) +seruart->R_DATA[8])/100.0f);

				if(YSorWSFlag == 1){	//对无刷进行操作
				if(App.Logic.Set_Spd >=MCPara[13]*10){
					App.Logic.Set_Spd = MCPara[13]*10;
				}
				App.Logic.u8WfFru = seruart->R_DATA[2]*2;		//更改速率

				//首先判断电机是否在运行---不在运行就赋值启动，在运行就判断是否要刹车
				if(MotorCommand_AreChannelsQuiescent() != 0U){ /* 预充电、启动和定位都不能冒充空闲直接改模式。 */
						App.Logic.CtlMode 	 = seruart->R_DATA[1];			//赋值电机模式
						App.Logic.u8WfFru 	 = seruart->R_DATA[2]*2;			//赋值往返频率
						App.Logic.Motor_Kind = seruart->R_DATA[3];			//赋值电机类型
						App.Logic.LogicHall_or_pll = seruart->R_DATA[6];//赋值霍尔类型
						App.Logic.Brake_Sign = 0;
						App.Logic.Brake_Kind = 0;
                        App.Logic.Brake_StopPwmOK = 0U; /* 新启动之前清除旧制动完成结果。 */
                        App.Logic.Brake_StopTimOK = 0U; /* 清除旧类型切换等待结果。 */
					
						if(App.Logic.Set_Spd > MCPara[13]*100){
								App.Logic.Set_Spd = MCPara[13]*100;			
							}

				}else{	//有其中一个电机正在运动	
					if(App.Logic.Set_Spd != 0){			//判定为正在运行刹车
						if(App.Logic.Motor_Kind == seruart->R_DATA[3]){		//判定电机类型不变
							if(App.Logic.CtlMode == seruart->R_DATA[1]){
								if(App.Logic.CtlMode !=3){										
									if(App.Logic.LogicHall_or_pll != seruart->R_DATA[6]){
										App.Logic.Change_Kind = seruart->R_DATA[3];		//刹车完更改电机类型
										App.Logic.Change_Mode= seruart->R_DATA[1];	//刹车完变换的模式类型
										App.Logic.Change_Hall =seruart->R_DATA[6];	//刹车完变换的Hall类型
										App.Logic.Brake_Sign  = 1;			//去刹车一次
										App.Logic.Brake_Kind  = 4;			//刹车种类为：切换Hall刹车
									}
								}else{
									if(App.Logic.Set_Spd > MCPara[8]*100){
										App.Logic.Set_Spd = MCPara[8]*100;				//模式3限制最高转速
									}
									App.Pos.PosFlag = 1;		//产生一次位置刹车
								}
							}else{
								App.Logic.Change_Kind = seruart->R_DATA[3];		//刹车完更改电机类型
								App.Logic.Change_Mode = seruart->R_DATA[1];	//刹车完变换的模式类型
								App.Logic.Change_Hall = seruart->R_DATA[6];	//刹车完变换的Hall类型
								App.Logic.CtlMode =	App.Logic.Change_Mode;	//提前切换模式
								App.Logic.Brake_Sign = 1;			//去刹车一次
								App.Logic.Brake_Kind = 3;			//刹车种类为：切换电机模式刹车
							}
						}else{																						//判定电机类型改变
							App.Logic.Change_Kind = seruart->R_DATA[3];		//刹车完更改电机类型
							App.Logic.Change_Mode= seruart->R_DATA[1];	//刹车完变换的模式类型
							App.Logic.Change_Hall =seruart->R_DATA[6];	//刹车完变换的Hall类型
							App.Logic.Brake_Sign  = 1;			//去刹车一次		
							App.Logic.Brake_Kind  = 2;			//刹车种类为：切换电机种类刹车				
						}
                    }else{
                        MotorCommand_BeginStop(); /* 兼容类型切换把目标置零时也遵守同一停止事务。 */
                    }
				}

				if(App.Logic.CtlMode == 4 || App.Logic.CtlMode ==5 ){
					App.Logic.CtlMode 	 = seruart->R_DATA[1];			//赋值电机模式
					if(App.Logic.CtlMode != 4 && App.Logic.CtlMode != 5){
						PWM_3l_Stop();
						PWM_3l_Stop2();
					} 
					if(App.Pos.LockPosStatus == 0){		//如果是等待拖动状态就可以拖动，并且锁定拖动的步数
						App.Pos.LockPosStatus = 1;	//准备去拖动
						App.Pos.PosSetSteps = (uint32_t)((seruart->R_DATA[4]<<8)+seruart->R_DATA[5]);
						if(App.Pos.PosSetSteps  > 65535 ){
							App.Pos.PosSetSteps = 65535;
						}
						if(App.Pos.PosSetSteps > 0){
							App.Pos.Start = 1;  					//允许拖动启动
						}
					}else{
					
					}
				}
				if(seruart->tempcurrent == 0){
					//这里可以加入选择电流
				}else{
						App.FB.Prot.OverCurrent = seruart->tempcurrent;
						App.FB2.Prot.OverCurrent = seruart->tempcurrent;
				}
			//无刷返回一帧数据
			/* 控制状态发布完成后，统一在恢复中断的位置回应。 */
			}
			else if(YSorWSFlag == 2){		//对有刷的操作
				//控制模式 01-正转 02-反转 03-往复模式
				App2.Log.CtlMode = seruart->R_DATA[1];		
				if(App2.Log.CtlMode > 3){
					App2.Log.CtlMode = 0;
				}
				//往复模式下频率
				temp = seruart->R_DATA[2];
				App2.Log.u8WfFru = temp*2 ;
				if(App2.Log.u8WfFru > 200){
					App2.Log.u8WfFru = 200;
				}else if(App2.Log.u8WfFru < 1){
					App2.Log.u8WfFru = 1;
				}
				App2.Log.Channel = seruart->R_DATA[3];			//通道选择
				if((App2.Log.Channel != 3)&&(App2.Log.Channel !=4)){			//防止主机发送过来错误的通道
					App2.Log.Channel = 3;
				}
				if(seruart->R_DATA[6] == 3){
					App2.Log.Drill = 1;
				}else if(seruart->R_DATA[6] == 4){
					App2.Log.Drill = 2;
				}else{
					App2.Log.Drill = 1;
				}
				//设定转速
//				App2.Log.u32SetSpd_temp = ((seruart->R_DATA[4] << 8) + seruart->R_DATA[5]);
				App2.Log.u32SetSpd = App.Logic.Set_Spd ;
				if(App2.Log.u32SetSpd > YSPara1[1]*10){
					App2.Log.u32SetSpd = YSPara1[1]*10;
				}else if(App2.Log.u32SetSpd == 0){
					App2.Err = E_NONE;						//清除错误u
					MotorCommand_BeginStop(); /* 兼容分支产生零速时不能清掉正在执行的有刷制动。 */
				}
				temp1 = ((float)((seruart->R_DATA[7]<<8)+seruart->R_DATA[8])/100.0f);
				if(App2.Log.Channel == 3){
					if(temp1 !=0){
						App2.Ch1.Prot.OverCurrent = temp1;		//通道1过流保护
					}
				}else if(App2.Log.Channel == 4){
					if(temp1 !=0){
						App2.Ch2.Prot.OverCurrent =temp1;		//通道2过流保护
					}
				}else{
					//根据系统而定
				}
				//有刷返回一帧指令
				/* 有刷回应同样不能占用控制状态临界区。 */
            }else{
                /* 未选择电机时只退出本次控制帧。 */
            }
        motor_command_reply:
            __set_PRIMASK(command_primask); /* 状态已经一次发布，串口发送期间必须允许ADC与停机监督继续运行。 */
            if (YSorWSFlag == 1U)
            {
                Customize_Return(USARTx, seruart); /* 无刷回应当前实际执行状态和停机故障。 */
            }
            else if (YSorWSFlag == 2U)
            {
                YSCustomize_Return(USARTx, seruart); /* 有刷沿用原12字节回应协议。 */
            }
        }

	}
}


//CRC校验函数
uint16_t CRC_Calc(unsigned char *snd, unsigned char num)
{
    unsigned char i, j;
    unsigned int c,crc=0xFFFF;
    for(i = 0; i < num; i ++)
    {
        c = snd[i] & 0x00FF;
        crc ^= c;
        for(j = 0;j < 8; j ++)
        { 
            if (crc & 0x0001)
            {
                crc>>=1;
                crc^=0xA001;
            }
            else crc>>=1;
        }
    }    
    return(crc);
}



/*
 * 函数功能：按请求版本回复完整反馈，新版额外报告实际采用的两项时间。
 * 输入参数：USARTx为串口；seruart的前10字节已填好运行状态。
 * 返回参数：无；CRC覆盖新反馈时间，不把Flash参数冒充手柄运行值。
 */
static void MotorCommand_SendFeedback(USART_Type* USARTx, MCUART_Type* seruart)
{
    uint16_t time22; /* 当前物理通道实际采用的往复保护时间。 */
    uint16_t time24; /* 当前物理通道实际采用的无霍尔保护时间。 */
    seruart->SendLen = 10U; /* 旧主控仍收到原12字节帧，保持兼容。 */
    if (seruart->Addr == MOTOR_PROTECTION_FRAME_ADDRESS)
    {
        time22 = (uint16_t)(((seruart->T_DATA[3] == 2U) || (seruart->T_DATA[3] == 4U)) ? App.FB2.Prot.OverCurrentCnt2 : App.FB.Prot.OverCurrentCnt2); /* 反馈跟随实际物理通道。 */
        time24 = (uint16_t)(((seruart->T_DATA[3] == 2U) || (seruart->T_DATA[3] == 4U)) ? App.FB2.Prot.OverCurrentCnt3 : App.FB.Prot.OverCurrentCnt3); /* 不把请求值代替实际缓存值。 */
        seruart->T_DATA[0] = MOTOR_PROTECTION_FRAME_ADDRESS; /* 新地址确认驱动支持完整运行帧。 */
        seruart->T_DATA[10] = (uint8_t)(time22 >> 8U); /* 参数22大端高字节。 */
        seruart->T_DATA[11] = (uint8_t)time22; /* 参数22大端低字节。 */
        seruart->T_DATA[12] = (uint8_t)(time24 >> 8U); /* 参数24大端高字节。 */
        seruart->T_DATA[13] = (uint8_t)time24; /* 参数24大端低字节。 */
        seruart->SendLen = 14U; /* 新回复前14字节全部参与CRC。 */
    }
    seruart->TxCRC = CRC_Calc(seruart->T_DATA, seruart->SendLen); /* 校验整个实际反馈版本。 */
    seruart->T_DATA[seruart->SendLen] = (uint8_t)seruart->TxCRC; /* CRC低字节先发。 */
    seruart->T_DATA[seruart->SendLen + 1U] = (uint8_t)(seruart->TxCRC >> 8U); /* CRC高字节后发。 */
    Modbus_Send(USARTx, seruart->T_DATA, seruart->SendLen + 2U); /* 恢复中断后发送，不阻塞保护调度。 */
}

/*
 * 函数功能：组装无刷运行反馈，按请求版本发送实际保护时间。
 * 输入参数：USARTx为串口；seruart为本次已校验的请求和回复缓存。
 * 返回参数：无。
 */
void Customize_Return(USART_Type* USARTx,MCUART_Type* seruart){
	uint32_t temp = 0;
//	uint32_t temp1 = 0;
//	uint32_t temp2 = 0;
	uint32_t temp3 = 0;
//	temp1 = (uint32_t)(App.FB.Prot.OverCurrent*100);
//	temp2 = (uint32_t)(App.FB2.Prot.OverCurrent*100);
	temp3 = (uint32_t)(App.FB.Prot.AllCur	*100);
	seruart->T_DATA[0] = 0xAA;									//帧头
	seruart->T_DATA[1] = App.Logic.CtlMode;			//控制模式
	seruart->T_DATA[2] = App.Logic.u8WfFru;			//往复频率
	seruart->T_DATA[3] = App.Logic.Motor_Kind;	//电机选择
	if(App.Logic.Motor_Kind == 1){
		seruart->T_DATA[7] = App.FB.Err;
		if(App.FB.Spd_Now >= 0){
			temp = App.FB.Spd_Now/10;							//倍率/10
		}else{
			temp = -App.FB.Spd_Now/10;						//倍率/10
		}
	}else if(App.Logic.Motor_Kind == 2){ 
		seruart->T_DATA[7] = App.FB2.Err;
		if(App.FB2.Spd_Now >= 0){
			temp = App.FB2.Spd_Now/10;					//倍率/10
		}else{
			temp = -App.FB2.Spd_Now/10;					//倍率/10
		}
	}
	seruart->T_DATA[4] = (temp>>8);
	seruart->T_DATA[5] = temp & 0x00FF;
	seruart->T_DATA[6] = App.Logic.LogicHall_or_pll;
//	if(App.Logic.Motor_Kind == 1){
//		seruart->T_DATA[8] =	(uint16_t) ((temp1&0x0000FF00)>>8);	
//		seruart->T_DATA[9] =	(uint16_t) ((temp1)&0x000000FF);
//	}else if(App.Logic.Motor_Kind ==2){
//		seruart->T_DATA[8] =	(uint16_t) ((temp2&0x0000FF00)>>8);
//		seruart->T_DATA[9] =	(uint16_t) ((temp2)&0x000000FF);		
//	}
	seruart->T_DATA[8] =	(uint16_t) ((temp3&0x0000FF00)>>8);
  seruart->T_DATA[9] =	(uint16_t) ((temp3)&0x000000FF);	

	MotorCommand_SendFeedback(USARTx, seruart); /* 新旧版本共用状态字段，实际时间与CRC由统一出口追加。 */
}


/*
 * 函数功能：组装有刷运行反馈并确认新协议能力，参数22/24不改变有刷保护规则。
 * 输入参数：USARTx为串口；seruart为本次请求和回复缓存。
 * 返回参数：无。
 */
void YSCustomize_Return(USART_Type* USARTx,MCUART_Type* seruart){
	uint32_t temp = 0;
	uint32_t temp1 = 0;
	uint32_t temp2 = 0;
	temp1 = (uint32_t)(App2.Ch1.ADC.CurLPF*100);		//通道1回传ADC滤波后的实际电流，单位0.01A
	temp2 = (uint32_t)(App2.Ch2.ADC.CurLPF*100);		//通道2回传ADC滤波后的实际电流，单位0.01A
	
	seruart->T_DATA[0] = 0xAA;									//帧头
	seruart->T_DATA[1] = App2.Log.CtlMode;			//控制模式
	seruart->T_DATA[2] = App2.Log.u8WfFru;			//往复频率
	seruart->T_DATA[3] = App2.Log.Channel;	//电机选择
	seruart->T_DATA[7] = App2.Err;
	
	temp = App2.SysCtl.SpdNowLPF/10;					//倍率/10
	
	seruart->T_DATA[4] = (temp>>8);
	seruart->T_DATA[5] = temp & 0x00FF;
	seruart->T_DATA[6] = App2.Log.Drill;
	if(App2.Log.Channel == 3){
		seruart->T_DATA[8] =	(uint16_t) ((temp1&0x0000FF00)>>8);	
		seruart->T_DATA[9] =	(uint16_t) ((temp1)&0x000000FF);
	}else if(App2.Log.Channel == 4){
		seruart->T_DATA[8] =	(uint16_t) ((temp2&0x0000FF00)>>8);
		seruart->T_DATA[9] =	(uint16_t) ((temp2)&0x000000FF);		
	}

	MotorCommand_SendFeedback(USARTx, seruart); /* 新旧版本共用状态字段，实际时间与CRC由统一出口追加。 */
}

//modbus 指令错误返回
void ModbusErrorResponse(USART_Type* USARTx,MCUART_Type* seruart,uint8_t type,uint8_t err){
	seruart->T_DATA[0] = seruart->Addr;
	seruart->T_DATA[1] = type;
	seruart->T_DATA[2] = err;

	seruart->SendLen = 3;
	seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
	seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
	seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;     
	seruart->TxLen = seruart->SendLen + 2;
	Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
	
}

void Modbus_20_Return(void){
	MCStatus[0] = App.FB.AllRun;//0，运行状态
	MCStatus[1] = myabs(App.FB.Spd_Now);//1，
	MCStatus[2] = App.FB.Prot.BusVol * 10.0f;//2，
	MCStatus[3] = App.FB.Prot.AllCur * 10.f;//3，
	MCStatus[4] = App.FB.Err;//4，
	MCStatus[5] = 0;//5，
	MCStatus[6] = App.FB.Dir.Now;//6，
	MCStatus[7] = 0;//7，
	MCStatus[8] = 0;//8，
	MCStatus[9] = 0;//9,
	
}



//modbus指令成功返回
void ModbusResponse(USART_Type* USARTx,MCUART_Type* seruart){
	seruart->T_DATA[0] = seruart->Addr;
	seruart->T_DATA[1] = seruart->FunCode;
	seruart->T_DATA[2] = seruart->R_DATA[2];
	seruart->T_DATA[3] = seruart->R_DATA[3];
	seruart->T_DATA[4] = seruart->R_DATA[4];
	seruart->T_DATA[5] = seruart->R_DATA[5];

	seruart->SendLen = 6;
	seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
	seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
	seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;     
	seruart->TxLen = seruart->SendLen + 2;
	
	Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
}


