#include "fangbo.h"
#include "hallfoc.h"
#include "tim.h"
#include "mclib.h"
#include "gpio.h"
#include "math.h"
#include "mcfoc.h"
#include "mcflash.h"
#include "adc.h"
#include "fangbo_nohall.h"
#include "at32_board.h"
#include "YSstatemachine.h"
#include "YSprotect.h"
#include "mcuart.h" /* 查询外部停止和输出锁定，阻止启动或换相覆盖停机。 */


#define NOHALL2_SPEED_MIN_AGE_TICKS 1000UL /* 5us计数下的5ms下限，只限制PI测速年龄，不作为保护期限。 */
#define NOHALL2_SPEED_MAX_AGE_TICKS 20000UL /* 5us计数下的100ms上限，原80000计数堵转保护保持独立。 */
#define NOHALL2_SPEED_ALL_SECTORS 0x3FU /* 六位分别表示本轮已经取得六个真实换相区间。 */

static volatile uint32_t s_nohall2_speed_version; /* 奇数表示换相中断正在更新，读者直接放弃本次使用，不等待中断。 */
static volatile uint32_t s_nohall2_speed_age_limit = NOHALL2_SPEED_MAX_AGE_TICKS; /* 最近六个真实区间最大值的三倍，限制在5至100ms。 */
static volatile uint8_t s_nohall2_speed_mask; /* 只由真实换相记录置位，开环强制换相不能放行PI。 */
static volatile uint8_t s_nohall2_skip_interval; /* 切入闭环后的首段跨越开环边界，丢弃后再收集完整区间。 */
static volatile uint8_t s_nohall2_pi_track_pending; /* 首次有效测速或恢复时只跟踪一次当前PWM，避免PI比例项造成接管跳变。 */
static volatile uint8_t s_nohall2_speed_ready; /* ADC已经用完整有效区间更新调速反馈，供500us调速入口检查。 */

/*
 * 函数功能：在B路启动和闭环接管前清除新增测速记录，不改原保护计数或测速数组。
 * 输入参数：无。
 * 返回参数：无。
 */
void NoHall2_ResetSpeedValidity(void)
{
    ++s_nohall2_speed_version; /* 暂停读者使用记录，避免看到复位到一半的有效性状态。 */
    s_nohall2_speed_mask = 0U; /* 本次运行必须重新收齐六个真实区间。 */
    s_nohall2_speed_age_limit = NOHALL2_SPEED_MAX_AGE_TICKS; /* 尚无完整周期时使用100ms采集上限。 */
    s_nohall2_skip_interval = 1U; /* 首次换相只建立起点，不能把开环拖动间隔算作闭环反馈。 */
    s_nohall2_speed_ready = 0U; /* 原速度值保留给保护，PI暂时禁止使用。 */
    s_nohall2_pi_track_pending = 1U; /* 新启动的首次有效速度必须与当前PWM平滑接管。 */
    ++s_nohall2_speed_version; /* 完整发布复位结果，不屏蔽硬件保护中断。 */
}

/*
 * 函数功能：在B路真实换相写入周期后登记新样本，丢失换相后重新收齐六个区间。
 * 输入参数：hall_val为刚写入的相序1至6，interval为原5us计数器在清零前的读数。
 * 返回参数：无。
 */
void NoHall2_RecordSpeedInterval(uint8_t hall_val, uint32_t interval)
{
    uint32_t max_interval = 0U; /* 只扫描六项已有数组，不新增速度换算或保护计数。 */
    uint32_t cycle_ticks = 0U; /* 用完整六扇区总时间排除虚假超高速，允许单段换相不均匀。 */
    uint32_t max_rpm = (uint32_t)MCPara2[13] * 10UL; /* 使用当前电机最高转速参数，保留与屏幕倍率的区别。 */
    uint32_t age_limit; /* 用32位保存三倍间隔，避免16位乘法截断。 */
    uint8_t i; /* 六区间扫描索引，只访问0至5。 */

    ++s_nohall2_speed_version; /* ADC和PI可能抢占换相中断，奇数版本禁止读取半成品。 */
    if ((s_nohall2_skip_interval != 0U) || (hall_val < 1U) || (hall_val > 6U) ||
        (interval == 0U) || (interval > s_nohall2_speed_age_limit)) /* 零间隔、越界相序或跨越丢失时段不能用于闭环调速。 */
    {
        s_nohall2_speed_mask = 0U; /* 原六段资格全部作废，后续必须重新取得完整周期。 */
        s_nohall2_speed_ready = 0U; /* 即使旧数组仍能算出转速，也不能继续推动PI。 */
        s_nohall2_pi_track_pending = 1U; /* 新数据恢复后重新匹配当前PWM，不直接复用失效前积分。 */
        s_nohall2_skip_interval = 0U; /* 本次边沿已建立新起点，下个区间可以重新登记。 */
        s_nohall2_speed_age_limit = NOHALL2_SPEED_MAX_AGE_TICKS; /* 重采集使用100ms上限，不更改原0.4s保护。 */
    }
    else
    {
        s_nohall2_speed_mask |= (uint8_t)(1U << (hall_val - 1U)); /* 仅标记刚由真实换相写入的区间。 */
        if (s_nohall2_speed_mask == NOHALL2_SPEED_ALL_SECTORS) /* 六项均被本轮真实换相覆盖后才计算年龄限制。 */
        {
            for (i = 0U; i < 6U; ++i) /* 不读取数组相邻字段。 */
            {
                cycle_ticks += App.FB2.OpenLoop.SpdCnt_Arr[i]; /* 累加完整周期，避免以某个短扇区直接拒绝真实换相。 */
                if (App.FB2.OpenLoop.SpdCnt_Arr[i] > max_interval)
                {
                    max_interval = App.FB2.OpenLoop.SpdCnt_Arr[i]; /* 采用最慢区间，容纳一个周期内的换相不均匀。 */
                }
            }
            if ((max_rpm == 0U) || (cycle_ticks < (6000000UL + max_rpm - 1UL) / max_rpm))
            {
                s_nohall2_speed_mask = 0U; /* 完整周期仍快于最高转速两倍才拒绝PI反馈，留出量化与短时超调余量。 */
                s_nohall2_speed_ready = 0U; /* 此检查只撤销测速资格，实际硬件换相和原保护继续执行。 */
                s_nohall2_pi_track_pending = 1U; /* 恢复可信周期后重新平滑接管当前PWM。 */
                ++s_nohall2_speed_version; /* 异常返回前结束本次发布，不能永久留下奇数版本。 */
                return; /* 不把毛刺产生的极高速度交给PI压低输出。 */
            }
            age_limit = max_interval * 3UL; /* 正常周期内允许三个最慢区间的样本年龄。 */
            if (age_limit < NOHALL2_SPEED_MIN_AGE_TICKS)
            {
                age_limit = NOHALL2_SPEED_MIN_AGE_TICKS; /* 高速时至少保留5ms采样余量。 */
            }
            if (age_limit > NOHALL2_SPEED_MAX_AGE_TICKS)
            {
                age_limit = NOHALL2_SPEED_MAX_AGE_TICKS; /* 低速时最多保留100ms，过期只冻结PI。 */
            }
            s_nohall2_speed_age_limit = age_limit; /* 发布与当前六项数组对应的有效年龄。 */
        }
    }
    ++s_nohall2_speed_version; /* 最后发布完整记录，原计数器仍由原换相代码复位。 */
}

/*
 * 函数功能：只读判断B路六项真实测速是否完整且未过期，不改任何原保护变量。
 * 输入参数：无。
 * 返回参数：1表示可用于调速，0表示样本不完整、过期或正在更新。
 */
static uint8_t NoHall2_IsSpeedFresh(void)
{
    uint32_t version = s_nohall2_speed_version; /* 保存读取前版本，识别被ADC或换相更新打断的读取。 */
    uint8_t fresh; /* 本次读取的完整性与年龄判断结果。 */

    if ((version & 1U) != 0U)
    {
        return 0U; /* 不在高优先级中断里等待低优先级写者，避免阻塞保护调度。 */
    }
    fresh = (uint8_t)((s_nohall2_speed_mask == NOHALL2_SPEED_ALL_SECTORS) &&
        (App.FB2.OpenLoop.Spd_NotHallCnt <= s_nohall2_speed_age_limit)); /* 只读原计数器；测速过期绝不替原保护清零。 */
    return (uint8_t)((version == s_nohall2_speed_version) && (fresh != 0U)); /* 版本变化时本周期保守冻结，不接受混合记录。 */
}

#define HALL_OSC_TIMER_PERIOD_TICKS 50000UL /* TMR5每5ms溢出，保留原10MHz分辨率和其他模式配置。 */
#define HALL_OSC_NO_EDGE_TICKS 1200000UL /* 120ms无有效霍尔后清测速；覆盖100rpm附近的正常边沿间隔。 */
#define HALL_OSC_ASSIST_REFERENCE_RPM 3000.0f /* 助力上限按电机目标转速线性缩放，达到此转速后封顶，不在门槛处突然撤销助力。 */
#define HALL_OSC_ASSIST_TICKS 120U /* 500us控制周期累计60ms；每次启动助力只能执行一次。 */
#define HALL_OSC_ASSIST_MAX_PWM 1200U /* 助力阶段最多约16%，并受原通道PWM上限约束。 */
#define HALL_OSC_ASSIST_PWM_STEP 20U /* 每500us增减助力20计数，限制静止启动和电流回退的变化量。 */

typedef struct
{
    uint32_t period[6]; /* 最近六个有效霍尔间隔，保持一电周期平均测速。 */
    uint32_t sum; /* 当前有效间隔总计数，单位0.1us。 */
    volatile uint8_t samples; /* 本次启动已有的有效间隔数；至少两个才允许发布速度。 */
    uint8_t index; /* 六个间隔的循环写入位置。 */
    uint8_t last_hall; /* 独立保存真实霍尔码，不使用ADC中预测补偿过的码。 */
    uint16_t assist_ticks; /* 本次启动已经使用的500us助力节拍。 */
    uint16_t assist_pwm; /* 助力自身的受限PWM目标。 */
    uint8_t assist_active; /* 仅初始化时允许启动助力，运行中失速不自动重新触发。 */
    uint8_t assist_releasing; /* 有效霍尔测速建立后只允许助力下降，本轮数据再丢失也不能重新加力。 */
} HallOscState;

static HallOscState s_hall_osc[2]; /* A/B分别保存测速与助力状态，切换通道不复用旧数据。 */
static volatile uint32_t s_hall_osc_overflow_ticks; /* 单个活动通道共用TMR5的累计溢出时间。 */

/*
 * 函数功能：限定新增逻辑仅处理无刷有霍尔电气往复，其他模式保留原控制路径。
 * 输入参数：无。
 * 返回参数：1表示启用有霍尔往复修正，0表示使用原路径。
 */
static uint8_t HallOsc_IsEnabled(void)
{
    return (uint8_t)((YSorWSFlag == 1U) && (App.Logic.CtlMode == 3U) &&
                     (App.Logic.LogicHall_or_pll == 2U)); /* PXBA/PXBB电气往复由主控选择此模式。 */
}

/*
 * 函数功能：丢弃不连续或超时的霍尔测速历史，避免旧速度进入新一轮启动。
 * 输入参数：state为本路往复状态；motor为本路无刷运行状态。
 * 返回参数：无。
 */
static void HallOsc_Invalidate(HallOscState *state, FANGBO_TYPEDEF *motor)
{
    state->samples = 0U; /* 撤销有效性，重新收集连续霍尔间隔。 */
    state->index = 0U; /* 新一轮从环形缓冲首项写入。 */
    state->sum = 0U; /* 丢弃上一方向或异常之前的累计时间。 */
    state->last_hall = 0U; /* 下一有效边沿只建立计时基准，不把等待时间当成转动时间。 */
    motor->Hall_Timer_Sum_Now = 0U; /* 旧周期值不再提供给速度环。 */
    motor->Spd_Enable = 1.0f; /* ADC在收到本轮有效测速前按零速处理。 */
}

/*
 * 函数功能：首次启动和每次换向分别复位霍尔测速，并准备一次从零缓升、限时退出的往复助力。
 * 输入参数：channel为无刷通道编号1或2。
 * 返回参数：无。
 */
static void HallOsc_Reset(uint8_t channel)
{
    HallOscState *state = &s_hall_osc[channel - 1U]; /* 调用点固定传入本路编号。 */
    FANGBO_TYPEDEF *motor = (channel == 1U) ? &App.FB : &App.FB2; /* 仅复位当前启动的通道。 */
    uint32_t saved_mask; /* 保存进入前的中断状态，不能擅自打开原已屏蔽的中断。 */
    if (HallOsc_IsEnabled() == 0U)
    {
        state->assist_active = 0U; /* 单向、无霍尔和定位模式不使用启动助力。 */
        return; /* 不改变其他模式的计时器和测速历史。 */
    }
    saved_mask = __get_PRIMASK(); /* 防止霍尔边沿与软件复位交叉更新同一计数。 */
    __disable_irq(); /* 仅在清理计数与标志期间屏蔽中断。 */
    HallOsc_Invalidate(state, motor); /* 清本次换向之前的有效测速。 */
    motor->Cyc_Hall = 0U; /* 清兼容计数，避免旧六边沿累计残留。 */
    motor->Hall_Timer_Cnt = 0U; /* 清旧的单边沿间隔。 */
    motor->Hall_Timer_Sum = 0U; /* 清旧周期总计数。 */
    motor->Spd_Now = 0.0f; /* 启动时不得向保护和控制环发布上一方向的速度。 */
    motor->Rad_Now = 0.0f; /* 角速度与转速同时失效。 */
    s_hall_osc_overflow_ticks = 0U; /* 从本次启动重新累计TMR5溢出。 */
    TMR_SetCounter(TMR5, 0U); /* 清硬件计数，与软件高位保持同一基准。 */
    TMR_ClearITPendingBit(TMR5, TMR_INT_Overflow); /* 旧溢出不能计入新启动。 */
    state->assist_ticks = 0U; /* 每次合法启动重新拥有有限的60ms助力窗口。 */
    state->assist_pwm = 0U; /* 不继承上一方向的助力，首次输出仍由原PI和PWM斜率决定。 */
    state->assist_releasing = 0U; /* 新启动尚未建立可靠测速，后续只允许一次由升到降的切换。 */
    state->assist_active = (uint8_t)(motor->Spd_Set > 0.0f); /* 正目标均使用连续的助力上限，高速PI足够时不抬高其输出。 */
    __set_PRIMASK(saved_mask); /* 恢复原中断状态。 */
}

/*
 * 函数功能：累计有霍尔往复的TMR5溢出，并将真正长期无边沿的测速置为无效。
 * 输入参数：无。
 * 返回参数：1表示已处理往复分支，0表示中断继续执行原模式逻辑。
 */
uint8_t HallOsc_TimerOverflow(void)
{
    uint32_t saved_mask; /* 溢出与霍尔捕获必须原子地交换高位计数。 */
    uint8_t channel = App.Logic.Motor_Kind; /* 计时器只属于当前活动无刷通道。 */
    if (HallOsc_IsEnabled() == 0U)
    {
        return 0U; /* 其他模式保留原溢出处理。 */
    }
    saved_mask = __get_PRIMASK(); /* 保存调用现场的屏蔽状态。 */
    __disable_irq(); /* 防止更高优先级霍尔中断重复统计同一溢出。 */
    if (TMR_GetINTStatus(TMR5, TMR_INT_Overflow) == SET) /* 捕获中断可能已经消费过该待处理溢出。 */
    {
        TMR_ClearITPendingBit(TMR5, TMR_INT_Overflow); /* 清标志和累计高位必须在同一临界区完成。 */
        if (s_hall_osc_overflow_ticks < HALL_OSC_NO_EDGE_TICKS)
        {
            s_hall_osc_overflow_ticks += HALL_OSC_TIMER_PERIOD_TICKS; /* 补上硬件回卷丢失的5ms。 */
        }
        if ((s_hall_osc_overflow_ticks >= HALL_OSC_NO_EDGE_TICKS) &&
            ((channel == 1U) || (channel == 2U)))
        {
            HallOsc_Invalidate(&s_hall_osc[channel - 1U],
                               (channel == 1U) ? &App.FB : &App.FB2); /* 真正无边沿时恢复零速，原堵转保护继续计时。 */
        }
    }
    __set_PRIMASK(saved_mask); /* 不改变调用前的中断许可。 */
    return 1U; /* 往复分支不能再使用原5ms即清零的规则。 */
}

/*
 * 函数功能：捕获本路真实霍尔边沿，补偿计时器溢出并生成启动期有效测速。
 * 输入参数：channel为触发霍尔中断的无刷通道编号1或2。
 * 返回参数：1表示本边沿已处理或丢弃，0表示调用者使用原模式的测速逻辑。
 */
uint8_t HallOsc_CaptureEdge(uint8_t channel)
{
    HallOscState *state; /* 本路独立的测速历史。 */
    FANGBO_TYPEDEF *motor; /* 本路原速度环与保护共享的反馈。 */
    uint32_t saved_mask; /* 保存捕获前的中断屏蔽状态。 */
    uint32_t elapsed; /* 本边沿距离上一边沿的完整0.1us计数。 */
    uint32_t counter; /* TMR5尚未溢出的低位计数。 */
    uint8_t hall; /* 直接采样的三位霍尔码。 */
    uint8_t expected; /* 当前方向允许出现的下一个霍尔码。 */
    if (HallOsc_IsEnabled() == 0U)
    {
        return 0U; /* 单向和定位等模式保持原实现。 */
    }
    if (((channel != 1U) && (channel != 2U)) || (channel != App.Logic.Motor_Kind))
    {
        return 1U; /* 非活动通道的迟到中断不能清共用计时器。 */
    }
    state = &s_hall_osc[channel - 1U]; /* 在通道校验后访问本路状态。 */
    motor = (channel == 1U) ? &App.FB : &App.FB2; /* 只写当前实际运行的无刷反馈。 */
    if ((motor->AllRun == 0U) || (motor->Status != HS_RUN) || (motor->Err != E_NONE))
    {
        return 1U; /* 预充电、制动和故障期间的边沿不得建立新运行测速。 */
    }
    hall = (channel == 1U) ? gethall : gethall2; /* 不能用ADC人为补齐的换相码测速。 */
    if ((hall == 0U) || (hall == 7U))
    {
        HallOsc_Invalidate(state, motor); /* 非法霍尔码立即撤销测速有效性。 */
        return 1U; /* 保留原故障处理，不生成虚假的转速。 */
    }
    if (hall == state->last_hall)
    {
        return 1U; /* 重复边沿不清计时器，避免将抖动误当成高速运动。 */
    }
    saved_mask = __get_PRIMASK(); /* 软件高位、低位和待处理溢出必须一致。 */
    __disable_irq(); /* 捕获临界区只包含计数读取和基准复位。 */
    elapsed = s_hall_osc_overflow_ticks; /* 读取已经由溢出中断累计的完整周期。 */
    counter = TMR_GetCounter(TMR5); /* 先读取当前低位，随后检查尚未服务的回卷。 */
    if (TMR_GetINTStatus(TMR5, TMR_INT_Overflow) == SET)
    {
        counter = TMR_GetCounter(TMR5); /* 发生回卷时重新读低位，不能混用回卷前的数值。 */
        elapsed += HALL_OSC_TIMER_PERIOD_TICKS; /* 捕获中断先于溢出中断执行时自行补一次高位。 */
    }
    elapsed += counter; /* 合成为完整边沿间隔。 */
    s_hall_osc_overflow_ticks = 0U; /* 下一间隔从零累计高位。 */
    TMR_SetCounter(TMR5, 0U); /* 同步复位硬件低位。 */
    TMR_ClearITPendingBit(TMR5, TMR_INT_Overflow); /* 已计入的溢出不能在退出后再次累计。 */
    __set_PRIMASK(saved_mask); /* 立即恢复原中断状态。 */
    if (state->last_hall == 0U)
    {
        state->last_hall = hall; /* 首个边沿只建立运动时间基准，排除预充电和静止等待。 */
        return 1U; /* 不能把首个不完整扇区用于速度闭环。 */
    }
    expected = (motor->Dir.Now == CW) ? NextCCWhall[state->last_hall] : NextCWhall[state->last_hall]; /* 与原实际换相顺序一致。 */
    if ((hall != expected) || (elapsed == 0U) || (elapsed >= HALL_OSC_NO_EDGE_TICKS))
    {
        HallOsc_Invalidate(state, motor); /* 反向惯性、跳码或超时必须重新建立连续测速。 */
        state->last_hall = hall; /* 当前合法霍尔码作为下一完整间隔的基准。 */
        return 1U; /* 不向速度环提交不可信的速度。 */
    }
    state->last_hall = hall; /* 保存本次真实码，供下一边沿检查顺序。 */
    if (state->samples < 6U)
    {
        ++state->samples; /* 启动时逐步收集样本，不必等待完整一圈才计算平均值。 */
    }
    else
    {
        state->sum -= state->period[state->index]; /* 样本齐全后移除最旧扇区时间。 */
    }
    state->period[state->index] = elapsed; /* 当前间隔替换环形缓冲对应项。 */
    state->sum += elapsed; /* 累加有效扇区的真实时间。 */
    state->index = (uint8_t)((state->index + 1U) % 6U); /* 六个扇区形成稳定的一电周期滑动平均。 */
    motor->Hall_Timer_Cnt = elapsed; /* 保留完整边沿时间供原运行状态读取。 */
    motor->Hall_Timer_Sum_Now = (state->sum * 6U) / state->samples; /* 归一为六扇区总时间，保留原10MHz测速公式。 */
    motor->Spd_Enable = (state->samples >= 2U) ? 0.0f : 1.0f; /* 至少两个连续正确间隔才发布本次测速。 */
    return 1U; /* 本边沿已处理，调用者不得再清计时器或叠加旧周期。 */
}

/*
 * 函数功能：为有霍尔往复叠加随目标转速连续变化的短时助力，可靠测速后缓退，60ms内退出且不改PI积分。
 * 输入参数：channel为通道1或2；requested_pwm为本周期原速度PI输出。
 * 返回参数：受助力幅度、剩余时间和原电流回退约束后的PWM目标，仍由原输出斜率与保护处理。
 */
static int32_t HallOsc_ApplyStartupAssist(uint8_t channel, int32_t requested_pwm)
{
    HallOscState *state = &s_hall_osc[channel - 1U]; /* 两路独立记录助力进度，换向和换通道不能复用旧输出。 */
    FANGBO_TYPEDEF *motor = (channel == 1U) ? &App.FB : &App.FB2; /* 只读取本路运行状态、目标和原保护电流。 */
    mcParam_PIController *pi = (channel == 1U) ? &mcApp_VoltageP_PIParam : &mcApp_VoltageP_PIParam2; /* 保留本路PI计算出的积分，仅同步本次输出记录。 */
    int32_t limit = HALL_OSC_ASSIST_MAX_PWM; /* 沿用原助力绝对上限，也保留电流回退分支的原上限。 */
    int32_t assist_limit; /* 本轮目标转速允许的助力上限，不限制已经更高的正常PI需求。 */
    uint32_t remaining_limit; /* 剩余节拍允许的助力值，保证按原下降步长能在60ms内归零。 */
    if ((state->assist_active == 0U) || (HallOsc_IsEnabled() == 0U))
    {
        return requested_pwm; /* 无霍尔、单向和已结束助力的路径逐值保留原PI输出。 */
    }
    if ((motor->Start == 0U) || (motor->AllRun == 0U) || (motor->Status != HS_RUN) ||
        (motor->Err != E_NONE) || (App.Logic.Brake_Sign != 0U) ||
        (MotorCommand_IsStopRequested() != 0U) || (MotorCommand_IsOutputLocked() != 0U) ||
        (motor->Spd_Set <= 0.0f) || (motor->Prot.OverCurrent <= 0.0f))
    {
        state->assist_active = 0U; /* 停机、故障、换模式或无有效电流门限立即撤销助力。 */
        return requested_pwm; /* 输出关断仍由已有停机与保护路径优先执行。 */
    }
    if (state->assist_ticks >= HALL_OSC_ASSIST_TICKS)
    {
        state->assist_active = 0U; /* 本次启动的60ms期限不延长，运行中不会重新触发。 */
        return requested_pwm; /* 助力正常退到零后完整交回原PI。 */
    }
    if (pi->qOutMax < (float)limit)
    {
        limit = (int32_t)pi->qOutMax; /* 原通道输出上限更小时仍使用原配置。 */
    }
    if ((limit <= 0) || (pi->qOutMin > (float)limit))
    {
        state->assist_active = 0U; /* 特殊PWM范围不允许本辅助逻辑覆盖。 */
        return requested_pwm; /* 保留原配置和原速度环行为。 */
    }
    ++state->assist_ticks; /* 每次调用消耗一个原500us节拍，助力退出不会重置计时。 */
    if (state->samples >= 2U)
    {
        state->assist_releasing = 1U; /* 两个连续正确霍尔间隔已能测速，此后只缓退，不再等待固定转速比例。 */
    }
    if (motor->Prot.AllCur >= motor->Prot.OverCurrent * 0.8f)
    {
        requested_pwm = App.Logic.VBusNowPWM - (int32_t)HALL_OSC_ASSIST_PWM_STEP; /* 沿用原80%保护电流处的减力动作，不修改保护阈值。 */
        if (requested_pwm < 0)
        {
            requested_pwm = 0; /* 沿用原电流回退零下限，禁止写入负PWM。 */
        }
        if (requested_pwm > limit)
        {
            requested_pwm = limit; /* 沿用原电流回退的助力绝对上限。 */
        }
        state->assist_pwm = (uint16_t)requested_pwm; /* 下个周期不能恢复电流回退之前的旧助力。 */
    }
    else
    {
        assist_limit = limit; /* 达到参考转速后助力封顶，但正常PI更高时仍保留正常PI。 */
        if (motor->Spd_Set < HALL_OSC_ASSIST_REFERENCE_RPM)
        {
            assist_limit = (int32_t)((float)HALL_OSC_ASSIST_MAX_PWM * motor->Spd_Set / HALL_OSC_ASSIST_REFERENCE_RPM); /* 低目标按比例减小助力，避免低档固定给满1200。 */
            if (assist_limit > limit)
            {
                assist_limit = limit; /* 缩放后仍服从原通道PWM上限。 */
            }
        }
        remaining_limit = (uint32_t)(HALL_OSC_ASSIST_TICKS - state->assist_ticks) * HALL_OSC_ASSIST_PWM_STEP; /* 提前为助力下降保留时间，期限到达时上限必为零。 */
        if ((uint32_t)assist_limit > remaining_limit)
        {
            assist_limit = (int32_t)remaining_limit; /* 没有测速时也在原60ms窗口内平滑退出。 */
        }
        if (state->assist_releasing != 0U)
        {
            state->assist_pwm = (state->assist_pwm > HALL_OSC_ASSIST_PWM_STEP) ?
                (uint16_t)(state->assist_pwm - HALL_OSC_ASSIST_PWM_STEP) : 0U; /* 可靠测速后每500us最多撤去20计数助力，不突然切掉整个输出下限。 */
        }
        else
        {
            state->assist_pwm += HALL_OSC_ASSIST_PWM_STEP; /* 从零按原步长建立短时输出，不直接跳到固定600。 */
        }
        if (state->assist_pwm > (uint16_t)assist_limit)
        {
            state->assist_pwm = (uint16_t)assist_limit; /* 同时落实目标转速、原PWM范围和剩余时间的限制。 */
        }
        if (requested_pwm < (int32_t)state->assist_pwm)
        {
            requested_pwm = state->assist_pwm; /* 仅在原PI不足时补到受限助力，正常更高的PI输出不被截低。 */
        }
        if (((state->assist_releasing != 0U) && (state->assist_pwm == 0U)) ||
            (state->assist_ticks >= HALL_OSC_ASSIST_TICKS))
        {
            state->assist_active = 0U; /* 助力已撤完或期限到达，本轮测速再次丢失也不重新加力。 */
        }
    }
    pi->qOut = (float)requested_pwm; /* 仅同步输出记录；积分继续由原PI及抗饱和计算更新，不灌入助力。 */
    return requested_pwm; /* 原PWM升降斜率、硬件输出门禁及全部保护仍在后续原路径执行。 */
}

//132645 
uint8_t NextCCWhall[7] = {
	0,
	3,//1
	6,//2
	2,//3
	5,//4
	1,//5
	4,//6
};

uint8_t	LastCCWhall[7] = {
	0,
	5,//1
	3,//2
	1,//3
	6,//4
	4,//5
	2,//6
};

//546231
uint8_t NextCWhall[7] = {
	0,
	5,//1
	3,//2
	1,//3
	6,//4
	4,//5
	2,//6
};

uint8_t LastCWhall[7] = {
	0,
	3,//1
	6,//2
	2,//3
	5,//4
	1,//5
	4,//6
};




// 5-4-6-2-3-1
const uint8_t RAMP_TABLE_Hall[7] = {
	5,//0
	4,//1
	6,//2		
	2,//3
	3,//4
	1,//5
	0,//6
};


//方波ADC中断服务函数 
/*
 * 函数功能：采样并换相；有霍尔往复只接受真实捕获确认的速度，保留其他模式原反馈路径。
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_ADCISRTasks(void){
	volatile short phaseVolP = 0;					//采取电压
	volatile uint8_t i = 0;								//速度读取用
	volatile int8_t j=0;
	volatile int8_t y=0;

	phaseVolP = get_Pvcc_adc_value;			//BUCK降压后电压
	App.FB.Prot.BusVolP = (float)phaseVolP * VOLTAGEP_ADC_PVCC_PHY_RATIO;	//BUCK电路输出电压
	
/************************************无霍尔方波************************************************************************/	
	if((App.Logic.LogicHall_or_pll == 1)&&(App.Logic.CtlMode == 1 ||App.Logic.CtlMode == 2 ||App.Logic.CtlMode == 3)){		//无霍尔的ADC中断，在闭环时在这里进行换相，
	//通道1
		if(App.FB.AllRun == 1){
			if(App.FB.OpenLoop.StartHallOkFg == 1 ){	//相序正确
				//计算速度
					for(i = 0 ; i<6 ;i++){
						App.FB.OpenLoop.SpdSum += App.FB.OpenLoop.SpdCnt_Arr[i];				//累计6次换向的全部时间，更加准
					}
					if(App.FB.OpenLoop.SpdSum != 0 ){
						if(App.FB.Dir.Now ==CW){
							 App.FB.RadFilTemp1 =1000000*(2.0f*M_PI)/(App.FB.OpenLoop.SpdSum * 5.0f);				//0.000005
							App.FB.Spd_Now = App.FB.RadFilTemp1*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm
						}else{
							App.FB.RadFilTemp1 = -1000000*(2.0f*M_PI)/(App.FB.OpenLoop.SpdSum * 5.0f);				//0.000005
							App.FB.Spd_Now = App.FB.RadFilTemp1*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm						
						}
						App.FB.Rad_Now = App.FB.RadFilTemp1*0.1f + App.FB.Rad_Now* 0.9f;
						App.FB.OpenLoop.SpdSum = 0;
				}
			}else if(App.FB.OpenLoop.StartHallOkFg == 0){	//相序还不正确
				//计算速度
					for(i = 0 ; i<6 ;i++){
						App.FB.OpenLoop.SpdSum += App.FB.OpenLoop.SpdCnt_Arr[i];				//累计6次换向的全部时间，更加准
					}
					if(App.FB.OpenLoop.SpdSum != 0 ){
					App.FB.Rad_Now =1000000*(2.0f*M_PI)/(App.FB.OpenLoop.SpdSum * 5.0f);				//0.000005
//					if(App.FB.Rad_Now >300){		//切换角速度，防止速度环过冲，启动转速测量误差较大
//						App.FB.Rad_Now = 300;
//					}
					App.FB.Spd_Now = App.FB.Rad_Now*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm
					App.FB.OpenLoop.SpdSum = 0;			
				}
			}
		}else{
				App.FB.Rad_Now = 0;
				App.FB.Spd_Now = 0;
		}
	//通道2
		if(App.FB2.AllRun == 1){
			if(App.FB2.OpenLoop.StartHallOkFg == 1 ){	//相序正确
				//计算速度
					uint32_t speed_sum = 0U; /* 本次独立累加六个区间，避免上次残值进入测速。 */
					for(i = 0 ; i<6 ;i++){ /* 数组只有0至5六项，禁止读到相邻字段。 */
						speed_sum += App.FB2.OpenLoop.SpdCnt_Arr[i]; /* 保留每个区间原5us计时单位。 */
					}
					App.FB2.OpenLoop.SpdSum = speed_sum; /* 六项累加完成后再发布，原方向、换算和滤波不变。 */
					if(App.FB2.OpenLoop.SpdSum != 0 ){
						if(App.FB2.Dir.Now ==CW){
							App.FB2.RadFilTemp2 =1000000*(2.0f*M_PI)/(App.FB2.OpenLoop.SpdSum * 5.0f);				//0.000005
							App.FB2.Spd_Now = App.FB2.RadFilTemp2*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm	

						}else{
							App.FB2.RadFilTemp2 = -1000000*(2.0f*M_PI)/(App.FB2.OpenLoop.SpdSum * 5.0f);				//0.000005
							App.FB2.Spd_Now = App.FB2.RadFilTemp2*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm
						}
						App.FB2.Rad_Now = App.FB2.RadFilTemp2*0.1f + App.FB2.Rad_Now*0.9f;
                    if ((s_nohall2_speed_version & 1U) == 0U) /* 换相记录正在写入时暂缓发布，不能把一次抢占误当成测速丢失。 */
                    {
                        if (NoHall2_IsSpeedFresh() != 0U) /* 仅完整且未过期的真实区间允许调速使用。 */
                        {
                            if (s_nohall2_speed_ready == 0U)
                            {
                                App.FB2.Rad_Now = App.FB2.RadFilTemp2; /* 恢复时用新周期重置滤波起点，避免旧反馈继续压低PI。 */
                            }
                            s_nohall2_speed_ready = 1U; /* 先完成新速度计算，再通知500us调速入口。 */
                        }
                        else
                        {
                            s_nohall2_speed_ready = 0U; /* Spd_Now保留原计算，低速堵转保护的输入规则不变。 */
                        }
                    }
					App.FB2.OpenLoop.SpdSum = 0;
				}
			}else if(App.FB2.OpenLoop.StartHallOkFg == 0){	//相序还不正确
				//计算速度
					uint32_t speed_sum = 0U; /* 本次独立累加六个区间，避免上次残值进入测速。 */
					for(i = 0 ; i<6 ;i++){ /* 数组只有0至5六项，禁止读到相邻字段。 */
						speed_sum += App.FB2.OpenLoop.SpdCnt_Arr[i]; /* 保留每个区间原5us计时单位。 */
					}
					App.FB2.OpenLoop.SpdSum = speed_sum; /* 六项累加完成后再发布，原方向、换算和滤波不变。 */
					if(App.FB2.OpenLoop.SpdSum != 0 ){
					App.FB2.Rad_Now =1000000*(2.0f*M_PI)/(App.FB2.OpenLoop.SpdSum * 5.0f);				//0.000005
					App.FB2.Spd_Now = App.FB2.Rad_Now*(60.0f)/(2.0f*M_PI);		//1对极	
					App.FB2.OpenLoop.SpdSum = 0;			
				}
//					if(App.FB2.Rad_Now >300){		//切换角速度，防止速度环过冲，启动转速测量误差较大
//						App.FB2.Rad_Now = 300;
//					}
			}
		}else{
				App.FB2.Rad_Now = 0;
				App.FB2.Spd_Now = 0;	
		}
	}
/************************************有霍尔方波************************************************************************/	
	else if((App.Logic.LogicHall_or_pll == 2 )&&(App.Logic.CtlMode == 1 ||App.Logic.CtlMode == 2 ||App.Logic.CtlMode == 3)){	//有霍尔的ADC中断，在闭环时在这里进行换相，
		//通道1
		App.FB.HallVal = gethall;			//获取Hall值
		if((App.FB.HallVal !=0)&&(App.FB.HallVal !=7)){			//判定霍尔的值不为0和7才给他判定方向
			if((App.FB.HallVal != App.FB.OldHallVal)&&(App.FB.OldHallVal !=0)){
				if (App.Logic.CtlMode != 3U) /* 往复测速有效性由真实霍尔捕获维护，ADC预测换相不能提前放行。 */
                {
                    App.FB.Spd_Enable =0; /* 单向模式保留原换相恢复测速行为。 */
                }
			}
		}
		//预判Hall处理，防止干扰打断Hall,需要校验本次的HallVal是否正确，如果不正确，那么需要在换向前，替换掉错误的
		if(App.FB.Dir.Now == CW){
			//判断本次hall在CW方向是否一致
			if(	App.FB.OldHallVal == App.FB.HallVal){	//判断正确
				App.FB.OldHallVal =App.FB.HallVal;//传递旧值
			}else if(App.FB.OldHallVal != App.FB.HallVal){
				if(App.FB.HallVal == NextCCWhall[App.FB.OldHallVal]){
					App.FB.OldHallVal = App.FB.HallVal;
				}else{
					App.FB.HallVal = NextCCWhall[App.FB.OldHallVal];
					App.FB.OldHallVal =App.FB.HallVal;
				}
			}
		}else if(App.FB.Dir.Now == CCW){
			//判断本次hall在CW方向是否一致
			if(	App.FB.OldHallVal == App.FB.HallVal){	//判断正确
				App.FB.OldHallVal =App.FB.HallVal;//传递旧值
			}else if(App.FB.OldHallVal != App.FB.HallVal){
				if(App.FB.HallVal == NextCWhall[App.FB.OldHallVal]){
					App.FB.OldHallVal = App.FB.HallVal;
				}else{
					App.FB.HallVal = NextCWhall[App.FB.OldHallVal];
					App.FB.OldHallVal =App.FB.HallVal;
				}
			}
		}
		
	if(App.FB.AllRun == 1){		//在允许启动才给换相

		Hall_PhaseChange(App.FB.HallVal,7000);		//90%
		if((App.FB.Hall_Timer_Sum_Now != 0)&&(App.FB.Spd_Enable == 0)){
			if(App.FB.Dir.Now == CW){
				App.FB.Rad_Now = 10000000*(2.0f*M_PI)/(App.FB.Hall_Timer_Sum_Now);								//16K计算一次角速度
				App.FB.Spd_Now = App.FB.Rad_Now*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm

			}else{
				App.FB.Rad_Now = -10000000*(2.0f*M_PI)/(App.FB.Hall_Timer_Sum_Now);								//16K计算一次角速度
				App.FB.Spd_Now = App.FB.Rad_Now*(60.0f)/(2.0f*M_PI);		//1对极		角速度转化为rpm
			}
		}else{
		App.FB.Rad_Now =0;
		App.FB.Spd_Now =0;	
		}
	}else{
		App.FB.Rad_Now =0;
		App.FB.Spd_Now =0;
	}
	
		//如果电机运行，开始计时
	if(App.FB.AllRun == 1){
		App.FB.FB_Cnt_Time ++;
		if(App.FB.FB_Cnt_Time > 10000)	//1s
		{
			App.FB.FB_Cnt_Time =0;
		}
	}else{
		App.FB.FB_Cnt_Time =0;
	}
/************************************************************************************************************/		
	//电机2
	App.FB2.HallVal = gethall2;
	if((App.FB2.HallVal !=0)||(App.FB2.HallVal!=7)){			//判定霍尔的值不为0和7才给他判定方向
			if((App.FB2.HallVal != App.FB2.OldHallVal)&&(App.FB2.OldHallVal !=0)){
				if (App.Logic.CtlMode != 3U) /* 往复测速有效性由真实霍尔捕获维护，ADC预测换相不能提前放行。 */
                {
                    App.FB2.Spd_Enable =0; /* 单向模式保留原换相恢复测速行为。 */
                }
			}
		}
	
		//预判Hall处理，防止干扰打断Hall,需要校验本次的HallVal是否正确，如果不正确，那么需要在换向前，替换掉错误的
		if(App.FB2.Dir.Now == CW){
			//判断本次hall在CW方向是否一致
			if(	App.FB2.OldHallVal == App.FB2.HallVal){	//判断正确
				App.FB2.OldHallVal =App.FB2.HallVal;//传递旧值
			}else if(App.FB2.OldHallVal != App.FB2.HallVal){
				if(App.FB2.HallVal == NextCCWhall[App.FB2.OldHallVal]){
					App.FB2.OldHallVal = App.FB2.HallVal;
				}else{
					App.FB2.HallVal = NextCCWhall[App.FB2.OldHallVal];
					App.FB2.OldHallVal =App.FB2.HallVal;
				}
			}
		}else if(App.FB2.Dir.Now == CCW){
			//判断本次hall在CW方向是否一致
			if(	App.FB2.OldHallVal == App.FB2.HallVal){	//判断正确
				App.FB2.OldHallVal =App.FB2.HallVal;//传递旧值
			}else if(App.FB2.OldHallVal != App.FB2.HallVal){
				if(App.FB2.HallVal == NextCWhall[App.FB2.OldHallVal]){
					App.FB2.OldHallVal = App.FB2.HallVal;
				}else{
					App.FB2.HallVal = NextCWhall[App.FB2.OldHallVal];
					App.FB2.OldHallVal =App.FB2.HallVal;
				}
			}
		}
		
		
		
		
		
		
		
		
		
		
	if(App.FB2.AllRun == 1){
		Hall_PhaseChange2(App.FB2.HallVal,7000);		//90%
	if((App.FB2.Hall_Timer_Sum_Now != 0)&&(App.FB2.Spd_Enable == 0)){
			
			if(App.FB2.Dir.Now == CW){
				App.FB2.Rad_Now = 10000000*(2.0f*M_PI)/(App.FB2.Hall_Timer_Sum_Now);								//16K计算一次角速度
				App.FB2.Spd_Now = App.FB2.Rad_Now*(60.0f)/(2.0f*M_PI);				//1对极		角速度转化为rpm
			}else{
				App.FB2.Rad_Now = -10000000*(2.0f*M_PI)/(App.FB2.Hall_Timer_Sum_Now);								//16K计算一次角速度
				App.FB2.Spd_Now = App.FB2.Rad_Now*(60.0f)/(2.0f*M_PI);				//1对极		角速度转化为rpm
			}
		}else{
		App.FB2.Rad_Now =0;
		App.FB2.Spd_Now =0;	
		}
	}else{
		App.FB2.Rad_Now =0;
		App.FB2.Spd_Now =0;
	}	
		
  }	
	else if(App.Logic.CtlMode == 4 ){				//位置模式，通过指令来控制他移动的步数

	}
	else if(App.Logic.CtlMode == 5){

	}
	//012345
	//546231
	if(App.Pos.LockFlag1 == 1){			//位置定位过,就需要实时刻刻去考虑Hall的位置，记录钻头的位置正负1圈 
		//电机1
		App.Pos.HallValue1 = gethall;		
		if(App.Pos.HallOldValue1 != App.Pos.HallValue1){
			//更新Hall了
				for(j=0 ;j<6;j++){
					if(App.Pos.HallOldValue1 == RAMP_TABLE_Hall[j]){
						if(RAMP_TABLE_Hall[j+1] == App.Pos.HallValue1){							//判断是否正转
							App.Pos.OutSidePosition1 += App.Pos.OutSideAngle1 ;			//正转路径累计
						}else if(RAMP_TABLE_Hall[j-1] == App.Pos.HallValue1){				//判断是否反转
							App.Pos.OutSidePosition1-= App.Pos.OutSideAngle1 ;			//反转路径累计
						}
						else if(j == 0){			//头
							if(RAMP_TABLE_Hall[5] == App.Pos.HallValue1){							//判断是否反转						
								App.Pos.OutSidePosition1-= App.Pos.OutSideAngle1 ;		//反转路径累计
							}
						}else if(j == 5){			//尾
							if(RAMP_TABLE_Hall[0] == App.Pos.HallValue1){								//判断是否正转
								App.Pos.OutSidePosition1 += App.Pos.OutSideAngle1 ;			//正转路径累计
							}							
						}
						break;			//跳出整个for
					}
				}
				if(App.Pos.OutSidePosition1 >= 1.0000f){					//正一圈溢出
					App.Pos.OutSidePosition1 -= 1.0000f;
				}
				else if(App.Pos.OutSidePosition1 <= -1.0000f){		//负一圈溢出
					App.Pos.OutSidePosition1 += 1.0000f;
				}
			App.Pos.HallOldValue1 = App.Pos.HallValue1;//传递旧值
		}
			App.Pos.PlanPosStep1 =(int16_t) (App.Pos.OutSidePosition1 / App.Pos.OutSideAngle1);			//时时刻刻计算步数

	}
	
	
	if(App.Pos.LockFlag2 == 1){
		//电机2		
		App.Pos.HallValue2 = gethall2;		
		if(App.Pos.HallOldValue2 != App.Pos.HallValue2){
			//更新Hall了
				for(y=0 ;y<6;y++){
					if(App.Pos.HallOldValue2 == RAMP_TABLE_Hall[y]){
						if(RAMP_TABLE_Hall[y+1] == App.Pos.HallValue2){							//判断是否正转
							App.Pos.OutSidePosition2 += App.Pos.OutSideAngle2 ;			//正转路径累计
						}else if(RAMP_TABLE_Hall[y-1] == App.Pos.HallValue2){				//判断是否反转
							App.Pos.OutSidePosition2-= App.Pos.OutSideAngle2 ;			//反转路径累计
						}
						else if(y == 0){			//头
							if(RAMP_TABLE_Hall[5] == App.Pos.HallValue2){							//判断是否反转						
								App.Pos.OutSidePosition2-= App.Pos.OutSideAngle2 ;		//反转路径累计
							}
						}else if(y == 5){			//尾
							if(RAMP_TABLE_Hall[0] == App.Pos.HallValue2){								//判断是否正转
								App.Pos.OutSidePosition2 += App.Pos.OutSideAngle2 ;			//正转路径累计
							}							
						}
						break;			//跳出整个for
					}
				}
				if(App.Pos.OutSidePosition2 >= 1.0000f){					//正一圈溢出
					App.Pos.OutSidePosition2 -= 1.0000f;
				}
				else if(App.Pos.OutSidePosition2 <= -1.0000f){		//负一圈溢出
					App.Pos.OutSidePosition2 += 1.0000f;
				}
			App.Pos.HallOldValue2 = App.Pos.HallValue2;//传递旧值
		}
			App.Pos.PlanPosStep2 =(int16_t) (App.Pos.OutSidePosition2 / App.Pos.OutSideAngle2);			//时时刻刻计算步数

	}

}	
	

/************************************************************************************************************/
//方波上电初始化
void Fangbo_Init(void){
//通道1
	App.FB.Status = HS_WAIT;
	App.FB.AllRun = 0;
	App.FB.Dir.Set =CW;
//通道2	
	App.FB2.Status = HS_WAIT;
	App.FB2.AllRun = 0;
	App.FB2.Dir.Set =CW;
//上电需要给一个初始值	
	App.FB.Prot.OverCurrent  = App.FB.Prot.OverCurrent1;
	App.FB2.Prot.OverCurrent = App.FB2.Prot.OverCurrent1; 
}


//	根据速度去电压闭环
/*
 * 函数功能：执行A路原速度PI与PWM爬坡，仅低速有霍尔往复增加受限启动助力。
 * 输入参数：无。
 * 返回参数：无。
 */
void Hall_Vbus_PI_Out(void){
	volatile int32_t temp_pwm = 0;
	
	
	float PWM_UP 	 = 0;
	float PWM_Down = 0;
	
	if(App.Logic.LogicHall_or_pll == 1){
		PWM_UP 	 = MCPara[64];
		PWM_Down = MCPara[65];
	}else{
		PWM_UP 	 = MCPara[36];
		PWM_Down = MCPara[37];
	}
	
	if(App.FB.AllRun ==1){
		//Buck测量值给定
		if(App.FB.Dir.Now == CW){
			mcApp_VoltageP_PIParam.qInMeas = App.FB.Rad_Now;
		}else{
			mcApp_VoltageP_PIParam.qInMeas = -App.FB.Rad_Now;
		}
		//Buck期望值给定
		mcApp_VoltageP_PIParam.qInRef  = App.FB.Rad_Set;
		//Buck的PI计算
		mcLib_CalcPI(&mcApp_VoltageP_PIParam);

			//输出值给temp；
				temp_pwm = (uint16_t)mcApp_VoltageP_PIParam.qOut;
                temp_pwm = HallOsc_ApplyStartupAssist(1U, temp_pwm); /* 低速往复短时建立起动力矩，随后由原PI连续接管。 */
			//给定期望值，但是需要爬坡
				App.Logic.VBusSetPWM = temp_pwm;

			if((App.Logic.VBusNowPWM - App.Logic.VBusSetPWM) < -PWM_UP){
					App.Logic.VBusNowPWM += PWM_UP;
				}else if( (App.Logic.VBusNowPWM - App.Logic.VBusSetPWM) >PWM_Down){
					App.Logic.VBusNowPWM -=PWM_Down;
				}else {
					App.Logic.VBusNowPWM = App.Logic.VBusSetPWM;
				}
				if(App.Logic.VBusNowPWM >mcApp_VoltageP_PIParam.qOutMax){
					App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam.qOutMax ;
				}else if(App.Logic.VBusNowPWM < mcApp_VoltageP_PIParam.qOutMin){
					App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam.qOutMin ;
				} 
				TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);

	
		}
	else{
//		App.Logic.VBusNowPWM = 0;					//都关
//		App.Logic.VBusSetPWM = 0;
//		mcApp_VoltageP_PIParam.qdSum=0;				//都关
//		mcApp_VoltageP_PIParam.qOut=0;	
//		TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);	//都关	
	}
}



//	根据速度去电压闭环
/*
 * 函数功能：B路无霍尔测速有效时执行原PI，失效时冻结；有霍尔往复保持原受限助力。
 * 输入参数：无。
 * 返回参数：无。
 */
void Hall_Vbus_PI_Out2(void){
	volatile int32_t temp_pwm = 0;
	float PWM_UP2 	 = 0;
	float PWM_Down2 = 0;
    if (App.Logic.LogicHall_or_pll == 1U) /* 本次门禁仅用于B路无霍尔闭环，不改变有霍尔调速。 */
    {
        if ((App.Logic.Motor_Kind != 2U) || (App.FB2.AllRun != 1U) || (App.FB2.Status != HS_RUN) ||
            (App.FB2.Start == 0U) || (App.FB2.Err != E_NONE) || (App.Logic.Brake_Sign != 0U) ||
            (MotorCommand_IsStopRequested() != 0U))
        {
            return; /* 停止、换向和故障交回原调度处理，本入口不能保持或重写硬件输出。 */
        }
        if ((s_nohall2_speed_version & 1U) != 0U)
        {
            App.Logic.VBusSetPWM = App.Logic.VBusNowPWM; /* 仅因正常换相记录正在发布时暂缓一次PI，不重置积分接管状态。 */
            return; /* 等待下一调速周期读取完整记录，高优先级中断不能等待被抢占的写者。 */
        }
        if ((s_nohall2_speed_ready == 0U) || (NoHall2_IsSpeedFresh() == 0U))
        {
            s_nohall2_pi_track_pending = 1U; /* 只标记下次有效反馈需要接管，失效期间不修改原积分。 */
            App.Logic.VBusSetPWM = App.Logic.VBusNowPWM; /* 撤销旧PI目标，最多保持当前PWM；当前为零就继续为零。 */
            return; /* 跳过PI、积分、最小PWM限幅和硬件重写，不主动增加输出；原保护仍独立执行。 */
        }
    }

	
	if(App.Logic.LogicHall_or_pll == 1){
		PWM_UP2 	 = MCPara2[64];
		PWM_Down2 = MCPara2[65];
	}else{
		PWM_UP2 	 = MCPara2[36];
		PWM_Down2 = MCPara2[37];
	}	
	
	
	if(App.FB2.AllRun ==1){
		//Buck测量值给定
		if(App.FB2.Dir.Now == CW){
			mcApp_VoltageP_PIParam2.qInMeas = App.FB2.Rad_Now;
		}else{
			mcApp_VoltageP_PIParam2.qInMeas = -App.FB2.Rad_Now;
		}
		//Buck期望值给定
		mcApp_VoltageP_PIParam2.qInRef  = App.FB2.Rad_Set;
		//Buck的PI计算
        if ((App.Logic.LogicHall_or_pll == 1U) && (s_nohall2_pi_track_pending != 0U))
        {
            mcApp_VoltageP_PIParam2.qErr = mcApp_VoltageP_PIParam2.qInRef - mcApp_VoltageP_PIParam2.qInMeas; /* 用本次新反馈计算接管误差。 */
            mcApp_VoltageP_PIParam2.qdSum = App.Logic.VBusNowPWM - mcApp_VoltageP_PIParam2.qKp * mcApp_VoltageP_PIParam2.qErr; /* 抵消首次比例项，使PI从当前实际PWM连续开始。 */
            if (mcApp_VoltageP_PIParam2.qdSum < 0.0f)
            {
                mcApp_VoltageP_PIParam2.qdSum = 0.0f; /* 保留原PI非负积分限制，不通过负积分改变运行策略。 */
            }
            if (mcApp_VoltageP_PIParam2.qdSum > mcApp_VoltageP_PIParam2.qdSumMax)
            {
                mcApp_VoltageP_PIParam2.qdSum = mcApp_VoltageP_PIParam2.qdSumMax; /* 保留原积分上限，后续PWM仍走原输出限幅和斜率。 */
            }
            if (mcApp_VoltageP_PIParam2.qdSum > mcApp_VoltageP_PIParam2.qOutMax)
            {
                mcApp_VoltageP_PIParam2.qdSum = mcApp_VoltageP_PIParam2.qOutMax; /* 与原PI中的输出上限约束一致。 */
            }
            mcApp_VoltageP_PIParam2.qOut = App.Logic.VBusNowPWM; /* 同步接管输出记录，不直接写硬件。 */
            s_nohall2_pi_track_pending = 0U; /* 本次接管只执行一次，后续恢复原PI积分调速。 */
        }
		mcLib_CalcPI(&mcApp_VoltageP_PIParam2);
		//输出值给temp；
		temp_pwm = (uint16_t)mcApp_VoltageP_PIParam2.qOut;
                temp_pwm = HallOsc_ApplyStartupAssist(2U, temp_pwm); /* 低速往复短时建立起动力矩，随后由原PI连续接管。 */
		
		//给定期望值，但是需要爬坡
		App.Logic.VBusSetPWM = temp_pwm;
		if((App.Logic.VBusNowPWM - App.Logic.VBusSetPWM) < -PWM_UP2){
				App.Logic.VBusNowPWM += PWM_UP2;
			}else if( (App.Logic.VBusNowPWM - App.Logic.VBusSetPWM) >PWM_Down2){
				App.Logic.VBusNowPWM -=PWM_Down2;
			}else {
				App.Logic.VBusNowPWM = App.Logic.VBusSetPWM;
			}
			if(App.Logic.VBusNowPWM >mcApp_VoltageP_PIParam2.qOutMax){
				App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam2.qOutMax ;
			}else if(App.Logic.VBusNowPWM < mcApp_VoltageP_PIParam2.qOutMin){
				App.Logic.VBusNowPWM  = mcApp_VoltageP_PIParam2.qOutMin ;
			} 
			TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);
		}else{
	}
}


//启动参数生成
void RAMP_TABLE_Init(float tim,float cnt){
	//启动拖动最终转速	=	60*1000/20000 /1/6 +0.5f  = 1
	App.FB.OpenLoop.Step_Min_Tim = (60.0f * tim) / MCPara[25] / MCPara[1] / 6 + 0.5f;
	if(App.FB.OpenLoop.Step_Min_Tim < 1){
		App.FB.OpenLoop.Step_Min_Tim = 1;		//最小值1
	}
	App.FB.OpenLoop.Step_Delay_Tim[63] = App.FB.OpenLoop.Step_Min_Tim;		//赋值给
	
	App.FB.OpenLoop.Step_temp_tim = App.FB.OpenLoop.Step_Min_Tim;
	for(App.FB.OpenLoop.StepCnt = 1;App.FB.OpenLoop.StepCnt < 64; App.FB.OpenLoop.StepCnt ++){
		App.FB.OpenLoop.Step_temp_tim = App.FB.OpenLoop.Step_temp_tim + cnt;		//CNT:拖动步进 40/1000 =0.04
		//规划每一步的换相时间！，不断减少
		App.FB.OpenLoop.Step_Delay_Tim[63 - App.FB.OpenLoop.StepCnt] = App.FB.OpenLoop.Step_temp_tim;
		
	}

}



//启动参数生成2
void RAMP_TABLE_Init2(float tim,float cnt){
	//启动拖动最终转速	=	60*1000/20000 /1/6 +0.5f  = 1
	App.FB2.OpenLoop.Step_Min_Tim = (60.0f * tim) / MCPara2[25] / MCPara2[1] / 6 + 0.5f;
	if(App.FB2.OpenLoop.Step_Min_Tim < 1){
		App.FB2.OpenLoop.Step_Min_Tim = 1;		//最小值1
	}
	//[63]是
	App.FB2.OpenLoop.Step_Delay_Tim[63] = App.FB2.OpenLoop.Step_Min_Tim;		//赋值给
	
	App.FB2.OpenLoop.Step_temp_tim = App.FB2.OpenLoop.Step_Min_Tim;
	for(App.FB2.OpenLoop.StepCnt = 1;App.FB2.OpenLoop.StepCnt < 64; App.FB2.OpenLoop.StepCnt ++){
		App.FB2.OpenLoop.Step_temp_tim = App.FB2.OpenLoop.Step_temp_tim + cnt;		//CNT:拖动步进 40/1000 =0.04
		//规划每一步的换相时间！，不断减少
		App.FB2.OpenLoop.Step_Delay_Tim[63 - App.FB2.OpenLoop.StepCnt] = App.FB2.OpenLoop.Step_temp_tim;
	}
}


//方波状态机各种状态

//等待 
void HallSqu_Wait(void){
	if(App.FB.Start == 1){			//Start由LOGIC决定	：条件：无错误、设定速度大于固定值、有方向、启动电压大于固定值、方向不改变
		App.FB.Status = HS_INIT;	//现在是指令启动
	}else{

	}
}



//等待 
void HallSqu_Wait2(void){
	if(App.FB2.Start == 1){
		App.FB2.Status = HS_INIT;	//现在是指令启动
	}else{

	}
}




//初始化
/*
 * 函数功能：处理无刷A路启动初始化，停止请求可以在产生后续输出前取消启动
 * 输入参数：无。
 * 返回参数：无。
 */
/*
 * 函数功能：初始化A路无刷启动；有霍尔往复分别复位测速与助力，其他模式保持原流程。
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Init(void){
	uint8_t i = 0;
    if ((App.FB.Start == 0U) || (MotorCommand_IsStopRequested() != 0U))
    {
        App.FB.AllRun = 0U; /* 启动许可已经撤销，预充电不能继续进入开环。 */
        App.FB.Status = HS_WAIT; /* 本通道直接退出尚未完成的启动阶段。 */
        App.Logic.VBusNowPWM = 0; /* 清启动阶段已经写入的母线值。 */
        TMR_SetCompare4(TMR2, 0U); /* 撤销实际启动供能。 */
        PWM_3l_Stop(); /* 关闭本通道上下桥，避免松脚后补发启动脉冲。 */
        return; /* 不再执行下方任何启动硬件访问。 */
    }
    App.Logic.Brake_StopPwmOK = 0U; /* 每次真正重新启动都清除旧制动结果。 */
    App.Logic.Brake_StopTimOK = 0U; /* 不继承上次切换等待结果。 */


	TIME1_PWM_Stop_3Channel();		//防止有刷一起转
	TIME8_PWM_Stop_3Channel();		//防止有刷一起转
	PVCC_PWM_ON;
	for(i = 0 ; i<6 ;i++){ /* 数组只有六项，启动清理禁止写入相邻内存。 */
		App.FB.OpenLoop.SpdCnt_Arr[i] = 0; /* 清除本路旧周期，不改任何保护计数。 */
	}
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 ||App.Logic.CtlMode ==3 ){
		if(App.Logic.LogicHall_or_pll == 2){	
			HALL2_GPIO_INIT_Disable();		//失能HALL中断
			HALL1_GPIO_INIT();			//有霍尔的话初始化HALL
			TIM3_DISABLE();
		}else{
			TIM_ENABC_Confi();						//无HALL定时器3
			HALL1_GPIO_INIT_Disable();		//失能HALL中断
			HALL2_GPIO_INIT_Disable();		//失能HALL中断
		}
		//启动初始化
		//保护初始化：参数
		if(App.Logic.LogicHall_or_pll == 2){
			if(App.Logic.CtlMode == 3){
				App.FB.Prot.OverCurrent = App.FB.Prot.OverCurrent2;				//有Hall往复运动保护电流
			}else{
				App.FB.Prot.OverCurrent = App.FB.Prot.OverCurrent1;				//有Hall单向运动保护电流
			}
		}else if (App.Logic.LogicHall_or_pll == 1){
			App.FB.Prot.OverCurrent = App.FB.Prot.OverCurrent3;					//无Hall单向运动保护电流
		}
		//速度PI初始化
		VoltageP1_Init();
        HallOsc_Reset(1U); /* 本次有霍尔往复清测速历史并准备有限启动助力；其他模式只撤销助力。 */
	}else if (App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){
		HALL1_GPIO_INIT();			//有霍尔的话初始化HALL
		TIM3_DISABLE();	
		//位置模式保护参数初始化
	}
	//充电
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 ||App.Logic.CtlMode ==3 ){
		if(App.Logic.LogicHall_or_pll == 2){					//有霍尔判断要不要快速去闭环。跳过充电	
			if(App.FB.Chang_Flag ==1){		//如果是快速切换转向，就不用再花多时间去充电，直接跳转闭环运行		
				App.FB.Status = HS_START;	
				App.FB.AllRun = 1;//允许运行
				App.FB.Chang_Flag = 0;		//清
			}else{
				App.FB.Status = HS_CHARGE;	//充电
				App.FB.ChaTim1msCnt = 0;		//清除充电时间
			}
		}else{
				App.FB.Status = HS_CHARGE;	//充电
				App.FB.ChaTim1msCnt = 0;		//清除充电时间		
		}
	}else if (App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){		//充电
			App.FB.Status = HS_POSDRAG;	//充电
			App.FB.ChaTim1msCnt = 0;		//清除充电时间		
	}
	App.FB.Dir.Now = App.FB.Dir.Set ;		//给设定的方向
	
	if(App.FB.Dir.Now == CW){
		App.FB.HallVal = gethall;
		App.FB.OldHallVal = LastCWhall[App.FB.HallVal];//传递旧值
	}else{
		App.FB.HallVal = gethall;
		App.FB.OldHallVal = LastCCWhall[App.FB.HallVal];//传递旧值
	}

	
}


//初始化
/*
 * 函数功能：处理无刷B路启动初始化，停止请求可以在产生后续输出前取消启动
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Init2(void){
	uint8_t i = 0;
    if ((App.FB2.Start == 0U) || (MotorCommand_IsStopRequested() != 0U))
    {
        App.FB2.AllRun = 0U; /* 启动许可已经撤销，预充电不能继续进入开环。 */
        App.FB2.Status = HS_WAIT; /* 本通道直接退出尚未完成的启动阶段。 */
        App.Logic.VBusNowPWM = 0; /* 清启动阶段已经写入的母线值。 */
        TMR_SetCompare4(TMR2, 0U); /* 撤销实际启动供能。 */
        PWM_3l_Stop2(); /* 关闭本通道上下桥，避免松脚后补发启动脉冲。 */
        return; /* 不再执行下方任何启动硬件访问。 */
    }
    App.Logic.Brake_StopPwmOK = 0U; /* 每次真正重新启动都清除旧制动结果。 */
    App.Logic.Brake_StopTimOK = 0U; /* 不继承上次切换等待结果。 */

	TIME1_PWM_Stop_3Channel();		//防止有刷一起转
	TIME8_PWM_Stop_3Channel();		//防止有刷一起转
	PVCC_PWM_ON;
	for(i = 0 ; i<6 ;i++){ /* 数组只有六项，启动清理禁止写入相邻内存。 */
		App.FB2.OpenLoop.SpdCnt_Arr[i] = 0; /* 清除本路旧周期，不改任何保护计数。 */
	}
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 ||App.Logic.CtlMode ==3 ){
		if(App.Logic.LogicHall_or_pll == 2){		//有霍尔
			HALL1_GPIO_INIT_Disable();		//失能HALL中
			HALL2_GPIO_INIT();
			TIM3_DISABLE();
		}else{
//			TIM_HALL_Disable();						//失能定时器5中断
			TIM_ENABC_Confi();						//无HALL定时器3
			HALL1_GPIO_INIT_Disable();		//失能HALL中断
			HALL2_GPIO_INIT_Disable();	//失能HALL2中断
		}
		//启动初始化
		//保护初始化：参数
		if(App.Logic.LogicHall_or_pll == 2){
			if(App.Logic.CtlMode == 3){
				App.FB2.Prot.OverCurrent = App.FB2.Prot.OverCurrent2;				//有Hall往复运动保护电流
			}else{
				App.FB2.Prot.OverCurrent = App.FB2.Prot.OverCurrent1;				//有Hall单向运动保护电流
			}
		}else if (App.Logic.LogicHall_or_pll == 1){
			App.FB2.Prot.OverCurrent = App.FB2.Prot.OverCurrent3;					//无Hall单向运动保护电流
		}
		//速度PI初始化
		VoltageP2_Init();
        HallOsc_Reset(2U); /* 本次有霍尔往复清测速历史并准备有限启动助力；其他模式只撤销助力。 */
	}else if (App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){
		HALL2_GPIO_INIT();			//有霍尔的话初始化HALL
		TIM3_DISABLE();	
		//位置模式保护参数初始化
	}
	//充电
	if(App.Logic.CtlMode == 1 || App.Logic.CtlMode == 2 ||App.Logic.CtlMode ==3 ){
		if(App.Logic.LogicHall_or_pll == 2){			//有霍尔去判断是否需要直接闭环，
			if(App.FB2.Chang_Flag ==1){		//如果是快速切换转向，就不用再花多时间去充电，直接跳转运行
				App.FB2.Status = HS_RUN;	//
				App.FB2.AllRun = 1;				//允许运行
				App.FB2.Chang_Flag = 0;		//清
			}else{
				App.FB2.Status = HS_CHARGE;	
				App.FB2.ChaTim1msCnt = 0;		//清0充电时间
			}
		}else{															//无霍尔就直接去充电
				App.FB2.Status = HS_CHARGE;	
				App.FB2.ChaTim1msCnt = 0;				//清0充电时间				
		}
	}else if (App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){		//充电
			App.FB2.Status = HS_POSDRAG;	//充电
			App.FB2.ChaTim1msCnt = 0;		//清除充电时间		
	}
	App.FB2.Dir.Now = App.FB2.Dir.Set ;		//给设定方向
	
	if(App.FB2.Dir.Now == CW){
		App.FB2.HallVal = gethall2;
		App.FB2.OldHallVal = LastCWhall[App.FB2.HallVal];//传递旧值
	}else{
		App.FB2.HallVal = gethall2;
		App.FB2.OldHallVal = LastCCWhall[App.FB2.HallVal];//传递旧值
	}
}


//充电
/*
 * 函数功能：处理无刷A路预充电，停止请求可以在产生后续输出前取消启动
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Charge(void){
    if ((App.FB.Start == 0U) || (MotorCommand_IsStopRequested() != 0U) || (App.FB.Err != E_NONE))
    {
        App.FB.AllRun = 0U; /* 预充电取消后不能再进入开环或闭环运行。 */
        App.FB.Status = (App.FB.Err == E_NONE) ? HS_WAIT : HS_ERR; /* 退出本通道启动流程，保留已有保护错误。 */
        App.Logic.VBusNowPWM = 0; /* 清除启动阶段遗留的母线目标。 */
        TMR_SetCompare4(TMR2, 0U); /* 松脚发生在预充电阶段也立即关闭BUCK。 */
        PWM_3l_Stop(); /* 关闭当前通道桥臂，禁止后续充电分支再次打开。 */
        return; /* 不执行预充电完成后的启动动作。 */
    }

	if(App.FB.ChaTim1msFg == 1){
		App.FB.ChaTim1msFg = 0;
		App.FB.ChaTim1msCnt ++;
		if(App.FB.ChaTim1msCnt < 10){
			PWM1H_OFF;
			PWM1L_ON;
			PWM2H_OFF;
			PWM2L_OFF;
			PWM3H_OFF;
			PWM3L_OFF;
			PWM1(charge_PWM);
			PWM2(0);
			PWM3(0);
		}else if(App.FB.ChaTim1msCnt < 20){
			PWM1H_OFF;
			PWM1L_OFF;
			PWM2H_OFF;
			PWM2L_ON;
			PWM3H_OFF;
			PWM3L_OFF;
			PWM1(0);
			PWM2(charge_PWM);
			PWM3(0);
		}else if(App.FB.ChaTim1msCnt < 30){
			PWM1H_OFF;
			PWM1L_OFF;
			PWM2H_OFF;
			PWM2L_OFF;
			PWM3H_OFF;
			PWM3L_ON;
			PWM1(0);
			PWM2(0);
			PWM3(charge_PWM);
		}else {
			//充电完成
			if(App.Logic.CtlMode ==1 ||App.Logic.CtlMode==2||App.Logic.CtlMode ==3){	//区分工作模式	
				if(App.Logic.LogicHall_or_pll == 1){//区分是有霍尔还是无霍尔，有霍尔转入运行去闭环，无霍尔的话就需要初始化参数去开环
					RAMP_TABLE_Init(1000,(float)MCPara[26]/1000.0f);
					//启动方式是Vbus，启动力量给定值，限制一下开环BUCK输出的最大最小值
					App.FB.OpenLoop.PwmUp = MCPara[29];
					App.FB.OpenLoop.MinPwm = MCPara[27];
					App.FB.OpenLoop.MaxPwm = MCPara[28];
					App.Logic.VBusNowPWM = 0;
					//相关参数需要归零
					App.FB.OpenLoop.StartHallOkFg = 0;
					App.FB.OpenLoop.StartHallOkCnt = 0;
					App.FB.OpenLoop.OpenLoopTime = 0;
					App.FB.OpenLoop.Spd_NotHallCnt = 0;

					//切入开环
					App.FB.Status =	HS_START;
					App.FB.AllRun = 1;//允许运行
					PWM_3l_Start();		//允许运行
					
				}else if (App.Logic.LogicHall_or_pll == 2){
					App.FB.ChaTim1msCnt = 0;
					PWM_3l_Start();
					App.FB.Status = HS_RUN;//跳转到闭环	//有霍尔就直接去闭环吧，
					App.FB.AllRun = 1;//允许运行
				}
			}
		else if(App.Logic.CtlMode == 4 ||App.Logic.CtlMode ==5){		//
				//切入开环
				App.FB.Status =	HS_POSDRAG;
				App.FB.AllRun = 1;//允许运行
				PWM_3l_Start();		//允许运行
			}					
		}
	}
	if(App.FB.Err != E_NONE){		//运行中发现错误
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();				//抱住下桥进行刹车
	}
	
}




//充电
/*
 * 函数功能：处理无刷B路预充电，停止请求可以在产生后续输出前取消启动
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Charge2(void){
    if ((App.FB2.Start == 0U) || (MotorCommand_IsStopRequested() != 0U) || (App.FB2.Err != E_NONE))
    {
        App.FB2.AllRun = 0U; /* 预充电取消后不能再进入开环或闭环运行。 */
        App.FB2.Status = (App.FB2.Err == E_NONE) ? HS_WAIT : HS_ERR; /* 退出本通道启动流程，保留已有保护错误。 */
        App.Logic.VBusNowPWM = 0; /* 清除启动阶段遗留的母线目标。 */
        TMR_SetCompare4(TMR2, 0U); /* 松脚发生在预充电阶段也立即关闭BUCK。 */
        PWM_3l_Stop2(); /* 关闭当前通道桥臂，禁止后续充电分支再次打开。 */
        return; /* 不执行预充电完成后的启动动作。 */
    }

	if(App.FB2.ChaTim1msFg == 1){
		App.FB2.ChaTim1msFg = 0;
		
		App.FB2.ChaTim1msCnt ++;
		if(App.FB2.ChaTim1msCnt < 10){
			PWM1H_OFF2; /* B路预充电只能操作B路桥臂。 */
			PWM1L_ON2;
			PWM2H_OFF2;
			PWM2L_OFF2;
			PWM3H_OFF2;
			PWM3L_OFF2;
			PWM12(charge_PWM);
			PWM22(0);
			PWM32(0);
		}else if(App.FB2.ChaTim1msCnt < 20){
			PWM1H_OFF2;
			PWM1L_OFF2;
			PWM2H_OFF2;
			PWM2L_ON2;
			PWM3H_OFF2;
			PWM3L_OFF2;
			PWM12(0);
			PWM22(charge_PWM);
			PWM32(0);
		}else if(App.FB2.ChaTim1msCnt < 30){
			PWM1H_OFF2;
			PWM1L_OFF2;
			PWM2H_OFF2;
			PWM2L_OFF2;
			PWM3H_OFF2;
			PWM3L_ON2;
			PWM12(0);
			PWM22(0);
			PWM32(charge_PWM);
		}else {
			//充电完成
			if(App.Logic.CtlMode == 1 ||App.Logic.CtlMode ==2||App.Logic.CtlMode ==3){
				//区分是有霍尔还是无霍尔，有霍尔转入运行去闭环，无霍尔的话就需要初始化参数去开环
				if(App.Logic.LogicHall_or_pll == 1){
					NoHall2_ResetSpeedValidity(); /* 新一轮无霍尔启动清除测速资格，不继承上次运行数据。 */
                    NoHall2_ResetStartup(); /* 新启动重新采集比较器边沿，不能复用上次故障的扇区资格。 */
                    App.FB2.OpenLoop.StepRampCnt = 0U; /* 从拖动表初始慢速段重新起步。 */
                    App.FB2.OpenLoop.OpenStartTimCnt = 0U; /* 清除上一扇区的残余等待时间。 */
                    App.FB2.OpenLoop.HallStartStepCnt = 0; /* 首次强制换相重新从固定表位置按当前方向推进。 */
                    App.FB2.OpenLoop.HallVal = 0; /* 第一次实际强制换相前没有可用于检测的驱动扇区。 */
                    App.FB2.OpenLoop.ClossLoopFlg = 0U; /* 撤销上一轮闭环接管标志。 */
                    App.FB2.OpenLoop.OpenStart1msFg = 0U; /* 等待本轮新的1ms启动节拍，不消费旧标志。 */
                    App.FB2.OpenLoop.SpdSum = 0U; /* 六项数组已在初始化清理，累计值也不能继承。 */
                    App.FB2.Rad_Now = 0.0f; /* 新一轮反馈不继承上一方向的角速度。 */
                    App.FB2.Spd_Now = 0.0f; /* 启动前测量状态归零，原低速保护仍按其原规则累计。 */
					RAMP_TABLE_Init2(1000,(float)MCPara2[26]/1000.0f);
					//启动方式是Vbus，启动力量给定值，限制一下开环BUCK输出的最大最小值
					App.FB2.OpenLoop.PwmUp = MCPara2[29];
					App.FB2.OpenLoop.MinPwm = MCPara2[27];
					App.FB2.OpenLoop.MaxPwm = MCPara2[28];
					App.FB2.OpenLoop.Spd_NotHallCnt = 0;
					App.Logic.VBusNowPWM = 0;
					//相关参数需要归零
					App.FB2.OpenLoop.StartHallOkFg = 0;
					App.FB2.OpenLoop.StartHallOkCnt = 0;
					App.FB2.OpenLoop.OpenLoopTime = 0;
					//切入开环
					App.FB2.Status =	HS_START;
					App.FB2.AllRun = 1;//允许运行
					PWM_3l_Start2();		//允许运行

				}else if (App.Logic.LogicHall_or_pll == 2){
					App.FB2.ChaTim1msCnt = 0;
					PWM_3l_Start2();
					App.FB2.Status = HS_RUN;//跳转到闭环	//有霍尔就直接去闭环吧，
					App.FB2.AllRun = 1;//允许运行
				}
			}else if(App.Logic.CtlMode ==4||App.Logic.CtlMode ==5){
				//切入开环
				App.FB2.Status =	HS_POSDRAG;
				App.FB2.AllRun = 1;//允许运行
				PWM_3l_Start2();		//允许运行				
			}
		}
	}
	if(App.FB2.Err != E_NONE){		//运行中发现错误
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();				//抱住下桥进行刹车
	}
}

//启动开环，无感方波专用
/*
 * 函数功能：在启动许可仍有效时执行无刷A路开环爬坡，松脚先转入制动
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Start(void){
    if ((App.FB.Start == 0U) || (MotorCommand_IsStopRequested() != 0U))
    {
        App.FB.Status = HS_BRA; /* 开环爬坡前先响应停止，交给原制动斜率撤销输出。 */
        return; /* 本周期不能再增加启动PWM或发布闭环运行。 */
    }
					//没有开环，那就直接转闭环
	if(App.Logic.LogicHall_or_pll == 1 ){
		if(App.FB.OpenLoop.OpenStart1msFg == 1){
			App.FB.OpenLoop.OpenStart1msFg = 0;
			//对BUCK电路进行PWM给定爬坡，太复杂，先给固定的PWM给占空比看看可不可以拖动起来，后面再修改此部分
			//简单的来赋值VBUS，
			if(App.FB.Prot.AllCur >= ((float)MCPara[58]/100.0f) ){
				App.Logic.VBusNowPWM -= App.FB.OpenLoop.PwmUp; 
			}else{
				//下面是新增的给定爬坡
				App.Logic.VBusNowPWM += App.FB.OpenLoop.PwmUp;
				if(App.Logic.VBusNowPWM >= App.FB.OpenLoop.MaxPwm){
					App.Logic.VBusNowPWM = App.FB.OpenLoop.MaxPwm;
				}else if(App.Logic.VBusNowPWM < App.FB.OpenLoop.MinPwm){
					App.Logic.VBusNowPWM  =	App.FB.OpenLoop.MinPwm;
				}				
			}
			TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);			//固定的PWM去拖，先托起来，调节MCPara[132]可以暂时调节，后面更改成爬坡
			//启动拖
			App.FB.OpenLoop.OpenStartTimCnt ++;
			if(App.FB.OpenLoop.OpenStartTimCnt >= App.FB.OpenLoop.Step_Delay_Tim[App.FB.OpenLoop.StepRampCnt]){
					App.FB.OpenLoop.OpenStartTimCnt = 0;
				//还没有正确的检测到相序，换相慢慢累加
				if(App.FB.OpenLoop.StartHallOkFg == 0){
					App.FB.OpenLoop.StepRampCnt++;	//换相步数增加
					
					if(App.FB.OpenLoop.StepRampCnt  >= STEP_RAMP_SIZE){	//如果大于最大换相步数都还没有检测到正确的相序，那么就报错
						App.FB.OpenLoop.StepRampCnt = STEP_RAMP_SIZE - 1;	//逐渐减小拖动换相的间隔

					}
					
					if(App.FB.Dir.Now == CW){
						App.FB.OpenLoop.HallStartStepCnt ++;
					}else{
						App.FB.OpenLoop.HallStartStepCnt --;
					}
					
					if(App.FB.OpenLoop.HallStartStepCnt < 0){	
						App.FB.OpenLoop.HallStartStepCnt = 5;
					}else if(App.FB.OpenLoop.HallStartStepCnt >5){
						App.FB.OpenLoop.HallStartStepCnt = 0;	
					}
					//开始换相拖动
					App.FB.OpenLoop.HallVal = RAMP_TABLE_Hall[App.FB.OpenLoop.HallStartStepCnt];
					
					App.FB.OpenLoop.SpdCnt_Arr[App.FB.OpenLoop.HallVal-1] = App.FB.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
					App.FB.OpenLoop.Spd_NotHallCnt = 0;	//清					
					
					Hall_PhaseChange(App.FB.OpenLoop.HallVal,7000);
				}else{
					//检测到正确的相序了，准备转入闭环

				}
			}
		}
		
	}else if(App.Logic.LogicHall_or_pll == 2)
	{
		App.FB.Status = HS_RUN;//跳转到闭环	
	}
	
	if(App.FB.Dir.Now != App.FB.Dir.Set){		//方向发生改变
		App.FB.Status = HS_BRA;

	}
	if(App.FB.Start != 1){
		App.FB.Status = HS_BRA;				//不符合启动条件
	}
	if(App.FB.Err != E_NONE){		//运行中发现错误
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();				//抱住下桥进行刹车
	}
	
}

//通道2启动
/*
 * 函数功能：在启动许可仍有效时执行无刷B路开环爬坡，松脚先转入制动
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Start2(void){
    if ((App.FB2.Start == 0U) || (MotorCommand_IsStopRequested() != 0U))
    {
        App.FB2.Status = HS_BRA; /* 开环爬坡前先响应停止，交给原制动斜率撤销输出。 */
        return; /* 本周期不能再增加启动PWM或发布闭环运行。 */
    }
    if ((App.Logic.LogicHall_or_pll == 1U) &&
        ((MCPara2[30] >= NOHALL2_START_TIMEOUT_MS) ||
         (App.FB2.OpenLoop.OpenLoopTime >= NOHALL2_START_TIMEOUT_MS)))
    {
        if (App.FB2.Err == E_NONE)
        {
            App.FB2.Err = E_Start; /* 参数要求无法在期限内接管或1s仍未成功，新增启动失败兜底不覆盖原故障。 */
        }
    }
    if (App.FB2.Err != E_NONE)
    {
        App.FB2.BraTimCnt = 0; /* 沿用原错误退出的制动计数处理。 */
        App.FB2.AllRun = 0; /* 沿用原故障运行许可撤销，错误期间不能继续开环加PWM。 */
        App.FB2.Status = HS_ERR; /* 交回原错误状态机统一锁存与回报。 */
        PWM_3l_Stop2(); /* 使用原B路关断函数，不修改过流和堵转关断路径。 */
        return; /* 故障确认后不再产生本周期启动输出。 */
    }

					//没有开环，那就直接转闭环
	
	if(App.Logic.LogicHall_or_pll == 1 ){
		if(App.FB2.OpenLoop.OpenStart1msFg == 1){
			App.FB2.OpenLoop.OpenStart1msFg = 0;

			if(App.FB2.Prot.AllCur >= ((float)MCPara[58]/100.0f) ){
				App.Logic.VBusNowPWM -= App.FB2.OpenLoop.PwmUp; 
			}else{
				//下面是新增的给定爬坡。
				App.Logic.VBusNowPWM += App.FB2.OpenLoop.PwmUp;
				if(App.Logic.VBusNowPWM >= App.FB2.OpenLoop.MaxPwm){
					App.Logic.VBusNowPWM = App.FB2.OpenLoop.MaxPwm;
				}	else if(App.Logic.VBusNowPWM < App.FB2.OpenLoop.MinPwm){
					App.Logic.VBusNowPWM  =	App.FB2.OpenLoop.MinPwm;
				}							
			}
            if (App.Logic.VBusNowPWM < 0)
            {
                App.Logic.VBusNowPWM = 0; /* 恒流回退到零后不能继续减成负数并写入无符号比较寄存器。 */
            }
			TMR_SetCompare4(TMR2, App.Logic.VBusNowPWM);			//固定的PWM去拖，先托起来，调节MCPara[132]可以暂时调节，后面更改成爬坡
			//启动拖
			App.FB2.OpenLoop.OpenStartTimCnt ++;
			if(App.FB2.OpenLoop.OpenStartTimCnt >= App.FB2.OpenLoop.Step_Delay_Tim[App.FB2.OpenLoop.StepRampCnt]){
					App.FB2.OpenLoop.OpenStartTimCnt = 0;
				//还没有正确的检测到相序，换相慢慢累加
				if(App.FB2.OpenLoop.StartHallOkFg == 0){
					App.FB2.OpenLoop.StepRampCnt++;	//换相步数增加
					if(App.FB2.OpenLoop.StepRampCnt  >= STEP_RAMP_SIZE){	//如果大于最大换相步数都还没有检测到正确的相序，那么就报错
						App.FB2.OpenLoop.StepRampCnt = STEP_RAMP_SIZE - 1;	//逐渐减小拖动换相的间隔

					}
					
					if(App.FB2.Dir.Now == CW){
						App.FB2.OpenLoop.HallStartStepCnt ++;
					}else{
						App.FB2.OpenLoop.HallStartStepCnt --;
					}
					
					if(App.FB2.OpenLoop.HallStartStepCnt < 0){	
						App.FB2.OpenLoop.HallStartStepCnt = 5;
					}else if(App.FB2.OpenLoop.HallStartStepCnt >5){
						App.FB2.OpenLoop.HallStartStepCnt = 0;	
					}
					//开始换相拖动
					App.FB2.OpenLoop.HallVal = RAMP_TABLE_Hall[App.FB2.OpenLoop.HallStartStepCnt];
					
					App.FB2.OpenLoop.SpdCnt_Arr[App.FB2.OpenLoop.HallVal-1] = App.FB2.OpenLoop.Spd_NotHallCnt;			//偏移1个位置存入滑动滤波数组，方便后面速度直接累加计算
					App.FB2.OpenLoop.Spd_NotHallCnt = 0;	//清					
					
					Hall_PhaseChange2(App.FB2.OpenLoop.HallVal,7000);
				}else{
					//检测到正确的相序了，准备转入闭环

				}
			}
		}
		
	}else if(App.Logic.LogicHall_or_pll == 2)
	{
		App.FB2.Status = HS_RUN;//跳转到闭环	
	}
	
	if(App.FB2.Dir.Now != App.FB2.Dir.Set){		//方向发生改变
		App.FB2.Status = HS_BRA;
	}
	if(App.FB2.Start != 1){										//不符合启动条件
		App.FB2.Status = HS_BRA;
	}
	if(App.FB2.Err != E_NONE){		//运行中发现错误
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();				//抱住下桥进行刹车
	}
}

//运行
void HallSqu_Run(void){
	if(App.FB.Dir.Now != App.FB.Dir.Set){		//方向发生改变
		App.FB.Status = HS_BRA;

	}
	if(App.FB.Start != 1){
		App.FB.Status = HS_BRA;				//不符合启动条件
	}
	
	if(App.FB.Err != E_NONE){		//运行中发现错误
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();				//抱住下桥进行刹车
	}

}

//运行
void HallSqu_Run2(void){
	if(App.FB2.Dir.Now != App.FB2.Dir.Set){		//方向发生改变
		App.FB2.Status = HS_BRA;
	}
	if(App.FB2.Start != 1){										//不符合启动条件
		App.FB2.Status = HS_BRA;
	}
	
	if(App.FB2.Err != E_NONE){		//运行中发现错误
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();				//抱住下桥进行刹车
	}
}
//位置拖动
void HallSqu_Drag(void){
	if(App.Logic.CtlMode == 0 || App.Logic.CtlMode == 1 ||App.Logic.CtlMode == 2||App.Logic.CtlMode == 3||App.Logic.Motor_Kind!=1){
		//退出位置拖动
		App.FB.Status = HS_WAIT;	
		//失能
		PWM_3l_Stop();
		PWM_3l_Stop2();		
		//记录这个位置，后续每次转动都需要停止在这个位置：
		App.Pos.OutSidePosition1 = 0;		
		App.Pos.LockFlag1 = 1;
		//清除在位置模式的临时值	
		App.Pos.PosNow1 = 0x1F;		
	}

	if(App.FB.Err != E_NONE){		//运行中发现错误
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();				//抱住下桥进行刹车
	}	
	
}
//位置拖动2
void HallSqu_Drag2(void){
	if(App.Logic.CtlMode == 0 || App.Logic.CtlMode == 1 ||App.Logic.CtlMode == 2||App.Logic.CtlMode == 3 ||App.Logic.Motor_Kind!=2){
		//退出位置拖动
		App.FB2.Status = HS_WAIT;	
		//失能
		PWM_3l_Stop();
		PWM_3l_Stop2();	
		//记录这个位置，后续每次转动都需要停止在这个位置：
		App.Pos.OutSidePosition2 = 0;		
		App.Pos.LockFlag2 = 1;
		//清除在位置模式的临时值	
		App.Pos.PosNow2 = 0x1F;			
		
	}
	if(App.FB2.Err != E_NONE){		//运行中发现错误
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();				//抱住下桥进行刹车
	}
}


//刹车 刹车方式
/*
 * 函数功能：处理无刷A路制动完成，先处理错误，外部停止先关输出再发布完成
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Bra(void)
{
    if (App.FB.Err != E_NONE)
    {
        App.Logic.Brake_StopPwmOK = 0U; /* 错误退出不能遗留一个可被下次停机使用的完成结果。 */
        App.Logic.Brake_StopTimOK = 0U; /* 清理与本次异常退出相关的旧等待结果。 */
        App.FB.BraTimCnt = 0; /* 结束本通道制动阶段计时。 */
        App.FB.AllRun = 0; /* 撤销本通道运行许可。 */
        App.FB.Status = HS_ERR; /* 错误优先于制动完成，不能在同次调用中先报完成再报错误。 */
        PWM_3l_Stop(); /* 立即关闭本通道桥臂。 */
        return; /* 不再执行下方完成标记置位。 */
    }
    if (App.FB.Bra1msFg == 1U)
    {
        App.FB.Bra1msFg = 0U; /* 消费一次制动检查节拍。 */
        ++App.FB.BraTimCnt; /* 保留本通道原制动进度计数。 */
        if ((App.Logic.VBusNowPWM < MCPara[45]) ||
            ((App.Logic.Brake_Kind == 5U) && (App.Logic.Brake_TimeCnt >= 10U)))
        {
            if (App.Logic.Brake_Kind == 1U)
            {
                App.Logic.VBusNowPWM = 0; /* 停机到退出阈值后最终清零，不保留阈值以内的旧输出。 */
                App.Logic.VBusSetPWM = 0; /* 撤销普通控制环残留目标。 */
                TMR_SetCompare4(TMR2, 0U); /* 先实际关闭BUCK，完成标记不能先于硬件动作。 */
                PWM_3l_Stop(); /* 同步关闭本通道桥臂，再允许状态扫描应用停止模式。 */
            }
            App.FB.Status = HS_STOP; /* 当前状态与完成标记必须属于同一次制动。 */
            App.FB.BraTimCnt = 0; /* 清理已经结束的制动计时。 */
            App.Logic.Brake_StopPwmOK = 1U; /* 最后发布完成，外部停机已经完成零输出动作。 */
            if ((App.Logic.CtlMode == 3U) && (App.Logic.LogicHall_or_pll == 2U))
            {
                App.FB.Chang_Flag = 1U; /* 保留有霍尔往复快速换向时跳过预充电的原行为。 */
            }
        }
    }
}



//刹车 刹车方式
/*
 * 函数功能：处理无刷B路制动完成，先处理错误，外部停止先关输出再发布完成
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Bra2(void)
{
    if (App.FB2.Err != E_NONE)
    {
        App.Logic.Brake_StopPwmOK = 0U; /* 错误退出不能遗留一个可被下次停机使用的完成结果。 */
        App.Logic.Brake_StopTimOK = 0U; /* 清理与本次异常退出相关的旧等待结果。 */
        App.FB2.BraTimCnt = 0; /* 结束本通道制动阶段计时。 */
        App.FB2.AllRun = 0; /* 撤销本通道运行许可。 */
        App.FB2.Status = HS_ERR; /* 错误优先于制动完成，不能在同次调用中先报完成再报错误。 */
        PWM_3l_Stop2(); /* 立即关闭本通道桥臂。 */
        return; /* 不再执行下方完成标记置位。 */
    }
    if (App.FB2.Bra1msFg == 1U)
    {
        App.FB2.Bra1msFg = 0U; /* 消费一次制动检查节拍。 */
        ++App.FB2.BraTimCnt; /* 保留本通道原制动进度计数。 */
        if ((App.Logic.VBusNowPWM < MCPara2[45]) ||
            ((App.Logic.Brake_Kind == 5U) && (App.Logic.Brake_TimeCnt >= 10U)))
        {
            if (App.Logic.Brake_Kind == 1U)
            {
                App.Logic.VBusNowPWM = 0; /* 停机到退出阈值后最终清零，不保留阈值以内的旧输出。 */
                App.Logic.VBusSetPWM = 0; /* 撤销普通控制环残留目标。 */
                TMR_SetCompare4(TMR2, 0U); /* 先实际关闭BUCK，完成标记不能先于硬件动作。 */
                PWM_3l_Stop2(); /* 同步关闭本通道桥臂，再允许状态扫描应用停止模式。 */
            }
            App.FB2.Status = HS_STOP; /* 当前状态与完成标记必须属于同一次制动。 */
            App.FB2.BraTimCnt = 0; /* 清理已经结束的制动计时。 */
            App.Logic.Brake_StopPwmOK = 1U; /* 最后发布完成，外部停机已经完成零输出动作。 */
            if ((App.Logic.CtlMode == 3U) && (App.Logic.LogicHall_or_pll == 2U))
            {
                App.FB2.Chang_Flag = 1U; /* 保留有霍尔往复快速换向时跳过预充电的原行为。 */
            }
        }
    }
}

//位置刹车
void HallSqu_PosBra(void){

	
	
	//如果报错也退出
		if(App.FB.Err != E_NONE){
		App.FB.BraTimCnt = 0;
		App.FB.AllRun = 0;
		App.FB.Status = HS_ERR;
		PWM_3l_Stop();
	}
}

//位置刹车
void HallSqu_PosBra2(void){
	
	
	//如果报错也退出
	if(App.FB2.Err != E_NONE){
		App.FB2.BraTimCnt = 0;
		App.FB2.AllRun = 0;
		App.FB2.Status = HS_ERR;
		PWM_3l_Stop2();
	}
}

//停止
void HallSqu_Stop(void){
	volatile int8_t temp =0 ;
	int8_t i = 0;
	if((App.Pos.LockFlag1 == 1)&&(App.Logic.CtlMode !=3)&&(App.Pos.PosFlag == 1)){				//模式3先不给进
		if(App.Pos.WaitStopTimCnt1 > App.Pos.WaitStopTim1){
			App.Pos.PosFlag = 0;
			App.Pos.WaitStopTimCnt1 = 0;
			App.FB.AllRun = 0;
			PWM_3l_Start();		//允许运行
			temp = gethall;						//读取Hall;
			for(i=0; i<7; i++ ){
				if(temp == RAMP_TABLE_Hall[i]){
					App.Pos.Tempi1 = i;
					if(App.Pos.Tempi1== 0){		//往前位移一个数组
						App.Pos.Tempi1 = 5;
					}else{
						App.Pos.Tempi1 --;
					}
					break;
				}
			}
			App.FB.Status = HS_POSBRA;
			App.FB.Prot.HallDragCnt = 0;
			TMR_SetCompare4(TMR2, MCPara[53]);			//
			App.Pos.PlanPosStep1 =(int16_t) (App.Pos.OutSidePosition1 / App.Pos.OutSideAngle1);			//计算本次停止应该执行的步数
		}else{	
			App.FB.AllRun = 0;
			PWM_3l_Stop();
		}
	}else{
		App.FB.Status = HS_WAIT;
		App.FB.AllRun = 0;
		PWM_3l_Stop();
	}
	
}


//停止
void HallSqu_Stop2(void){
	
	volatile int8_t temp =0 ;
	int8_t i = 0;
	if((App.Pos.LockFlag2 == 1)&&(App.Logic.CtlMode !=3)&&(App.Pos.PosFlag == 1)){				//模式3先不给进
		if(App.Pos.WaitStopTimCnt2 > App.Pos.WaitStopTim2){
		App.Pos.PosFlag = 0;
			App.Pos.WaitStopTimCnt2 = 0;
			App.FB2.AllRun = 0;
			PWM_3l_Start2();						//允许运行
			temp = gethall2;						//读取Hall;
			for(i=0; i<7; i++ ){
				if(temp == RAMP_TABLE_Hall[i]){
					App.Pos.Tempi2 = i;
					if(App.Pos.Tempi2== 0){		//往前位移一个数组
						App.Pos.Tempi2 = 5;
					}else{
						App.Pos.Tempi2 --;
					}
					break;
				}
			}
			App.FB2.Status = HS_POSBRA;
			App.FB2.Prot.HallDragCnt = 0;
			TMR_SetCompare4(TMR2, MCPara2[53]);			//
			App.Pos.PlanPosStep2 =(int16_t) (App.Pos.OutSidePosition2 / App.Pos.OutSideAngle2);			//计算本次停止应该执行的步数
		}else{	
		App.FB2.AllRun = 0;
		PWM_3l_Stop2();
		}
	}else{
		App.FB2.Status = HS_WAIT;
		App.FB2.AllRun = 0;
		PWM_3l_Stop2();
	}
}

//错误
/*
 * 函数功能：关闭无刷A路错误输出并清理制动结果，恢复后回等待态
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Err(void){
    App.Logic.Brake_StopPwmOK = 0U; /* 错误恢复不得把完成标记带到下一次运行。 */
    App.Logic.Brake_StopTimOK = 0U; /* 同步清除旧等待完成结果。 */

	App.FB.AllRun = 0;
	App.Logic.VBusNowPWM = 0;			//错误需要清零PWM现在值，防止重启时
	PWM_3l_Stop();
	TMR_SetCompare4(TMR2,0);
	if(App.FB.Err == E_NONE){
		App.FB.Status = HS_WAIT;
	}
}

//错误
/*
 * 函数功能：关闭无刷B路错误输出并清理制动结果，恢复后回等待态
 * 输入参数：无。
 * 返回参数：无。
 */
void HallSqu_Err2(void){
    App.Logic.Brake_StopPwmOK = 0U; /* 错误恢复不得把完成标记带到下一次运行。 */
    App.Logic.Brake_StopTimOK = 0U; /* 同步清除旧等待完成结果。 */

	App.FB2.AllRun = 0;
	App.Logic.VBusNowPWM = 0;			//错误需要清零PWM现在值，防止重启时
	PWM_3l_Stop2();
	TMR_SetCompare4(TMR2,0);
	if(App.FB2.Err == E_NONE){
		App.FB2.Status = HS_WAIT;
	}
}



//状态机控制
void HallSqu_Ctl(void){
	switch(App.FB.Status){
				 case HS_WAIT : //等待
					HallSqu_Wait();
						break;      
				 case HS_INIT:   //初始化
					HallSqu_Init();
						break; 
				 case HS_CHARGE: //充电
					HallSqu_Charge();
						break;
				 case HS_START:  //启动/开环
					HallSqu_Start();
						break;
				 case HS_RUN:    //运行
					HallSqu_Run();
						break;
				 case HS_BRA:    //刹车
					HallSqu_Bra();
						break;
				 case HS_STOP:    //停止
					HallSqu_Stop();
						break; 
				 case HS_ERR:     //错误
					HallSqu_Err();
						break; 
				 case HS_POSBRA:
					 HallSqu_PosBra();//刹车拖动
						break;
				 case HS_POSDRAG:
					 HallSqu_Drag();//拖动
						break;
					 
				 
			   default:
					 break;
	}
}



//状态机控制
void HallSqu_Ctl2(void){
	switch(App.FB2.Status){
				 case HS_WAIT : //等待
					HallSqu_Wait2();
						break;      
				 case HS_INIT:   //初始化
					HallSqu_Init2();
						break; 
				 case HS_CHARGE: //充电
					HallSqu_Charge2();
						break;
				 case HS_START:  //启动/开环
					HallSqu_Start2();
						break;
				 case HS_RUN:    //运行
					HallSqu_Run2();
						break;
				 case HS_BRA:    //刹车
					HallSqu_Bra2();
						break;
				 case HS_STOP:    //停止
					HallSqu_Stop2();
						break; 
				 case HS_ERR:     //错误
					HallSqu_Err2();
						break; 
				 case HS_POSBRA:
					 HallSqu_PosBra2();//刹车拖动
						break;
				 case HS_POSDRAG:
					 HallSqu_Drag2();//拖动
						break;
				 
			   default:
					
					 break;
	}
}



//方波换相
								// 5 1 3 2 6 4		CCW
//换相中断			// 5 4 6 2 3 1		CW
							// AB-AC-BC-BA-CA-CB
/*
 * 函数功能：在有效运行或定位状态下执行A路桥臂换相，并防止停机抢占后旧换相重开输出
 * 输入参数：hall_val为相序，pwm为兼容接口占空比参数。
 * 返回参数：无。
 */
void Hall_PhaseChange(u8 hall_val,u16 pwm){
    uint32_t primask = __get_PRIMASK(); /* 换相可能被500us停机抢占，检查和桥臂写入必须连成一个短操作。 */
    __disable_irq(); /* 防止已关断的中断返回后继续执行旧换相指令。 */
    if ((MotorCommand_IsOutputLocked() != 0U) || (App.FB.Err != E_NONE) ||
        ((App.FB.AllRun == 0U) && (App.FB.Status != HS_POSBRA) &&
         !(((App.Logic.CtlMode == 4U) || (App.Logic.CtlMode == 5U)) && (App.FB.Start != 0U))))
    {
        __set_PRIMASK(primask); /* 不具备有效运行或定位状态时保持已经关闭的输出。 */
        return; /* 旧换相不能恢复已完成的停止。 */
    }

	
		if(App.FB.Dir.Now == CCW){
			hall_val = 7 - hall_val;		//反方向
		}
		switch(hall_val){
					case 5 : //BA
					
					PWM3H_OFF;
					PWM3L_OFF;
					PWM3(0);
					
					PWM2L_OFF;
					PWM1H_OFF;
					
					PWM1(PWM_ARR);
					PWM2H_ON;

					PWM1L_ON;
					PWM2(PWM_ARR);
												
						break;      
				 case 1:  //CA
					 PWM2H_OFF;
					PWM2L_OFF;
					PWM2(0);
				 
					PWM3L_OFF;
					PWM1H_OFF;
					
					
				 
					PWM1(PWM_ARR);
					PWM3H_ON;

					PWM1L_ON;
					PWM3(PWM_ARR);
					
						break; 
				 case 3: //CB
					 PWM1H_OFF;
					PWM1L_OFF;
					PWM1(0);
				 
					PWM3L_OFF;
					PWM2H_OFF;
					
					
				 
					PWM2(PWM_ARR);
					PWM3H_ON;

					PWM2L_ON;
					PWM3(PWM_ARR);
					
						break;
				 case 2:  //AB
					 PWM3H_OFF;
					PWM3L_OFF;
					PWM3(0);
				 
					PWM1L_OFF;
					PWM2H_OFF;
					
					
				 
					PWM2(PWM_ARR);
					PWM1H_ON;

					PWM2L_ON;
					PWM1(PWM_ARR);
					
						break;
				 case 6:  //AC		
					 PWM2H_OFF;
					PWM2L_OFF;
					PWM2(0);
				 
					PWM1L_OFF;
					PWM3H_OFF;
					
					PWM3(PWM_ARR);
					PWM1H_ON;

					PWM3L_ON;
					PWM1(PWM_ARR);
					
						break;
					case 4: //BC			
						PWM1H_OFF;
					PWM1L_OFF;
					PWM1(0);
					
					PWM2L_OFF;
					PWM3H_OFF;
					
					PWM3(PWM_ARR);
					PWM2H_ON;

					PWM3L_ON;
					PWM2(PWM_ARR);
					
						break; 
					default:

					break;
	}
    __set_PRIMASK(primask); /* 本次桥臂更新结束，恢复ADC和500us控制中断。 */
}




//换相中断
/*
 * 函数功能：在有效运行或定位状态下执行B路桥臂换相，并防止停机抢占后旧换相重开输出
 * 输入参数：hall_val为相序，pwm为兼容接口占空比参数。
 * 返回参数：无。
 */
void Hall_PhaseChange2(u8 hall_val,u16 pwm){
    uint32_t primask = __get_PRIMASK(); /* 换相可能被500us停机抢占，检查和桥臂写入必须连成一个短操作。 */
    __disable_irq(); /* 防止已关断的中断返回后继续执行旧换相指令。 */
    if ((MotorCommand_IsOutputLocked() != 0U) || (App.FB2.Err != E_NONE) ||
        ((App.FB2.AllRun == 0U) && (App.FB2.Status != HS_POSBRA) &&
         !(((App.Logic.CtlMode == 4U) || (App.Logic.CtlMode == 5U)) && (App.FB2.Start != 0U))))
    {
        __set_PRIMASK(primask); /* 不具备有效运行或定位状态时保持已经关闭的输出。 */
        return; /* 旧换相不能恢复已完成的停止。 */
    }

	
		if(App.FB2.Dir.Now == CCW){
			hall_val = 7 - hall_val;		//反方向
		}
		switch(hall_val){
					case 5 : //BA
					
					PWM3H_OFF2;
					PWM3L_OFF2;
					PWM32(0);
					
					PWM2L_OFF2;
					PWM1H_OFF2;
					
					PWM12(PWM_ARR);
					PWM2H_ON2;

					PWM1L_ON2;
					PWM22(PWM_ARR);
					
						break;      
				 case 1:  //CA
					PWM2H_OFF2;
					PWM2L_OFF2;
					PWM22(0);
				 
					PWM3L_OFF2;
					PWM1H_OFF2;
					
					
				 
					PWM12(PWM_ARR);
					PWM3H_ON2;

					PWM1L_ON2;
					PWM32(PWM_ARR);
					
						break; 
				 case 3: //CB
					PWM1H_OFF2;
					PWM1L_OFF2;
					PWM12(0);
				 
					PWM3L_OFF2;
					PWM2H_OFF2;
					
					
				 
					PWM22(PWM_ARR);
					PWM3H_ON2;

					PWM2L_ON2;
					PWM32(PWM_ARR);
					
						break;
				 case 2:  //AB
					 PWM3H_OFF2;
					PWM3L_OFF2;
					PWM32(0);
				 
					PWM1L_OFF2;
					PWM2H_OFF2;
					
					
				 
					PWM22(PWM_ARR);
					PWM1H_ON2;

					PWM2L_ON2;
					PWM12(PWM_ARR);
					
						break;
				 case 6:  //AC
					 PWM2H_OFF2;
					PWM2L_OFF2;
					PWM22(0);
				 
					PWM1L_OFF2;
					PWM3H_OFF2;
					
					PWM32(PWM_ARR);
					PWM1H_ON2;

					PWM3L_ON2;
					PWM12(PWM_ARR);
					
						break;
					case 4: //BC
					PWM1H_OFF2;
					PWM1L_OFF2;
					PWM12(0);
					
					PWM2L_OFF2;
					PWM3H_OFF2;
					
					PWM32(PWM_ARR);
					PWM2H_ON2;

					PWM3L_ON2;
					PWM22(PWM_ARR);
					
						break; 
					default:
					break;
	}
    __set_PRIMASK(primask); /* 本次桥臂更新结束，恢复ADC和500us控制中断。 */
}


//下桥全开
void PWM_3l_L_Open(void){
	PWM1H_OFF;
	PWM2H_OFF;
	PWM3H_OFF;
	
	PWM1L_ON;
	PWM2L_ON;
	PWM3L_ON;
	
	PWM1(PWM_ARR);
	PWM2(PWM_ARR);
	PWM3(PWM_ARR);
}

//全关
void PWM_3l_Stop(void){
	PWM1H_OFF;
	PWM1L_OFF;
	PWM2H_OFF;
	PWM2L_OFF;
	PWM3H_OFF;
	PWM3L_OFF;
	PWM1(0);
	PWM2(0);
	PWM3(0);	
}

//全关
void PWM_3l_Stop2(void){
	PWM1H_OFF2;
	PWM1L_OFF2;
	PWM2H_OFF2;
	PWM2L_OFF2;
	PWM3H_OFF2;
	PWM3L_OFF2;
	PWM12(0);
	PWM22(0);
	PWM32(0);	
}

//全开
/*
 * 函数功能：在输出门禁允许时开启无刷A路桥臂并初始化相PWM
 * 输入参数：无。
 * 返回参数：无。
 */
void PWM_3l_Start(void){
    if ((MotorCommand_IsOutputLocked() != 0U) || (App.FB.Err != E_NONE))
    {
        return; /* 上电门禁、失联或停机故障时不能重新开启本通道桥臂。 */
    }

	PWM1H_ON;
	PWM1L_ON;
	PWM2H_ON;
	PWM2L_ON;
	PWM3H_ON;
	PWM3L_ON;
	PWM1(0);
	PWM2(0);
	PWM3(0);
}
//全开
/*
 * 函数功能：在输出门禁允许时开启无刷B路桥臂并初始化相PWM
 * 输入参数：无。
 * 返回参数：无。
 */
void PWM_3l_Start2(void){
    if ((MotorCommand_IsOutputLocked() != 0U) || (App.FB2.Err != E_NONE))
    {
        return; /* 上电门禁、失联或停机故障时不能重新开启本通道桥臂。 */
    }

	PWM1H_ON2;
	PWM1L_ON2;
	PWM2H_ON2;
	PWM2L_ON2;
	PWM3H_ON2;
	PWM3L_ON2;
	PWM12(0);
	PWM22(0);
	PWM32(0);
}       

//通道1电压环初始化
void VoltageP1_Init(void){
	if(App.Logic.CtlMode == 3){		//往复运动的PID参数
	mcApp_VoltageP_PIParam.qKp  = (float)MCPara[46]/ 1000.0f;
	mcApp_VoltageP_PIParam.qKi  = (float)MCPara[47]/ 1000.0f;
	mcApp_VoltageP_PIParam.qKc	= (float)MCPara[48]/ 1000.0f;
	}else{												//单向运动的PID参数
		if(App.Logic.LogicHall_or_pll == 2){	//有Hall单向运动
		mcApp_VoltageP_PIParam.qKp  = (float)MCPara[31]/ 1000.0f;
		mcApp_VoltageP_PIParam.qKi  = (float)MCPara[32]/ 1000.0f;
		mcApp_VoltageP_PIParam.qKc	= (float)MCPara[33]/ 1000.0f;
		}else{		//无Hall单向运动
		mcApp_VoltageP_PIParam.qKp  = (float)MCPara[61]/ 1000.0f;
		mcApp_VoltageP_PIParam.qKi  = (float)MCPara[62]/ 1000.0f;
		mcApp_VoltageP_PIParam.qKc	= (float)MCPara[63]/ 1000.0f;
		}
	}
	mcApp_VoltageP_PIParam.qOutMax = (float)MCPara[34];
	mcApp_VoltageP_PIParam.qOutMin = (float)MCPara[35];
	mcApp_VoltageP_PIParam.qdSumMax = (float)MCPara[34];
	mcApp_VoltageP_PIParam.qdSumMin = -mcApp_VoltageP_PIParam.qdSumMax;
	
	
	if(App.Logic.LogicHall_or_pll == 1){
		mcApp_VoltageP_PIParam.qOutMin = (float)MCPara[27];
		mcApp_VoltageP_PIParam.qOutMax = (float)MCPara[28];
	}
		
	
	mcApp_VoltageP_PIParam.qdSum=0;
	mcApp_VoltageP_PIParam.qOut=0;	
	mcLib_InitPI(&mcApp_VoltageP_PIParam);
}

//通道2电压环初始化
/*
 * 函数功能：按B路运行模式初始化本通道速度PI及输出范围
 * 输入参数：无。
 * 返回参数：无。
 */
void VoltageP2_Init(void){
	
	if(App.Logic.CtlMode == 3){		//往复运动的PID参数
	mcApp_VoltageP_PIParam2.qKp  = (float)MCPara2[46]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
	mcApp_VoltageP_PIParam2.qKi  = (float)MCPara2[47]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
	mcApp_VoltageP_PIParam2.qKc	= (float)MCPara2[48]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
	}else{
		if(App.Logic.LogicHall_or_pll == 2){	//有Hall单向运动
			mcApp_VoltageP_PIParam2.qKp  = (float)MCPara2[31]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
			mcApp_VoltageP_PIParam2.qKi  = (float)MCPara2[32]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
			mcApp_VoltageP_PIParam2.qKc	=  (float)MCPara2[33]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
		}else{
			mcApp_VoltageP_PIParam2.qKp  = (float)MCPara2[61]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
			mcApp_VoltageP_PIParam2.qKi  = (float)MCPara2[62]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
			mcApp_VoltageP_PIParam2.qKc	=  (float)MCPara2[63]/ 1000.0f; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
		}
	}

	mcApp_VoltageP_PIParam2.qOutMax = (float)MCPara2[34]; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
	mcApp_VoltageP_PIParam2.qOutMin = (float)MCPara2[35]; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
	mcApp_VoltageP_PIParam2.qdSumMax = (float)MCPara2[34];
	mcApp_VoltageP_PIParam2.qdSumMin = -mcApp_VoltageP_PIParam2.qdSumMax;
	
	if(App.Logic.LogicHall_or_pll == 1){
		mcApp_VoltageP_PIParam2.qOutMin = (float)MCPara2[27]; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
		mcApp_VoltageP_PIParam2.qOutMax = (float)MCPara2[28]; /* B路参数只写B路PI，避免切换通道时污染另一通道。 */
	}
		
	
	mcApp_VoltageP_PIParam2.qdSum=0;
	mcApp_VoltageP_PIParam2.qOut=0;	
	mcLib_InitPI(&mcApp_VoltageP_PIParam2);
}

