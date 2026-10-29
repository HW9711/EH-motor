#ifndef __FANGBO_H
#define __FANGBO_H

#include "mcctl.h"

#define STEP_RAMP_SIZE 64
#define NOHALL2_START_TIMEOUT_MS 1000U /* 启动最多强制拖动1s，未接管走原错误关断；不修改原过流或堵转期限。 */
void Hall_PhaseChange(u8 hall_val,u16 pwm);
void Hall_PhaseChange2(u8 hall_val,u16 pwm);

void HallSqu_Ctl(void);
void HallSqu_Ctl2(void);

void PWM_3l_Stop(void);
void PWM_3l_Stop2(void);

void PWM_3l_L_Open(void);
void PWM_3l_Start(void);
void PWM_3l_Start2(void);



uint8_t HallOsc_CaptureEdge(uint8_t channel); /* 仅有霍尔往复使用完整计时捕获；返回0时继续原模式逻辑。 */
uint8_t HallOsc_TimerOverflow(void); /* 仅有霍尔往复累计TMR5溢出，其他模式返回0。 */
void HallSqu_ADCISRTasks(void);
void Hall_Vbus_PI_Out(void);
void Hall_Vbus_PI_Out2(void);
void NoHall2_ResetStartup(void); /* 新启动前清除B路比较器边沿资格。 */
void NoHall2_ResetSpeedValidity(void); /* B路启动和闭环接管时复位独立测速资格，不改原保护计数。 */
void NoHall2_RecordSpeedInterval(uint8_t hall_val, uint32_t interval); /* 只在B路真实换相写周期后调用，interval为原5us计数。 */



void RAMP_TABLE_Init(float tim,float cnt);
void RAMP_TABLE_Init2(float tim,float cnt);

void Fangbo_Init(void);
void VoltageP1_Init(void);
void VoltageP2_Init(void);
void HallSqu_PosBra(void);
void HallSqu_PosBra2(void);
void HallSqu_Drag(void);
void HallSqu_Drag2(void);

extern uint8_t NextCCWhall[7];
extern uint8_t NextCWhall[7];
extern const uint8_t RAMP_TABLE_Hall[7];
#endif





