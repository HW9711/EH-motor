#ifndef __FANGBO_H
#define __FANGBO_H

#include "mcctl.h"

#define STEP_RAMP_SIZE 64
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





