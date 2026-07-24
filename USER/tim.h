#ifndef __TIM_H
#define __TIM_H
#include "at32f4xx_tim.h"
#include "userpara.h"

void TIM_Init_App(void);
void TIM3_BUCK_Init(void);
void TIM_HALL_Confi(void);
void TIM_ENABC_Confi(void);
void TIM3_DISABLE(void);
void TIM_HALL_Disable(void);
void TIM_Motor_PWM(TMR_Type* TMRx);
#endif
