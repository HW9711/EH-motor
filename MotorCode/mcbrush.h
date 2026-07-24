#ifndef __MCBRUSH_H
#define __MCBRUSH_H
#include "userpara.h"

#if defined (STM32_FOC)
#include "stm32f4xx.h"
#elif defined(AT32_FOC)
#include "at32f4xx.h"
#endif
void TIME_PWM_Start_Unipolarity(void);//单极性
void TIME_PWM_Start_bipolar(void);
void Set_PWM_Updata(void);//SetPwm 越大，力量越大
#endif
