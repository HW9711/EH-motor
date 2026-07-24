#ifndef __INT_H
#define __INT_H

#include  "userpara.h"
#include "at32f4xx.h"

extern volatile int16_t PWMNow_text;
void Ctl_1ms(void);
void Ctl_1ms2(void);

void FangBo_1ms(void);
void TIM_Break_Int2(void);
void TIM_Break_Int(void);
#endif

