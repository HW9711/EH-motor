#ifndef __FANGBO_NOHALL_H_
#define __FANGBO_NOHALL_H_

#include "mcctl.h"

#define		phase1			1
#define		phase2			2
#define		phase3			3
#define		phase4			4
#define		phase5			5
#define		phase6			6

void OpenLoop_check_changeCW(uint8_t hall_val);
void OpenLoop_change(uint8_t hall_val);
void OpenLoop_check_changeCCW(uint8_t hall_val);
void OpenLoop_check_changeCCW2(uint8_t hall_val);
void OpenLoop_check_changeCW2(uint8_t hall_val);
#endif





