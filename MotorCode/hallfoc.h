#ifndef __HALLFOC_H
#define __HALLFOC_H
#include "mcctl.h"
#include  "userpara.h"

void hall_cal(void);
void hall_cal2(void);
void Hall_FOC_CalculateParkAngle(void);
void Hall_FOC_CalculateParkAngle2(void);
void Hall_FOC_DoControl(void);
void Hall_FOC_DoControl2(void);
void Hall_Stady(void);
void Hall_Stady2(void);
void HallStady_Check(void);
void HallStady_Check2(void);

#endif
