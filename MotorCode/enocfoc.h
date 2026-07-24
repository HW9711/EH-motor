#ifndef __ENOC_H
#define __ENOC_H


#include "mcctl.h"

void Enoc_Z_Offset(void);
void Enoc_Z2_Offset(void);
uint16_t return_enoc_val(TMR_Type* TMRx);
void set_enoc_val(TMR_Type* TMRx,uint16_t val);



#endif

