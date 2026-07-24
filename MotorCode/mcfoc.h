#ifndef _MCFOC_H
#define _MCFOC_H

#include  "userpara.h"

void mcApp_ADCISRTasks(void);
//void CalSpd_Init(void);
void pll_run(float phase, float dt, volatile float *phase_var,volatile float *speed_var);
void pll_run2(float phase, float dt, volatile float *phase_var,volatile float *speed_var);
void Hall_Spd_Mea(void);
void MC_APP_MC_CalculateParkAngle(void);
void MC_APP_MC_CalculateParkAngle2(void);
void MC_APP_MC_DoControl(void );
void MC_APP_MC_DoControl2(void );
void mcApp_InitControlParameters(void);
void mcApp_InitControlParameters2(void);
void mcApp_InitEstimParm(void);
void mcApp_InitEstimParm2(void);
void Spd_PI_Out(void);
void Spd_PI_Out2(void);
void Id_Ctl(void);
void Id_Ctl2(void);


#endif 
