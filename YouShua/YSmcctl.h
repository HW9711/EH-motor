#ifndef __YSMCCTL_H_
#define	__YSMCCTL_H_
#include  "stdint.h"
#include "mcctl.h"



typedef struct {
	
		float   qdSum;  // Integrator Output of the PI Compensator
		float   qdSumMax;  // Integrator Output of the PI Compensator
		float   qdSumMin;  // Integrator Output of the PI Compensator
    float   qKp;    // Proportional Coefficient of the PI Compensator
    float   qKi;    // Integral Coefficient of the PI Compensator
    float   qKc;    // Anti-windup Coefficient of the PI Compensator
    float   qOutMax;// Max output limit of the PI Compensator
    float   qOutMin;// Min output limit of the PI Compensator
    float   qInRef; // Reference input of the PI Compensator
    float   qInMeas;// Feedback input of the PI Compensator
    float   qOut;   // Proportional + Integral Output of the PI Compensator
		
    float   qErr;   // Error input of the PI Compensator
		float   qErrMax;   // Error input of the PI Compensator
		float   qErrMin;   // Error input of the PI Compensator
//    float   qErrDelta;
//    float   qLastErr;
}mcParam_PIController2;


extern mcParam_PIController2 MC_Spd;
extern mcParam_PIController2 MC_VBus;

extern uint16_t YSPara1[ParaNum];
extern uint16_t YSPara2[ParaNum];
void mcLib_InitPI2( mcParam_PIController2 *pParam);
void mcLib_CalcPI2( mcParam_PIController2 *pParam);

void YSPI_Init(void);

void YS_SpdPI_Out(void);
void YS_VBus_PI_Out(void);
void YScalculation(void);
void YS1msCtl(void);
void YS500msCtl(void);

#endif







