#ifndef __YSSTATEMACHINE_H_
#define	__YSSTATEMACHINE_H_


typedef enum{
    CTLS_WAIT=0,			//0
    CTLS_STOP,				//1
    CTLS_INIT,				//2
    CTLS_CHECK,          	//3
    CTLS_OPENLOOP_RUN,		//4
    CTLS_CLOSELOOP_RUN,		//5
    CTLS_ERROR,          //错误	//6
		CTLS_BRAKE,          //刹车	//7
}MTSTATUS_TYPEDEF;


void TIME1_PWM_Stop_3Channel(void);
void TIME8_PWM_Stop_3Channel(void);
void Vbus_Tim_ON(void);
void Vbus_Tim_OFF(void);

void YSMotor_Ctl1(void);
void YSMotor_Ctl2(void);


//状态机函数声明函数
void Ctl_Wait(void);
void Ctl_Init(void);
void Ctl_OpenLoop(void);
void Ctl_CloseLoop(void);
void Ctl_Error(void);
void Ctl_Stop(void);
void Ctl_Check(void);
void Ctl_Brake(void);

void Ctl_Wait2(void);
void Ctl_Init2(void);
void Ctl_OpenLoop2(void);
void Ctl_CloseLoop2(void);
void Ctl_Error2(void);
void Ctl_Stop2(void);
void Ctl_Check2(void);
void Ctl_Brake2(void);

#endif






