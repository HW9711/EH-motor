#ifndef __YOUSHUA_H_
#define __YOUSHUA_H_

#include  "stdint.h"
#include  "gpio.h"
#include  "userpara.h"
#include  "calculation.h"
#include  "mclib.h"
#include	"mcctl.h"




typedef struct{

	volatile uint8_t	Channel;		//选择的通道
	volatile uint8_t	MotorKind;	//选择通道所对应的电机种类
	volatile uint8_t 	CtlMode;		//电机的控制模式
	volatile uint8_t	OldChannel;	//上一次选择的通道，用来判定通道是否发生变换！
	
	volatile uint8_t	Drill;				//选择的钻头，通道1有1/2，通道2有1/2
	volatile uint8_t	OldDrill;			//上一次选择的钻头
	
	uint16_t RunTimCnt1;					//通道1启动运行后，多少ms开始才允许估算转速	
	uint16_t RunTimCnt2;					//通道2启动运行后，多少ms开始才允许估算转速	
	uint16_t UartSetSpd;					//通讯设置转速

	int32_t VbusSetPWM;						//BUCK电路目标PWM
	int32_t VbusNowPWM;						//BUCK电路实际PWM
	uint8_t BreakSta;							//刹车标志					0：没有刹车，1：正在刹车
	float VbusRef;								//速度环的输出值，给电压环的参考值			速度环-->电压环
	
	
	uint8_t u8WfFru;					//往复模式频率
	uint16_t u16WfCyc;				//往复模式周期
	uint8_t u8MaxSpdFru;			//最大转速频率
	uint8_t Mode31msFg;				//
	uint32_t Mode3TimCnt;
	uint8_t Mode3Cha;
	uint32_t u32SetSpd;
	uint32_t u32SetSpd_temp;
	uint32_t MotorFlashAddr1;
	uint32_t MotorFlashAddr2;
}Log_TYPEDEF;


typedef struct{
	//方向
	//转速
	float		SpdNow;								//实际转速
	float		SpdNowLPF;						//滤波后的转速
	float		SpdSet;								//目标转速
	float		SpdSetSet;						//实际传入PID的目标转速	（目标转速也需要爬坡）
	float		MaxSpdSet;						//最大转速设置
	float		MinSpdSet;						//最小转速设置
	
	//PWM
	uint16_t PwmMaxSet;						//PWM最大值，作用定时器CCR
	uint16_t PwmMinSet;						//PWM最小值，作用定时器CCR
	int32_t  PwmSet;							//PWM目标值
	int32_t  PwmNow;							//PWM实际值（需要爬坡）
	
	//SysTim
	uint8_t  CalAdcFg;            //规则通道计算时间周期 100us
	uint8_t  Log1msCnt1;					//逻辑通道1 1ms计数，为了得到1ms
	uint8_t  Log1msCnt2;					//逻辑通道2 1ms计数，为了得到1ms
	uint16_t StaLedCyc;						//错误指示灯时间计数
	uint8_t  CheckCyc;						//系统保护100us1次
	uint8_t	 CH1Motor_Ctl1msFlg;	//状态机1ms执行一次
	uint8_t	 CH2Motor_Ctl1msFlg;	//状态机1ms执行一次
	
	volatile uint8_t	YS1msCnt;		//有刷1ms计数
	volatile uint16_t	YS500msCnt;	//有刷500ms计数
	
}SYSCTL__TYPEDEF;


typedef struct{

	float Mbus;										//系统母线电压
	float Temper;									//系统温度
	
	float			OverVoltage;				//系统过压保护值
	int16_t		OverVolCnt;					//系统过压保护时间阈值
	
	float			UnderVoltage;				//系统欠压保护值
	int16_t		UnderVolCnt;				//系统欠压保护时间阈值
	
	float			OverTemperature;		//系统过温保护值
	int16_t		OverTempCnt;				//系统过温保护时间阈值
	int16_t		UartErrCnt;					//掉线错误
	
	volatile uint8_t	System1msCnt;				//系统1ms计数
	volatile uint8_t	SystemProtectFlag;	//系统保护1ms标志
	
}SYS_Prot_TYPEDEF;


typedef struct{

	float OverCurrent;				//最后赋值到保护的过流保护值
	
	float	OverCurrent1;				//该通道刀头1过流保护值(单向)
	float	OverCurrent2;				//该通道刀头1过流保护值（往复）
	
	float	OverCurrent3;				//该通道刀头2过流保护值（单向）
	float OverCurrent4;				//该通道刀头2过流保护值（往复）

	volatile uint16_t	OverCurCnt;	//该通道刀头1过流保护计数值
	
	volatile uint16_t	CurCnt;	//最后赋值到保护的计数值保护值
	
	volatile uint16_t	CurCnt1;	//该通道刀头1过流保护计数值
	volatile uint16_t	CurCnt2;	//该通道刀头1过流保护计数值	
	volatile uint16_t	CurCnt3;	//该通道刀头1过流保护计数值
	volatile uint16_t	CurCnt4;	//该通道刀头1过流保护计数值
	
	int16_t	RunStallCnt1;			//刀头1 正/反转堵转保护			计数
	int16_t	RunStallCnt2;			//刀头1 往复运动转堵转保护	计数
	
	int16_t	RunStallCnt3;			//刀头2 正/反转堵转保护			计数
	int16_t	RunStallCnt4;			//刀头2 往复运动转堵转保护	计数
	

	float 	LosePhaseVal;					//此刻的电流AD值
	int32_t LosePhaseTim;					//积分时间
	float 	LosePhaseAllVal;			//电流积分滤波后的值
	float 	LosePhaseAllTempVal;	//电流积分缓存值（用于滤波）
	int32_t LosePhaseCnt;					//缺相判定计数时间阈值
	
	
	
}CH_Prot_TYPEDEF;


typedef struct{

	float CurMid;							//该通道电流中间值
	float CurNow;							//该通道电流瞬时值
	float CurLPF;							//该通道电流滤波值
	float Vbus;								//该通道经过BUCK电路调压后，施加在电机两端的电压

}ADC_TYPEDEF;

typedef struct{                      //保护相关
	float qLsDtq;
	float qRs;
	float qRsxisu;
	float qSpdxisu;
	float qEmf;
	float CalR;
	float CalV;
	float CalI;
	float qEmfLPF;
	float qLastI;
	float qIerr;
}EMF_TYPEDEF;

typedef struct{
	uint8_t  StopTimFg;			
	uint16_t StopTimCnt;	
}TIM_TYPEDEF;

typedef struct{

	volatile uint8_t	Status;			//该通道电机此时的状态
	volatile uint8_t	Start;			//允许启动标志
	volatile uint8_t	AllowRun;		//允许允许标志
	DIR_TYPEDEF				Dir;				//方向参数
	CH_Prot_TYPEDEF		Prot;				//电机保护参数
	ADC_TYPEDEF				ADC;				//该通道ADC的参数
	EMF_TYPEDEF				Emf;				//该通道刀头1的电机参数：电阻电感等
	TIM_TYPEDEF				Tim;				//通道时间参数

}CHANNEL__TYPEDEF;





typedef struct{                      //按键相关
  
	volatile uint8_t		Err;		//系统错误
	Log_TYPEDEF 				Log;			//系统逻辑
	SYSCTL__TYPEDEF			SysCtl;		//系统控制
	SYS_Prot_TYPEDEF		SysPort;	//系统保护参数
	CHANNEL__TYPEDEF		Ch1;			//通道1的全部参数
	CHANNEL__TYPEDEF		Ch2;			//通道2的全部参数
	
}APP2_TYPEDEF;			//有刷变量



void Youshua_ADCISRTasks(void);
void YoushuaInit(void);
//有刷状态机
void YoushuaMotor_Ctl(void);
void YoushuaMotor_Ct2(void);
extern void MCPwm_Init1(void);
extern void MCPwm_Init2(void);
extern APP2_TYPEDEF App2;

#endif




