#ifndef _MCCTL_H    
#define _MCCTL_H

#include  "stdint.h"
#include  "gpio.h"
#include  "userpara.h"
#include  "calculation.h"
#include  "mclib.h"

#define ParaNum 145     //参数个数
#define StatusNum 15   //状态参数个数
#define SpeNum 		15   //状态参数个数
typedef struct{              //逻辑相关
	uint8_t Start;             //启动控制
	
	uint8_t CalFg;
	
	uint16_t RunTim;
	uint16_t StopTim;
	uint8_t StopTimFg;
	
	uint8_t Stop1msFg;
	uint32_t Stop1msCnt;
	
	
	uint8_t CtlMode; // = 0 按键控制， = 1 指令控制
	uint16_t ZLSetSpd;//指令设置转速
	int32_t ZLTempSetSpd;
	uint16_t UartStart;
	
	uint8_t OperaFg; //=0未启动 =1启动  =0防飞车 =1可以启动
	
	float MaxCur;
	
	uint8_t MotorType;//=0 24V =12V
	uint32_t MotorFlashAddr;
	
	
	int32_t StopTimCnt;
	uint8_t StopTim1msFg;
	
	uint16_t CanOpenAddr;//Can地址
	
	uint8_t TestMode;
	int8_t TestDir;
	int16_t TestSpd;
	
//	int16_t VBusNowPWM;
//	int16_t VBusSetPWM;
	
}LOG_TYPEDEF;



typedef struct{                      //按键相关
  int8_t Run;
	int8_t Dir;
	int8_t Brake;
	
	int8_t StadyFg;
	
	int8_t Sw1;
	int8_t Sw2;
	int8_t Sw3;
	
}KEY_TYPEDEF;


typedef struct{                      //时间相关
  uint8_t  StaLedCyc;                //状态灯闪烁周期
	uint8_t CheckCyc;                  //保护检测
	
	uint8_t Cyc1msCnt;
	uint8_t Cyc1msFg;
	uint16_t Cyc500msCnt;
	
	uint8_t CanHeart1msFg;
	uint16_t CanHeart1msCnt;
	
	uint8_t CanStartFg;
	
	uint8_t	CycPWM1msCnt;
	uint8_t CycPWMSetFg;
	
}TIME_TYPEDEF;

typedef  struct{
	
	float ReadMidAng;//当前hall直接读数组中的角度
	float MidAng;
	float ReadMidAngPrev;//上一次hall直接读数组中的角度
	
	float Ang;//当前角度
	float AngAvg;//角度平均
	
	volatile uint8_t Val;
	volatile uint8_t OldVal;
	
	//hall自学习部分
	uint16_t SdSunXuCnt;
	float SdRecAng[8];
	float SdSaveAng[8];
	uint16_t SdSaveAng_Q15[8];
	int16_t SdAngQuYu[8];
	int16_t SdAngQuYuPJ;
	int16_t SdAngQuYu_PJ[8];
	int32_t SdAngQuYu_AllPJ;
	uint8_t SdJiaoChaFg;
	
	float Hall_Time;
	uint8_t  Hall_Change_Flag;
	

}HALL_TYPEDEF;

typedef struct{
	uint8_t HallorPll;				//计算取哪个速度 2-Hall 1-Pll 0-无
	uint8_t HallStadySta;     //hall相角学习标志位 0-无学习 1-正在学习 2-学习完成 3-保存数据 4-学习完成
	uint8_t HallAllRun;       //hallfoc时允许启动标志位 = 1允许启动 在学习时允许，在学习完之后正常情况下允许

}HPLL_TYPEDEF;



typedef enum{
    POS_WAIT = 0,//等待
    POS_STOP,//停止
    POS_INIT,//找零 门
		CTLS_SPD,//运行 如果是在供弹电机情况下，起初先按圈数运行
    CTLS_RUN,//运行 如果是在供弹电机情况下，圈数走完之后，立即停止
		CTLS_POS,//运行 如果是在供弹电机情况下，无运动时，要做锁轴功能
}POS_STUTYPEDEF;





typedef struct{
	int64_t NowCnt;
	int64_t NowOffSet;
	int64_t ZNowCnt;
	
	int64_t SetCnt;
	int64_t OldSetCnt;
	
	POS_STUTYPEDEF PosStatus;//位置控制状态
	uint8_t PosArriFg;//到位置标志位
	uint8_t PosLocaFg;//电机位置找零完成标志位
	uint8_t PosStart;//运行到设定位置标志位
	
	uint8_t CheckCur1msFg;
	int16_t CheckOCCnt;
	uint16_t LocaTimCnt;
	uint16_t CheckStallCnt;
}POS_TYPEDEF;


typedef struct{                      //速度相关
  float  Set;
	float  Now;
	float  RadSet;
	float  MinRad;//最小rad 由最小速度转换
	float  RampupDelta;
  float  RampdownDelta;
	
	float  SetRamp;
	
	float  SetSpdADCLPF;
	int16_t SetSpdADCOKCnt;
	uint8_t SetSpdADCOKFg;
	
	float Id_Up_Down;
	uint8_t Id_Cycle;
	
}SPD_TYPEDEF;


typedef struct{                      //速度相关
  float VescSpd;
	float NowRad;			//现在的角速度
	float OldRad;			// 过去一个时刻的角度
	float NowAng;			// 现在角度
	float Dt;				//△t
}CalSpd_TYPEDEF;

typedef struct{                      //计算相关
  float  BusCur;                     //总电流
	float  BusVol;                     //总电压 
	float  Torque;                     //扭矩
	float  Torque1;											//扭矩
	float  Power;
	float  Temper;                     //温度
	float  AllCur;
	float  AllCurLPF;
	float  BusVolP;
}CUR_TYPEDEF;

typedef struct{                      //开环相关
	uint8_t  AngCtlMode;    					 //角度开环还是闭环
  int8_t   OpenLoop;                 //开环运行
	float    Ang;                      //开关角度
	uint32_t LocateTimCnt;             //定位时间次数计算
	
	uint32_t LocateTimSum;             //定位时间次数总数
	float    RampupRad2SecLooptime;    //设置速度的角度长度
	float    RampAngleRadsPerSec;      //角度的累加
	float    RampupInc;                //给角度累加的大小
	
  float    RampupCurQ;               //加速Q值
	
  float    RampupCurD;    					 //加速D值
	float    CloLoopCurD;    					 //闭环
	float    CloCurD_Up;
	float    CloCurD_Dw;
	float    CloCurD_Max;
	float    CloCurD_Err;
	float    LockCurQ;      					 //定位Q值
	int8_t   ChangeMode;         			 //开环到闭环的状态变更，仅执行一次
	
	int32_t  Close_Open_Time;
	
}OPEN_TYPEDEF;

typedef struct{                      //方向相关
  int8_t   Set;                      	//设置的方向
	int8_t   Now;                      //当前方向

}DIR_TYPEDEF;

typedef struct{                      //保护相关
	int16_t RunStallCnt1;               //堵转保护时间
	int16_t RunStallCnt2;               //堵转保护时间
	int16_t LoseSpdCnt;                //失速保护时间
	int16_t UndVolCnt;                 //低压保护时间
	int16_t OverVolCnt;                //过压保护时间
	int16_t OverCurCnt;                //过流保护时间
	int16_t OverTempCnt;
	int16_t OverTempCnt1;
	int16_t HallErrCnt;
	float UnderVoltage;
	float OverVoltage;
	float OverCurrent;
	
	
	int16_t LongOverCurCnt;                //过流保护时间
	float LongOverCurrent;
	
	
	//抖动保护
	float UndSpd1;
	float MeaEsqu;
	
	uint16_t UnVolRecVal;//欠压恢复电压
	uint16_t OvVolRecVal;//过压恢复电压
	uint16_t OvTempRecVal;//过温恢复温度
	
	uint8_t PhaseLossSta;
	uint16_t PhaseLossNum;
	
	float PhaseLossACnt;
	float PhaseLossBCnt;
	float PhaseLossCCnt;
	
	float PhaseLossAVal;
	float PhaseLossBVal;
	float PhaseLossCVal;
	
	float tempa_b;
	float tempb_c;
	float tempc_a;
	float tempmax;
	float tempmin;
	
	//缺相检测
	int16_t CLackCnt;                  //缺相
	int16_t BLackCnt;                  //缺相
	int16_t ALackCnt;                  //缺相

	
	
	
}PROT_TYPEDEF;


typedef struct{//串口相关
	u8  Tim1msFg;
	u16 Tim1msCnt;
	u8  Sta;
}UART_TYPEDEF;


typedef struct{              //PWM
	uint8_t FrePwmUpdata;
	uint16_t IC2Value;//记录周期计数
	float DutyCycle;//占空比
	float Frequency;//频率
	int16_t FreErrCnt;//无pwm输入
	uint8_t FreCycOK;
	int16_t CheOkCnt;
	uint8_t CheOkFg;
	
}PWMIN_TYPEDEF;



typedef  struct{
	int8_t Sta;//0-未hall校准 1-已经hall校准，未Z轴校准，2-已经Z轴校准。
	uint8_t CalAngOffsetFg;
	
	uint16_t Now;
	
	float  AngOffset;
	float tempOffset;
	int16_t AngOffsetErrCnt;
	float  NowAngOffset;
	
	int32_t  EZOffset;
	int32_t  EZNowEnoc;
	int32_t  EZSetEnoc;
	
	int32_t  Enoc_I;
	uint16_t Enoc_I_Cnt;
	int16_t  Enoc_I_PJ;//ZI平均值
	
	uint16_t Enoc_Long;
	
	int16_t Enoc_Err;
	uint16_t Old_Enoc;
	uint16_t Enoc_Mid;
}Enoc_TYPEDEF;




typedef enum
{ 
  E_NONE         = 0,         //0无故障        
	E_FAIL         ,         //1模块保护         
	E_OC1          ,         //2过流保护方式1   
	E_OV           ,         //3过压保护         
	E_UV           ,         //4欠压保护         
	E_RUNSTALL     ,         //5运行中被堵故障
	E_Temp  	 		 ,         //6驱动器过温保护
	E_SAVE         ,				//7保存参数错误
	E_OVR         ,        //8刹车时间过长错误
	E_ENOC       ,				//9编码器错误
	E_Start				,				//10开环检测错误
	E_NOHALL  	 	 , 				//11霍尔断线错误
	E_SDAHALL      ,				//12霍尔学习错误
	E_HandShake			,				//13通讯错误
	E_PhaseLoss     ,    		//14缺相
	E_PosDrag			,					//15有Hall拖动错误
	
	
}Error_TypeDef;


//
#define ERR_WL_Phase_Loss    	(uint32_t)(1 << 0)	//缺相
#define ERR_WL_ENOC_Loss    	(uint32_t)(1 << 1)	//编码器
#define ERR_WL_Angle_Learn		(uint32_t)(1 << 2)	//相角学习
#define ERR_WL_Hall_Loss    	(uint32_t)(1 << 3)	//霍尔
#define ERR_WL_RUNSTALL				(uint32_t)(1 << 4)	//堵转
#define ERR_WL_OCUR						(uint32_t)(1 << 5)	//过流
#define ERR_WL_FAIL						(uint32_t)(1 << 6)	//硬件保护
#define ERR_WL_PosLoca				(uint32_t)(1 << 7)	//回原错误

//
#define ERR_GD_Phase_Loss    	(uint32_t)(1 << 8)	//缺相
#define ERR_GD_ENOC_Loss    	(uint32_t)(1 << 9)	//编码器
#define ERR_GD_Angle_Learn		(uint32_t)(1 << 10)	//相角学习
#define ERR_GD_Hall_Loss    	(uint32_t)(1 << 11)	//霍尔
#define ERR_GD_RUNSTALL				(uint32_t)(1 << 12)	//堵转
#define ERR_GD_OCUR						(uint32_t)(1 << 13)	//过流
#define ERR_GD_FAIL						(uint32_t)(1 << 14)	//硬件保护
#define ERR_GD_Pri						(uint32_t)(1 << 15)	//接近开关

//系统错误相关
#define ERR_XT_Brake_Timeout  (uint32_t)(1 << 16)	//刹车电机开启超时
#define ERR_XT_Save_MCPara    (uint32_t)(1 << 17)	//保存参数错误
#define ERR_XT_Erase_MCPara		(uint32_t)(1 << 18)	//无
#define ERR_XT_Overtemp	    	(uint32_t)(1 << 19)	//过温
#define ERR_XT_Undervol				(uint32_t)(1 << 20)	//欠压
#define ERR_XT_Overvol				(uint32_t)(1 << 21)	//过压








typedef struct{
	
	volatile uint8_t 			CtlMode;			//逻辑状态机的控制模式 0：停止	1：正转	2：反转	3：往返
	volatile int8_t				Set_Dir;			//  (-1): CCW		(0) 停止	（1）CW 
	volatile uint8_t			Motor_Kind;			//  -1:电机1		-2：电机2
	volatile uint32_t			Set_Spd;			//设置转速
	
	volatile uint8_t			u8WfFru;			//往返的频率，0-100//往复模式频率
	volatile uint16_t u16WfCyc;//往复模式周期
	volatile uint8_t u8MaxSpdFru;//最大转速频率
	volatile uint8_t Mode31msFg;
	volatile uint32_t Mode3TimCnt;
	volatile uint8_t Mode3Cha;
	volatile uint8_t Logic1msCnt;
	volatile uint8_t Hall_Change_TimOKFlag;
	volatile uint32_t Hall_Change_TimCnt;
	
	volatile uint8_t LogicHall_or_pll;		//指令设置的霍尔或者无霍尔
	volatile uint8_t Change_Hall;					//需要去改变的Hall类型
	volatile uint8_t Change_Kind;					//需要去改变的电机类型
	volatile uint8_t Change_Mode;					//需要去改变的电机类型
	volatile uint8_t Hall_Run_Stady_Flag;
	
	volatile uint16_t Motor_Kind_Change_Tim;	//切换电机计时
	volatile uint16_t Motor_Hall_Change_Tim;	//切换霍尔计时
	volatile uint16_t Motor_Stop_Start_Tim;	//启停最短时间
	volatile uint8_t Motor_Kind_Change_Tim_Cnt;		//切换电机最短时间
	volatile uint16_t Motor_Hall_Change_Tim_Cnt;	//切换Hall最短时间
	volatile uint16_t Motor_Stop_Start_Tim_Cnt;	//启停最短时间
	volatile uint8_t	Motor_Stop_StartFlag;
	
	volatile uint8_t	Brake_Sign;		//刹车标志
	volatile uint8_t	Brake_Kind;		//刹车种类	0:没有刹车 1:停机刹车 2:切换电机种类刹车 3:切换电机模式刹车 4:切换霍尔刹车 5：快速正反转调用刹车
	volatile uint8_t  Brake_StopTimOK;	//停止时间到达
	volatile uint8_t	Brake_StopPwmOK;	//停止PWM
	volatile uint8_t	Calculate_1msFlag;				//
	volatile uint8_t	Brake_TimeCnt;
	int16_t VBusNowPWM;		//总PWM控制：现在的PWM值
	int16_t VBusSetPWM;		//总PWM控制：目标的PWM值
	volatile uint8_t			Brak_OkFlag;							//完成刹车标志
	
	volatile uint32_t		HandShakeProtTim;				//通讯保护时间，多久没有通讯就报错
	volatile uint32_t		HandShakeProtCnt;				//通讯保护计数
	
	volatile uint32_t MotorFlashAddr1;
	volatile uint32_t MotorFlashAddr2;
	
	volatile int32_t	UasrtErrRestCnt;			//串口错误重启计数
	
}MOTOR_CONTROL;


typedef struct
{
	volatile float Last_Ang;
	volatile float Now_Ang;
	volatile float Spd;
	volatile float RowSpd;
	volatile float dt;
	volatile float Hall_Cnt;
	volatile uint16_t Tim500ms;
	
}HALL_SPD_TYPEDEF;



typedef struct{
	
	//设定圈数
	__IO uint16_t SetTurns;//设定的圈数
	__IO uint16_t NowTurns;//当前圈数
	
  __IO uint8_t SetWeiNum;//
	
	__IO uint8_t RelaySta;//电磁阀状态控制
	__IO int64_t RelayAng;//电磁阀状态控制
	
	__IO uint8_t PrsINSendFg;//接近开关触发一次
	__IO uint64_t PrsINSendCnt;//接近开关触发一次
	__IO uint64_t OldPrsINSendCnt;//接近开关触发一次
	__IO uint64_t Old1PrsINSendCnt;//接近开关触发一次
	__IO int64_t PrsINSendPos;//触发是的位置
	__IO int64_t OldPos;
	__IO uint8_t CanHear;
	__IO uint8_t CanRxOKFg;
	__IO float AirPress;
	
	__IO float BraVBus;
}Ctl_TYPEDEF;


typedef struct{              //PWMOut
	uint16_t Set; 
	int32_t Now;
	uint16_t MaxSet;
	uint8_t Updata1msFg;
	int16_t TimCnt;
	
	uint8_t TimSta;
}PWM_TYPEDEF;


typedef struct{              //开环openloop
	
	volatile uint8_t Phase;					//开环阶段！
	float Step_Min_Tim;				//换相最小时间，也就是换最后一相的时间
	float Step_temp_tim;			//每一个规划的时间临时变量
	volatile uint16_t Step_Delay_Tim[64];	//换相间隔时间规划
	volatile uint16_t StepCnt;				//开环执行到的步数
	
	volatile uint16_t PwmUp ;				//开环PWM爬升
	volatile uint16_t MinPwm;				//开环最小PWM输出
	volatile uint16_t MaxPwm;				//开环最大PWM输出
	
	volatile uint8_t OpenStart1msFg;			//开环1ms标志位
	volatile uint8_t StartHallOkFg;			//检测相序正确标志位			-0 相序还未正确，-1相序正确
	volatile uint8_t StepRampCnt;			//换相步数计数
	volatile int8_t HallStartStepCnt;		//换相步数
	volatile int8_t HallVal;					//开环换相霍尔值
	
	volatile uint16_t OpenStartTimCnt; 		//换相时间计数
	
	volatile uint8_t  Haless_Val;			//无霍尔开环的反电动势hall值
	volatile uint8_t  Haless_OldVal;			//无霍尔开环的反电动势hall的old值
	
	volatile uint16_t StartHallOkCnt;			//判定启动成功次数标志
	
	volatile uint8_t DebounceCnt;				//消抖次数
	
	volatile uint8_t En1A_Val;						//A相比较器输出值
	volatile uint8_t En1B_Val;						//B相比较器输出值
	volatile uint8_t En1C_Val;						//C相比较器输出值
	
	volatile uint32_t Spd_NotHallCnt;				//无霍尔速度估算计数
	volatile uint16_t SpdCnt_Arr[6];				//速度滑动滤波数组
	volatile uint64_t SpdSum;								//速度数组和
	volatile uint32_t SpdSumCheck;					//速度数组和检验，用来判定是否堵转
	volatile uint16_t OpenLoopTime;					//开环时间
	volatile uint8_t	ClossLoopFlg;					//切闭环标志位
	
}OPEN_LOOP;


typedef struct{              //PWMOut
	
	float CurrentLPFXiSHU;		//该通道电流滤波系数
	
  float  BusCur;                     //总电流
	float  BusVol;                     //总电压 
	
	float  Power;
	float  Temper;                     //温度
	
	float  AllCur;
	float  AllCurLPF;
	
	float  BusVolP;
	
	float UnderVoltage;				//欠压保护值
	uint16_t UndVolCnt;
	float OverVoltage; 				//过压保护值
	uint16_t OverVolCnt;
	float OverTemper;					//过温保护值
	uint16_t OverTempCnt1;		
	
	float OverCurrent;				//最后赋予保护的值
	uint16_t OverCurrentCnt;			//保护时间计数
	uint32_t LongOverCurrentCnt;	//长时间大电流保护计数

	int16_t RunStallCnt1;               //堵转保护时间
	int16_t RunStallCnt2;               //堵转保护时间
	
	
	float OverCurrent1;				//有Hall单向过流保护值	
	float OverCurrentCnt1;			//保护时间计数	
	float OverCurrent2;				//有Hall往复过流保护值
	float OverCurrentCnt2;			//保护时间计数	
	float OverCurrent3;				//无Hall单向过流保护值
	float OverCurrentCnt3;			//保护时间计数	
	float OverCurrent4;				//长时间大电流过流保护值	
	float OverCurrentCnt4;			//保护时间计数		
	float OverCurrent5;				//备用
	float OverCurrent6;				//备用

	
	volatile uint32_t		HallDragCnt;
	
	
}Prot_TYPEDEF;


typedef struct{     
	volatile 	uint8_t Status;			//方波状态机
	volatile uint8_t	AllRun;			//允许运行标志（底层）
	volatile uint8_t	Start;			//允许启动标志（应用层）
	volatile uint8_t	Err;				//方波错误
	OPEN_LOOP					OpenLoop;		//开环参数
	Prot_TYPEDEF 			Prot;				//保护参数
	DIR_TYPEDEF				Dir;				//方向参数
	
	volatile uint8_t  StaLedCyc;                //状态灯闪烁周期
	volatile uint8_t CheckCyc;                  //保护检测
	volatile uint8_t Cyc1msCnt;									//1ms计数
	volatile uint8_t Cyc1msFg;									//1ms标志
	volatile uint16_t Cyc500msCnt;							//500ms计数
	
	volatile uint8_t HallVal;			//Hall值
	volatile uint8_t OldHallVal;	//Hall旧值
	
	volatile float FB_Cnt_Time;		//
	volatile uint8_t CalSpdFg;
	float Rad_Now;			//计算角速度
	float Spd_Now;			//计算转速
	float Spd_Enable;		//计数器溢出标志，		=0，Hall正常运行， =1：Hall不动/异常，测量转速不给值
	float Rad_Set;			//设置的角速度
	float Spd_Set;			//设置的转速
	
	volatile uint8_t Cyc_Hall;		//速度计算周期	：结果多少次霍尔才计算一次转速
	volatile uint8_t FB1ms_Cnt;				//1ms计数
	volatile uint8_t ChaTim1msFg;			//充电1ms标志
	volatile uint8_t ChaTim1msCnt;			//充电计时
	
	volatile uint8_t Rota;			//现在的方向
	volatile uint8_t Bra1msFg;		//刹车标志
	volatile uint16_t BraTimCnt;		//刹车计时
	volatile uint8_t Bra1msCnt;		//刹车1ms计时
	volatile uint8_t Chang_Flag;	//允许变化方向标志
	
 volatile uint32_t Hall_Timer_Cnt;	//两个中断直接的时间间隔，us
 volatile uint32_t Hall_Timer_Sum;	//一个周期的总时间
 volatile uint32_t Hall_Timer_Sum_Now;	//一个周期的总时间
 	float RadFilTemp1 ;						
	float RadFilTemp2 ;
}FANGBO_TYPEDEF;


typedef enum{                      //方波有感状态机
  HS_WAIT =0,                    	//0 等待
	HS_INIT,                          //1 初始化
	HS_CHARGE,                           //2 充电
	HS_START,                          //3 启动
	HS_RUN,                          //4 运行
	HS_BRA,                          //5 刹车
	HS_STOP,                          //6 停止
	HS_ERR,                          //7 错误
	HS_POSBRA,												//8刹车停止位置状态
	HS_POSDRAG,												//9拖动模式
	
}HALLSQUSTA;




typedef struct{                     

	volatile uint8_t Start;						//位置控制允许标志
	volatile uint8_t PosFlag;					//需要产生一次位置拖动
	volatile uint8_t LockPosStatus; 	//目前拖动的状态		0：没有步数，等待设置步数， 1：主机设置了步数，锁定需要执行的步数，
																		//									2：正在按步数拖动， 				3：拖动完成，强定位一段时间，拖动结束回到0
	
	volatile uint32_t PosSetSteps;		//位置模式设定的拖动步数
	uint16_t 					DragTim;				//拖动的时间
	volatile uint16_t DragTimCnt;			//拖动的时间计数
	volatile uint8_t 	DragFlash;			//=0拖动完成， = 1正在拖动
	uint16_t					LocatedTim;			//定位时间
	uint16_t					LocatedTimeCnt;	//定位时间计数
	
	volatile uint16_t	PlanSteps;			//规划的步数
	volatile uint8_t	PosHallVal;			//现在的Hall值
	volatile uint8_t	PosHallValNext;	//换相的Hall值
	volatile int8_t		PosHalli;				//换相值
	volatile int8_t		PosNow1;				//通道1现在Hall的位置，以这个位置为起点(是进入位置拖动模式，不是全局)	
	volatile int8_t		PosNow2;				//通道2现在Hall的位置，以这个位置为起点(是进入位置拖动模式，不是全局)	

	volatile uint8_t	LockFlag1;			//定位标志， 0 ：还没有位置定位过，	1：位置定位过
	volatile uint8_t	LockFlag2;			//定位标志， 0 ：还没有位置定位过，	1：位置定位过
	
	volatile uint16_t	WaitStopTim1;		//等待刹车结束，开始位置回拉的时间
	volatile uint16_t	WaitStopTim2;		//等待刹车结束，开始位置回拉的时间
	volatile uint16_t	WaitStopTimCnt1;		//等待刹车结束，开始位置回拉的时间
	volatile uint16_t	WaitStopTimCnt2;		//等待刹车结束，开始位置回拉的时间
	
	float		ReduceRatio1;				//减速比
	float		ReduceRatio2;				//减速比
	
	float		OutSideAngle1;				//一个Hall变换对应外圈变换角度
	float		OutSideAngle2;				//一个Hall变换对应外圈变换角度
	
	float 	OutSidePosition1;				//外边的位置
	float 	OutSidePosition2;				//外边的位置
	
	volatile int16_t	PlanPosStep1;	//规划的位置步数
	volatile int16_t	PlanPosStep2;	//规划的位置步数	
	
	volatile int8_t  Tempi1;			//刹车后的数组值
	volatile int8_t  Tempi2;			//刹车后的数组值
	
	int8_t		PlanStepsMAX1;		//规划步数最大值，模值；
	int8_t		PlanStepsMAX2;		//规划步数最大值，模值；
	
	volatile uint8_t	HallValue1;	//用于位置规划的Hall值，避免和测速的产生影响
	volatile uint8_t	HallValue2;	//用于位置规划的Hall值，避免和测速的产生影响
	
	volatile uint8_t	HallOldValue1;	//用于位置规划的Hall过去值，避免和测速的产生影响
	volatile uint8_t	HallOldValue2;	//用于位置规划的Hall过去值，避免和测速的产生影响
	
	
}POS_CONTROL;

typedef struct{              //YoushuaCH
	volatile uint8_t	Status;			//该通道电机此时的状态
	volatile uint8_t	Start;			//允许启动标志
	volatile uint8_t	AllowRun;		//允许允许标志
	
	DIR_TYPEDEF				Dir;				//方向参数

}CH_TYPEDEF;





typedef struct{
	
	volatile uint8_t Err;	
	MOTOR_CONTROL Logic;		//公用逻辑
	FANGBO_TYPEDEF FB;			//方波有感电机1变量
	FANGBO_TYPEDEF FB2;			//方波有感电机2变量
	POS_CONTROL		Pos;			//位置控制变量
}APP_TYPEDEF;

extern  APP_TYPEDEF App;


extern uint16_t MCPara[ParaNum];
extern uint16_t MCPara2[ParaNum];

extern uint16_t SMCPara[ParaNum];
extern uint16_t SMCPara2[ParaNum];

extern int32_t MCStatus[StatusNum];
extern __IO uint16_t ADC1Value[3];	

void Motor_Ctl(void);
void Motor_Ctl2(void);


void Hard_Init(void);
void Soft_Init(void);
void Pos_Uart_Init(void);
void Pos_Init(void);
//void TIME_PWM_Stop_3Channel(TMR_Type *TMRx);
//void TIME_PWM_Start_3Channel(TMR_Type *TMRx);
//void TIME_PWM_Brake_3Channel(TMR_Type *TMRx);

//#define AD_NUM  3
enum SEQADCDMA{ 
	AD_Vol,                           //0，电压
	AD_VolP,						  //1, 电压P
	AD_T1,														//2，T1总电流
	AD_T2,														//3，T2总电流
	AD_TEMP,						  //4，温度Temp
	
};
#define TRUE   1       
#define FALSE  0
#endif 
