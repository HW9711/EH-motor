#ifndef __POS_PID_H
#define __POS_PID_H

#include "mclib.h"






typedef struct{                      //位置控制相关
	//记录起点和终点的中间位置
	int64_t MidPos;
	//初始化起点最小速度
	int64_t StaMinSpd; //起点最小速度
	//初始化终点最小速度
	int64_t EndMinSpd; //终点最小速度
	//初始化全程最大速度
	int64_t AllMaxSpd; //全程最大速度
	//初始化加速 斜率
	int64_t UpS;
	//初始化减速 斜率
	int64_t DownS;
	//记录起始位置
	int64_t OldNow;
	
}POSSpd_TYPEDEF;




extern POSSpd_TYPEDEF 				PosSpd;
extern POSSpd_TYPEDEF 				PosSpd2;




void Pos_SetSpd_Init(void);
void Pos2_SetSpd_Init(void);
int32_t Pos_SetSpd(int64_t nowpos,POSSpd_TYPEDEF posspd);
int32_t Pos_SetSpd2(int64_t nowpos,POSSpd_TYPEDEF posspd);
void Pos_Sta(void);
void Pos2_Sta(void);


#endif

