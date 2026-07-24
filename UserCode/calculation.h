#ifndef __CALCULA_H
#define __CALCULA_H

#include  "userpara.h"
#include "at32f4xx.h"

typedef struct{
   float          k;//系数
   float   				b;//y=kx+b
   float 					Out;//输出
   float 					In;//输入
	 unsigned short OutMax;
	 unsigned short OutMin;
	 unsigned short InMax;
	 unsigned short InMin;
	 int16_t FiltCnt;//滤波次数
	 signed char    Status;
}Mapping_TYPE;


//计算函数 输出
float Map_CalOut(float SetADC,float InPutMin,float InPutMax,float OutPutMin,float OutPutMax );

extern float Filter(float filerreturn,const float filterNum,const float depth);

float temp_data(int32_t ADCTemp, int32_t UpRes);
float temp_data2(int32_t adc);
float temp_data3(int32_t adc);
float AirPress_Cal(int32_t adc);
void Ctl_StatusCalc(void);

void Calculate(void);
#endif
