#include "userpara.h"
#include "math.h"
#include "mcctl.h"
#include "adc.h"

//无刷计算总电流
void Calculate(void){
	float temp = 0;
	if(App.Logic.Calculate_1msFlag ==1){
		App.Logic.Calculate_1msFlag = 0;
		//总电流
		temp = (float)get_shunt_t2_adc_value * ADC_CURRENT_SCALE;   
		//总电流滤波
		App.FB.Prot.AllCur = temp*(1-App.FB.Prot.CurrentLPFXiSHU)+ App.FB.Prot.AllCur*App.FB.Prot.CurrentLPFXiSHU;
		App.FB2.Prot.AllCur = App.FB.Prot.AllCur ;
	}
}


//滤波
float Filter(float filerreturn,const float filterNum,const float depth)
{
	filerreturn=filerreturn * (1.0f - depth) + (float)filterNum * depth;
	return filerreturn;
}

//电压换算成温度
//ADCTemp是ADC测量值，UpRes是分压电阻上分阻值
float temp_data(int32_t ADCTemp, int32_t UpRes)
{
	float temp;
	float Rt=0;
	float Rp=10000;
	float T2=273.15+25;
	float Bx=3950;
	float Ka=273.15;
	float vol=0;
	vol=(float)(ADCTemp)*(3.3/4096);  
	Rt=(vol*(float)UpRes)/(3.3f-vol);
	temp=1/(1/T2+log(Rt/Rp)/Bx)-Ka+0.5f;
	return temp;
}


float temp_data2(int32_t adc){
	float temp = adc * 3.3f/ 4096;
	return (temp - 0.46f)/0.02566f;
}


//IPM-BIPN60050C
float temp_data3(int32_t adc){
	float temp = adc * 3.3f / 4096;
	return (temp - 0.32f) / 0.026f;
}

//0-35MP 4-20mA 496.485-2482.424
float AirPress_Cal(int32_t adc){
	if(adc <= 496){
		return 0;
	}else if(adc >= 2482){
		return 35;
	}else {
		return (adc - 496) * 35.0f / (2482 - 496);
	}
	
}



//计算函数 输出
float Map_CalOut(float SetADC,float InPutMin,float InPutMax,float OutPutMin,float OutPutMax ){
	if(SetADC <= InPutMin){
		return OutPutMin;
	}else if(SetADC >= InPutMax){
		return OutPutMax;
	}else {
		return (SetADC - InPutMin) * (OutPutMax - OutPutMin) / (InPutMax - InPutMin) + OutPutMin;
	}
}


