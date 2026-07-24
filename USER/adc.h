#ifndef __ADC_H
#define __ADC_H
#include "at32f4xx.h"
#include  "userpara.h"

#define ADC_BufferSize 								2

#define get_shunt_t2_adc_value        ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1)

#define get_vbus_adc_value            ADC1Value[1]
#define get_temp_adc_value						ADC1Value[0]

#define get_Pvcc_adc_value			 			ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1)//ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2)//定义电压 ADC 采样值


#define ADC_SoftwareStart             ADC_SoftwareStartConvCtrl(ADC1, ENABLE)//软件触发ADC中断


void ADC_Init_App(void);		// = 0 抢占通道设置为以Timer1_CH4作为触发事件， =1 抢占通道设置为以Timer8_CH4作为触发事件
#endif
