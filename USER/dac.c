#include "dac.h"
#include "mcctl.h"
void DAC_GPIO_Init(void);
void DAC_Base_Confi(void);


#define _DAC_PA4          0
#define _DAC_PA5					1



void DAC_Init_App(void){
	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_DAC, ENABLE);
	DAC_GPIO_Init(); 
	DAC_Base_Confi();
}
void DAC_GPIO_Init(void){
	GPIO_InitType     GPIO_InitStructure;
	#if _DAC_PA4 == 1
  GPIO_InitStructure.GPIO_Pins =  GPIO_Pins_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_ANALOG;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	#endif
	
	#if _DAC_PA5 == 1
  GPIO_InitStructure.GPIO_Pins =  GPIO_Pins_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_ANALOG;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	#endif
	
}
void DAC_Base_Confi(void)
{
	DAC_InitType            DAC_InitStructure;
	DAC_StructInit(&DAC_InitStructure);
	
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmsk_Bits11_0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	
	#if _DAC_PA4 == 1
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_Ctrl(DAC_Channel_1, ENABLE);
  DAC_SetChannel1Data(DAC_Align_12b_Right, MCPara[11]);
  DAC_SoftwareTriggerCtrl(DAC_Channel_1, ENABLE);
	#endif
	
	#if _DAC_PA5 == 1
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);
  DAC_Ctrl(DAC_Channel_2, ENABLE);
  DAC_SetChannel2Data(DAC_Align_12b_Right, MCPara[11]);
  DAC_SoftwareTriggerCtrl(DAC_Channel_2, ENABLE);
	#endif
}







