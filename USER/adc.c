#include "adc.h"
#include "at32f4xx_conf.h"
#include "mcctl.h"

void ADC_Calibration(ADC_Type* ADCx);
//ADC时钟使能
void RCC_Conf_ADC(void)
{
	RCC_ADCCLKConfig(RCC_APB2CLK_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_ADC2, ENABLE);

}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Conf_ADC(void)
{
	GPIO_InitType GPIO_InitStructure;

	//规则通道
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pins = GPIO_Pins_0; //温度
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_ANALOG;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pins = GPIO_Pins_1; //电压
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_ANALOG;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//注入通道
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pins = GPIO_Pins_2; //电压P
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_ANALOG;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pins = GPIO_Pins_4; //T2总电流
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_ANALOG;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void ADC_Init_App(void)
{
	DMA_InitType DMA_InitStructure;
	ADC_InitType ADC_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	//时钟
	RCC_Conf_ADC();
	//GPIO
	GPIO_Conf_ADC();
	
	//DMA配置 ADC1
	DMA_Reset(DMA1_Channel1);              
	DMA_DefaultInitParaConfig(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&ADC1->RDOR;          //指定Channelx的外设基地址  DMA_CPBAx  传输源或目标
	DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)&ADC1Value;   //数据储存地址    DMA_CMBAx     写入这个地址
	DMA_InitStructure.DMA_Direction             = DMA_DIR_PERIPHERALSRC;          //从外设读
	DMA_InitStructure.DMA_BufferSize            = ADC_BufferSize;                 //传输个数  对应DMA传输通道个数
	DMA_InitStructure.DMA_PeripheralInc         = DMA_PERIPHERALINC_DISABLE;      //指定外设地址寄存器是否递增  由于都是从&ADC1->RDOR中读取，所以不需要递增
	DMA_InitStructure.DMA_MemoryInc             = DMA_MEMORYINC_ENABLE;           //指定内存地址是否递增        
	DMA_InitStructure.DMA_PeripheralDataWidth   = DMA_PERIPHERALDATAWIDTH_HALFWORD;//数据宽度     外设
	DMA_InitStructure.DMA_MemoryDataWidth       = DMA_MEMORYDATAWIDTH_HALFWORD;   //数据宽度      内存
	DMA_InitStructure.DMA_Mode                  = DMA_MODE_CIRCULAR;              //CIRM 循环模式  启动之后，一直传输
	DMA_InitStructure.DMA_Priority              = DMA_PRIORITY_HIGH;              //优先级
	DMA_InitStructure.DMA_MTOM                  = DMA_MEMTOMEM_DISABLE;           //是否内存到内存传输  否
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);                                  //将上面的配置赋给通道x
	DMA_ChannelEnable(DMA1_Channel1, ENABLE);                                     //使能
	
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode              = ADC_Mode_InjecSimult;     //ADC模式   ADC1 ADC2工作在同步注入模式，
	ADC_InitStructure.ADC_ScanMode          = ENABLE;                  //扫描模式  对于多个通道有效
	ADC_InitStructure.ADC_ContinuousMode    = DISABLE;                   //连续模式
	ADC_InitStructure.ADC_ExternalTrig      = ADC_ExternalTrig_None;    //此方式为定时器触发	规则组
	ADC_InitStructure.ADC_DataAlign         = ADC_DataAlign_Right;      //对齐方式
	ADC_InitStructure.ADC_NumOfChannel      = ADC_BufferSize;                        //规定了顺序进行规则转换的 ADC 通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration */ 

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28_5);//温度TEMP
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28_5); //总电压
	/* Enable ADC1 external trigger conversion */ 
	ADC_ExternalTrigConvCtrl(ADC1, ENABLE);
	
	//注入通道配置
	/* Set injected sequencer length */
	ADC_InjectedSequencerLengthConfig(ADC1, 1);   //通道数
	/* ADC1 injected channel Configuration */

	ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7_5); //电压P


	/* ADC1 injected external trigger configuration */
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjec_TMR1_CC4);//触发方式

	ADC_ExternalTrigInjectedConvCtrl(ADC1,ENABLE);//注入通道开启外部触发
	
	//ADC2配置
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
//	ADC_InitStructure.ADC_Mode              = ADC_Mode_Independent;     //ADC模式   ADC1 ADC2工作在独立注入模式
	ADC_InitStructure.ADC_Mode              = ADC_Mode_InjecSimult;     //ADC模式   ADC1 ADC2工作在同步注入模式，
//	ADC_InitStructure.ADC_Mode              = ADC_Mode_InjecSimult_FastInterl;		//注入转换同时进行，快速交错模式：ADC1和ADC2的注入转换同时进行，交替触发，且触发频率较快。
	ADC_InitStructure.ADC_ScanMode          = ENABLE;                  //扫描模式  对于多个通道有效
	ADC_InitStructure.ADC_ContinuousMode    = DISABLE;                   //连续模式
	ADC_InitStructure.ADC_ExternalTrig      = ADC_ExternalTrig_None;    //在双ADC中，主ADC配置为定时器触发，从ADC配置成软件触发，说明书376页//规则组
	ADC_InitStructure.ADC_DataAlign         = ADC_DataAlign_Right;      //对齐方式
	ADC_InitStructure.ADC_NumOfChannel      = 0;                        //规定了顺序进行规则转换的 ADC 通道的数目
	ADC_Init(ADC2, &ADC_InitStructure);
	//注入通道配置
	/* Set injected sequencer length */
	ADC_InjectedSequencerLengthConfig(ADC2, 1);   //通道数
	
	/* ADC1 injected channel Configuration */ 
	
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_14, 1, ADC_SampleTime_7_5);//T2总电流

	
	/* ADC1 injected external trigger configuration */

	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjec_None);//触发方式

	ADC_ExternalTrigInjectedConvCtrl(ADC2,ENABLE);//注入通道开启外部触发
	
	
	ADC_DMACtrl(ADC1, ENABLE);
	ADC_Ctrl(ADC1, ENABLE);
	
	//ADC_DMACtrl(ADC2, ENABLE);
	ADC_Ctrl(ADC2, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//ADC采样中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
	ADC_INTConfig(ADC1,ADC_INT_JEC,ENABLE);//开启注入通道转换完成中断
//	ADC_INTConfig(ADC2,ADC_INT_JEC,ENABLE);//开启注入通道转换完成中断
	//校准
	ADC_Calibration(ADC1);
	//校准
	ADC_Calibration(ADC2);
}
//ADC 校准
void ADC_Calibration(ADC_Type* ADCx)
{
		/* Enable ADC1 reset calibration register */   
	ADC_RstCalibration(ADCx);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADCx));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADCx);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADCx));
}
