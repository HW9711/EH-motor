#ifndef 		USERPARAMS_H
#define 		USERPARAMS_H

#define			Id_Ctrl_Max					1.0f


#define			_protect											1	//是否开启保护，0-不保护，1保护

#define 		PLL_Cur_Pi_SW								 0//是否电流闭环 0-不闭环 1-闭环
#define			_Relay_SW										1//0-电磁阀控制 1-充当刹车电阻用

#define			PVCC_PWM_EN					1	//是否启用BUCK电路	： 0-不使用BUCK电路	1-使用BUCK电路调节母线电压


#define 		use_HSI 										 	0//1-内部时钟 0-晶振
	
#define     PBCLK_PWM                    	(240000000ul)
//#define     PWM_FREQUENCY                	30000
#define     PWM_FREQUENCY                	16000
	
#define     PWM_FREQ                     	PWM_FREQUENCY   //注意要和main.h的stcube 设定的PWM_FREQUENCY一样
#define     PWM_PERIOD_COUNT             	((PBCLK_PWM/PWM_FREQ)/2-1)
#define     MAX_ADC_COUNT                	(float)(4095)    
#define     MAX_ADC_INPUT_VOLTAGE        	(float)(3.3f)     
#define     HALF_ADC_INPUT_VOLTAGE       	(float)(1.65f)
	
//#define     RSHUNT                       	(float)(0.013f)   //采样电阻3个R040
#define     RSHUNT                       	(float)(0.034f)  //采样电阻3个R100
//#define     RSHUNT                       	(float)(0.067f)  //采样电阻3个R24444444444444444444444444r00
#define     RS_HIGH                      	(float)(18.0f)  // 实测相电流及有刷总电流采集增益为18
#define     RS_LOW                       	(float)(1.0f)
#define     CAMP_SCALE                   	(float)((RS_HIGH/RS_LOW))
#define     MAX_BOARD_CURRENT            	(float)(MAX_ADC_INPUT_VOLTAGE/RSHUNT/CAMP_SCALE)      
#define     ADC_CURRENT_SCALE            	(float)(MAX_BOARD_CURRENT/MAX_ADC_COUNT)
	
#define     RSHUNT_t                    	(float)(0.01) 
#define 		CAMP_SCALE_t                 	(float)(18.0f)  // 实测无刷总电流采集增益为18
#define 		MAX_BOARD_CURRENT_t          	(float)(MAX_ADC_INPUT_VOLTAGE/RSHUNT_t/CAMP_SCALE_t)      
#define 		ADC_CURRENT_SCALE_t          	(float)(MAX_BOARD_CURRENT_t/MAX_ADC_COUNT)
	
//修改2	
#define     DCBUS_SENSE_TOP_RESISTOR     	(float)(100.0f)     
#define     DCBUS_SENSE_BOTTOM_RESISTOR  	(float)(10.0f)//(float)(6.8f)     
#define     DCBUS_SENSE_RATIO            	(float)(DCBUS_SENSE_BOTTOM_RESISTOR/(DCBUS_SENSE_BOTTOM_RESISTOR + DCBUS_SENSE_TOP_RESISTOR))
#define     VOLTAGE_ADC_TO_PHY_RATIO     	(float)(MAX_ADC_INPUT_VOLTAGE/(MAX_ADC_COUNT * DCBUS_SENSE_RATIO))
	

#define		DCBUSP_SENSE_TOP_RESISTOR				(float)(100.0f)
#define		DCBUSP_SENSE_BOTTOM_RESISTOR		(float)(10.0f)
#define		DCBUSP_SENSE_RATIO							(float)(DCBUSP_SENSE_BOTTOM_RESISTOR/(DCBUSP_SENSE_BOTTOM_RESISTOR + DCBUSP_SENSE_TOP_RESISTOR))
#define		VOLTAGEP_ADC_PVCC_PHY_RATIO			(float)(MAX_ADC_INPUT_VOLTAGE/(MAX_ADC_COUNT * DCBUSP_SENSE_RATIO))

//修改3	
#define     R_coefficient                	(float)(1000.0f)
#define     L_coefficient                	(float)(1000000.0f)
#define     BEMF_coefficient             	(float)(100.0f)
	
//开环切换到闭环的角度偏移值          	
#define     D_CURRCNTR_OUTMAX            	(float)(0.95f)
#define     Q_CURRCNTR_OUTMAX            	(float)(0.95f)
#define     MAX_DUTY                     	(PWM_PERIOD_COUNT * 0.95f)
#define			sqrtMax												0.98f
#define     LOOPTIME_SEC                 	(float)((1.0f)/PWM_FREQ)  // PWM Period - 50 uSec, 20Khz PWM
	
#define     ANGLE_2PI                    	(float)(2.0f*M_PI)
#define     CW                           	(1)  
#define     CCW                          	(-1) 

#define     myabs(Q)  										( (Q < 0) ? (-Q) : (Q) )
#define     mysign(Q) 										( (Q < 0) ? (-1) : (1) )
#define     __IO    											volatile

//PWM使能/失能		电机1--TMR1
#define 		PWM1H_ON               				TMR_CCxCmd(TMR1, TMR_Channel_1, TMR_CCx_Enable)       
#define 		PWM1H_OFF              				TMR_CCxCmd(TMR1, TMR_Channel_1, TMR_CCx_Disable)      

#define 		PWM2H_ON               				TMR_CCxCmd(TMR1, TMR_Channel_2, TMR_CCx_Enable)  
#define 		PWM2H_OFF              				TMR_CCxCmd(TMR1, TMR_Channel_2, TMR_CCx_Disable)

#define 		PWM3H_ON               				TMR_CCxCmd(TMR1, TMR_Channel_3, TMR_CCx_Enable)  
#define 		PWM3H_OFF              				TMR_CCxCmd(TMR1, TMR_Channel_3, TMR_CCx_Disable)
								
#define 		PWM1L_ON               				TMR_CCxNCmd(TMR1, TMR_Channel_1, TMR_CCxN_Enable)
#define 		PWM1L_OFF              				TMR_CCxNCmd(TMR1, TMR_Channel_1, TMR_CCxN_Disable)
#define 		PWM2L_ON               				TMR_CCxNCmd(TMR1, TMR_Channel_2, TMR_CCxN_Enable)
#define 		PWM2L_OFF              				TMR_CCxNCmd(TMR1, TMR_Channel_2, TMR_CCxN_Disable)
#define 		PWM3L_ON               				TMR_CCxNCmd(TMR1, TMR_Channel_3, TMR_CCxN_Enable)
#define 		PWM3L_OFF              				TMR_CCxNCmd(TMR1, TMR_Channel_3, TMR_CCxN_Disable)
								
#define 		PWM1(X)                				TMR_SetCompare1(TMR1, X)
#define 		PWM2(X)                				TMR_SetCompare2(TMR1, X)
#define 		PWM3(X)                				TMR_SetCompare3(TMR1, X)
#define 		PWM4(X)                				TMR_SetCompare4(TMR1, X)
#define 		PWM_ARR                				((PBCLK_PWM/PWM_FREQUENCY/2)-1)
#define 		charge_PWM             				(PWM_ARR/5)		//充电PWM




//PWM使能/失能		电机2---TMR8
#define 		PWM1H_ON2               				TMR_CCxCmd(TMR8, TMR_Channel_1, TMR_CCx_Enable)       
#define 		PWM1H_OFF2              				TMR_CCxCmd(TMR8, TMR_Channel_1, TMR_CCx_Disable)      

#define 		PWM2H_ON2               				TMR_CCxCmd(TMR8, TMR_Channel_2, TMR_CCx_Enable)  
#define 		PWM2H_OFF2              				TMR_CCxCmd(TMR8, TMR_Channel_2, TMR_CCx_Disable)

#define 		PWM3H_ON2               				TMR_CCxCmd(TMR8, TMR_Channel_3, TMR_CCx_Enable)  
#define 		PWM3H_OFF2              				TMR_CCxCmd(TMR8, TMR_Channel_3, TMR_CCx_Disable)
								
#define 		PWM1L_ON2               				TMR_CCxNCmd(TMR8, TMR_Channel_1, TMR_CCxN_Enable)
#define 		PWM1L_OFF2              				TMR_CCxNCmd(TMR8, TMR_Channel_1, TMR_CCxN_Disable)
#define 		PWM2L_ON2               				TMR_CCxNCmd(TMR8, TMR_Channel_2, TMR_CCxN_Enable)
#define 		PWM2L_OFF2              				TMR_CCxNCmd(TMR8, TMR_Channel_2, TMR_CCxN_Disable)
#define 		PWM3L_ON2               				TMR_CCxNCmd(TMR8, TMR_Channel_3, TMR_CCxN_Enable)
#define 		PWM3L_OFF2              				TMR_CCxNCmd(TMR8, TMR_Channel_3, TMR_CCxN_Disable)


#define 		PWM12(X)                				TMR_SetCompare1(TMR8, X)
#define 		PWM22(X)                				TMR_SetCompare2(TMR8, X)
#define 		PWM32(X)                				TMR_SetCompare3(TMR8, X)
#define 		PWM42(X)                				TMR_SetCompare4(TMR8, X)
#define 		PWM_ARR2                				((PBCLK_PWM/PWM_FREQUENCY/2)-1)
#define 		charge_PWM2             				(PWM_ARR/5)		//充电PWM


//霍尔传感器相关
//#define 		hall_tim                     	TMR4
//#define 		read_hall                    	get_hall()//(((GPIOC->IPTDT)>>6) & 0x07)
//#define 		Get_Hall_Tim_CCR             	hall_tim->CC1
//#define 		charge_PWM                   	(PWM_ARR/2)
//#define 		HALL_ARR                     	65535
//#define 		HALL_PSC                     	(240-1)
//#define 		hallsqu_spd_pi_cyc           	2 //霍尔方波 速度PI 计算 周期 ms
//#define 		Hall_Tim_Init_Ctl            	Hall_Tim_Init(hall_tim)
//#define 		Hall_Tim_Enable_Ctl          	TMR_Cmd(hall_tim, ENABLE)

#define 		read_enoc              				return_enoc_val(TMR3)//TMR2 -> CNT)
#define 		set_enoc	              			(TMR3 -> CNT)

#define 		read_enoc2              				return_enoc_val(TMR4)//TMR2 -> CNT)
#define 		set_enoc2	              			(TMR4 -> CNT)
	
//刹车电阻频率	
#define 		OVR_PWM_Fru                  	(240000000 / 10000)
#define 		Debug_vis                    	0//是否开启jlink 打印输出功能

#endif
