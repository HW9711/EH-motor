
#include "tim.h"
#include "mcctl.h"
//有刷电机驱动配置
//初始化

void TIME_PWM_Start_Unipolarity(void)//单极性
{
	App.M1.Pwm.HigLowSpd = 1;
	#if defined (STM32_FOC)
	TIM1->CCMR1 &= 0xefff;                   //通道二 PWM 模式一
		if(App.M1.Pwm.HigLowSpd == 0){//低速档
			if(App.M1.Dir.Now == CW){
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);   //   |
				HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1); //   | 
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);    //    -----------
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);//              |
			}else{
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);    //              |
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);//              | 
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);   //    -----------
				HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2); //    |
			}
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);
		}else{//高速档
			if(App.M1.Dir.Now == CW){
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);   //   |
				HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3); //   | 
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);    //    -----------
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);//              |
			}else{
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);    //              |
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);//              | 
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);   //    -----------
				HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2); //    |
			}
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
		}
	#elif defined (AT32_FOC)
		TMR_CCxCmd(TMR1, TMR_Channel_1, TMR_CCx_Enable);  
		TMR_CCxCmd(TMR1, TMR_Channel_2, TMR_CCx_Enable);  
		TMR_CCxCmd(TMR1, TMR_Channel_3, TMR_CCx_Enable);  
		TMR_CCxNCmd(TMR1, TMR_Channel_1, TMR_CCxN_Enable);  
		TMR_CCxNCmd(TMR1, TMR_Channel_2, TMR_CCxN_Enable);  
		TMR_CCxNCmd(TMR1, TMR_Channel_3, TMR_CCxN_Enable);
	#endif
}

void TIME_PWM_Start_bipolar(void)//双极性
{
	App.M1.Pwm.HigLowSpd = 0;
	#if defined (STM32_FOC)
		if(App.M1.Pwm.HigLowSpd == 0){//低速档
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);
		}else{//高速档
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
		}
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
		TIM1->CCMR1 |= 0x7000;                   //通道二 PWM 模式二
		//TIM1->CCMR1 &= 0xefff;                   //通道二 PWM 模式一
	#elif defined (AT32_FOC)
		TMR_CCxCmd(TMR1, TMR_Channel_1, TMR_CCx_Enable);  
		TMR_CCxCmd(TMR1, TMR_Channel_2, TMR_CCx_Enable);  
		TMR_CCxCmd(TMR1, TMR_Channel_3, TMR_CCx_Enable);  
		TMR_CCxNCmd(TMR1, TMR_Channel_1, TMR_CCxN_Enable);  
		TMR_CCxNCmd(TMR1, TMR_Channel_2, TMR_CCxN_Enable);  
		TMR_CCxNCmd(TMR1, TMR_Channel_3, TMR_CCxN_Enable);
	#endif
}

void Brush_Unipolarity_Init(void){//单极性
	
}
void Brush_bipolar_Init(void){//双极性
	
}

//单极性控制
void Brush_Unipolarity_Ctl(void){
	
}

//双极性控制
void Brush_bipolar_Ctl(void){
	
}

void Set_PWM_Updata(void){//SetPwm 越大，力量越大
	#ifdef STM32_FOC
	if(App.M1.AllowRun == 1){
		if(App.M1.Pwm.Now < App.M1.Pwm.Set){
			App.M1.Pwm.Now +=1;
		}else if(App.M1.Pwm.Now > App.M1.Pwm.Set){
			App.M1.Pwm.Now -=1;
		}
		if(App.M1.Pwm.Now > (SYS_CORE_CLOCK/PWM_FREQUENCY)/2-1){
			App.M1.Pwm.Now = (SYS_CORE_CLOCK/PWM_FREQUENCY)/2-1;
		}else if(App.M1.Pwm.Now <= 0){
			App.M1.Pwm.Now = 0;
		}
		if(App.M1.Pwm.HigLowSpd == 0){//低速档 12
			if(App.M1.Dir.Now == CW){
				if(App.M1.MotorType == 1){
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,App.M1.Pwm.Now);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(SYS_CORE_CLOCK/PWM_FREQUENCY)/2-1);
				}else if(App.M1.MotorType == 2){
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,App.M1.Pwm.Now);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,App.M1.Pwm.Now);
				}
			}else{
				if(App.M1.MotorType == 1){
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,App.M1.Pwm.Now);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(SYS_CORE_CLOCK/PWM_FREQUENCY)/2-1);
				}else if(App.M1.MotorType == 2){
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,App.M1.Pwm.Now);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,App.M1.Pwm.Now);
				}
			}
		}else{                     //高速档 23
			if(App.M1.Dir.Now == CW){
				if(App.M1.MotorType == 1){
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,App.M1.Pwm.Now);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(SYS_CORE_CLOCK/PWM_FREQUENCY)/2-1);
				}else if(App.M1.MotorType == 2){
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,App.M1.Pwm.Now);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,App.M1.Pwm.Now);
				}
			}else{
				if(App.M1.MotorType == 1){
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,App.M1.Pwm.Now);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(SYS_CORE_CLOCK/PWM_FREQUENCY)/2-1);
				}else if(App.M1.MotorType == 2){
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,App.M1.Pwm.Now);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,App.M1.Pwm.Now);
				}
			}
		}
	}else{
		if(App.M1.MotorType == 1){
				App.M1.Pwm.Now = 0;
		}else if(App.M1.MotorType == 2){
				App.M1.Pwm.Now = (SYS_CORE_CLOCK/PWM_FREQUENCY)/4;
		}
	}
	#endif
}
