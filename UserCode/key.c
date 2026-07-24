#include "tim.h"
#include "multi_button.h"
#include "key.h"
#include "gpio.h"
#include "mcctl.h"

#if key_run == 1
	struct Button KeyRun;
	uint8_t Read_RUN_GPIO(void) 
	{
		return KAYRUN;
	}
#endif

#if key_dir == 1
	struct Button KeyDir;
	uint8_t Read_DIR_GPIO(void) 
	{
		return KAYDIR;
	}
#endif
	
#if key_HallSdy == 1
	struct Button KeyStady;
	uint8_t Read_Stady_GPIO(void) 
	{
		return KAYStady;
	}
#endif
	
#if key_EStop == 1
	struct Button KeyEStop;
	uint8_t Read_EStop_GPIO(void) 
	{
		return KEYEStop;
	}
#endif
	

#if Key_SW1 == 1
	struct Button KeySW1;
	uint8_t Read_SW1_GPIO(void) 
	{
		return KAYSW1;
	}
#endif
	
	
#if Key_SW2 == 1
	struct Button KeySW2;
	uint8_t Read_SW2_GPIO(void) 
	{
		return KAYSW2;
	}
#endif

#if Key_SW3 == 1
	struct Button KeySW3;
	uint8_t Read_SW3_GPIO(void) 
	{
		return KAYSW3;
	}
#endif


//按键相关初始化
void Key_Init_App(void)
{
	#if key_run == 1
	button_init(&KeyRun,Read_RUN_GPIO,0);
	button_start(&KeyRun);
	#endif
	#if key_dir == 1
	button_init(&KeyDir,Read_DIR_GPIO,0);
	button_start(&KeyDir);
	#endif
	#if key_brake == 1
	button_init(&KeyBrake,Read_BRAKE_GPIO,0);
	button_start(&KeyBrake);
	#endif
	
	#if key_HallSdy == 1
	button_init(&KeyStady,Read_Stady_GPIO,0);
	button_start(&KeyStady);
	#endif
	
	#if key_EStop == 1
	button_init(&KeyEStop,Read_EStop_GPIO,0);
	button_start(&KeyEStop);
	#endif
	
	#if Key_SW1 == 1
	button_init(&KeySW1,Read_SW1_GPIO,0);
	button_start(&KeySW1);
	#endif
	
	#if Key_SW2 == 1
	button_init(&KeySW2,Read_SW2_GPIO,0);
	button_start(&KeySW2);
	#endif
	
	#if Key_SW3 == 1
	button_init(&KeySW3,Read_SW3_GPIO,0);
	button_start(&KeySW3);
	#endif
	
	
}

void Key_Scan(void)
{
	#if key_run == 1
	static unsigned char run=0;
	#endif
	
	#if key_dir == 1
	static unsigned char dir=0;
	#endif
	
	#if key_brake == 1
	static unsigned char brake=0;
	#endif
	
	#if key_HallSdy == 1
	static unsigned char stady=0;
	#endif
	
	#if key_EStop == 1
	static unsigned char estop=0;
	#endif
	
	
	#if Key_SW1 == 1
	static unsigned char sw1=0;
	#endif
	
	#if Key_SW2 == 1
	static unsigned char sw2=0;
	#endif
	
	#if Key_SW3 == 1
	static unsigned char sw3=0;
	#endif
	
		button_ticks();


	#if key_run == 1
		if(run!= get_button_event(&KeyRun)){
			run = get_button_event(&KeyRun);
			if(run == PRESS_DOWN || run == LONG_PRESS_HOLD || run == LONG_PRESS_START){//

			}
			else{

			}
		}
	#endif
	
	#if key_dir == 1
		if(dir!= get_button_event(&KeyDir)){
			dir = get_button_event(&KeyDir);
			if(dir==PRESS_DOWN||dir==LONG_PRESS_HOLD||dir==LONG_PRESS_START){

			}else{

			}
		}
	#endif
		
	#if key_brake == 1
		if(brake!= get_button_event(&KeyBrake)){
			brake = get_button_event(&KeyBrake);
			if(brake==PRESS_DOWN||brake==LONG_PRESS_HOLD||brake==LONG_PRESS_START){

			}else{

			}
		}
	#endif
		
	#if key_HallSdy == 1
		if(stady != get_button_event(&KeyStady)){
			stady = get_button_event(&KeyStady);
			if(stady == PRESS_DOWN || stady == LONG_PRESS_HOLD || stady == LONG_PRESS_START){

			}else{

			}
		}
	#endif
		
	#if key_EStop == 1
		if(estop != get_button_event(&KeyEStop)){
			estop = get_button_event(&KeyEStop);
			if(estop == PRESS_DOWN || estop == LONG_PRESS_HOLD || estop == LONG_PRESS_START){

			}else{

			}
		}
	#endif
		
	
	#if Key_SW1 == 1
		if(sw1 != get_button_event(&KeySW1)){
			sw1 = get_button_event(&KeySW1);
			if(sw1 == PRESS_DOWN || sw1 == LONG_PRESS_HOLD || sw1 == LONG_PRESS_START){

			}else{

			}
		}
	#endif
	
	#if Key_SW2 == 1
		if(sw2 != get_button_event(&KeySW2)){
			sw2 = get_button_event(&KeySW2);
			if(sw2 == PRESS_DOWN || sw2 == LONG_PRESS_HOLD || sw2 == LONG_PRESS_START){
	
			}else{
			
			}
		}
	#endif
	
	#if Key_SW3 == 1
		if(sw3 != get_button_event(&KeySW3)){
			sw3 = get_button_event(&KeySW3);
			if(sw3 == PRESS_DOWN || sw3 == LONG_PRESS_HOLD || sw3 == LONG_PRESS_START){
			
			}else{
				
			}
		}
	#endif
	
}
