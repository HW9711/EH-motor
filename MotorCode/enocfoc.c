#include "enocfoc.h"
#include "mcctl.h"


uint16_t return_enoc_val(TMR_Type* TMRx){
	return (TMRx -> CNT);
}

void set_enoc_val(TMR_Type* TMRx,uint16_t val){
	(TMRx -> CNT) = val;
}






void Enoc_Z_Offset(void){
	if(App.M1.Enoc.Sta == 1 && App.M1.Hpll.HallStadySta == 3){//已经设置过编码器的值了 并且已经学习过hall了。
		App.M1.Enoc.Enoc_I_Cnt ++;
		if(App.M1.Enoc.Enoc_I_Cnt >= 5){
			App.M1.Enoc.Enoc_I_Cnt = 5;
			
			App.M1.Enoc.Enoc_I = read_enoc + App.M1.Enoc.EZOffset;
			
			while(App.M1.Enoc.Enoc_I < 0){
				App.M1.Enoc.Enoc_I += MCPara[83];
			}
			
			while(App.M1.Enoc.Enoc_I >= MCPara[83]){
				App.M1.Enoc.Enoc_I -= MCPara[83];
			}
			
			App.M1.Enoc.Enoc_I_PJ = myabs((App.M1.Enoc.Enoc_Mid - App.M1.Enoc.Enoc_I));
			
			if(App.M1.Enoc.Enoc_I_PJ < MCPara[84]){//误差很大，需要处理
				//read_enoc = 2048 - App.M1.Enoc.EZOffset;
				
				if(read_enoc < App.M1.Enoc.EZSetEnoc){
					set_enoc += 1;
				}else if(read_enoc > App.M1.Enoc.EZSetEnoc){
					set_enoc -= 1;
				}
			}else {//误差特别大，报错
				App.M1.Err = E_ENOC;
				App.BitErr |= ERR_GD_ENOC_Loss;//供弹电机编码器错误
			}
			
			
		}else if(App.M1.Enoc.Enoc_I_Cnt >= 3){//从第三个编码器I信号开始开始
			
			App.M1.Enoc.EZOffset = App.M1.Enoc.Enoc_Mid - read_enoc;
			App.M1.Enoc.EZSetEnoc = App.M1.Enoc.Enoc_Mid - App.M1.Enoc.EZOffset;
			if(App.M1.Enoc.EZOffset > App.M1.Enoc.Enoc_Mid || App.M1.Enoc.EZOffset < -App.M1.Enoc.Enoc_Mid){
				App.M1.Err = E_ENOC;//编码器错误
				App.BitErr |= ERR_GD_ENOC_Loss;//供弹电机编码器错误
				
			}
			
		}else {//前面上个编码器I轴信号滤除
			
		}
		
	}
}



void Enoc_Z2_Offset(void){
	if(App.M2.Enoc.Sta == 1 && App.M2.Hpll.HallStadySta == 3){//已经设置过编码器的值了 并且已经学习过hall了。
		App.M2.Enoc.Enoc_I_Cnt ++;
		if(App.M2.Enoc.Enoc_I_Cnt >= 5){
			App.M2.Enoc.Enoc_I_Cnt = 5;
			
			App.M2.Enoc.Enoc_I = read_enoc2 + App.M2.Enoc.EZOffset;
			
			while(App.M2.Enoc.Enoc_I < 0){
				App.M2.Enoc.Enoc_I += MCPara2[83];
			}
			
			while(App.M2.Enoc.Enoc_I >= MCPara2[83]){
				App.M2.Enoc.Enoc_I -= MCPara2[83];
			}
			
			App.M2.Enoc.Enoc_I_PJ = myabs((App.M2.Enoc.Enoc_Mid - App.M2.Enoc.Enoc_I));
			
			if(App.M2.Enoc.Enoc_I_PJ < MCPara2[84]){//误差很大，需要处理
				//read_enoc = 2048 - App.M2.Enoc.EZOffset;
				
				if(read_enoc2 < (App.M2.Enoc.EZSetEnoc)){
					set_enoc2 += 1;
				}else if(read_enoc2 > (App.M2.Enoc.EZSetEnoc)){
					set_enoc2 -= 1;
				}
			}else {//误差特别大，报错
				App.M2.Err = E_ENOC;
				App.BitErr |= ERR_WL_ENOC_Loss;//威力电机编码器错误
			}
			
			
		}else if(App.M2.Enoc.Enoc_I_Cnt >= 3){//从第三个编码器I信号开始开始
			
			App.M2.Enoc.EZOffset = App.M2.Enoc.Enoc_Mid - read_enoc2;
			App.M2.Enoc.EZSetEnoc = App.M2.Enoc.Enoc_Mid - App.M2.Enoc.EZOffset;
			if(App.M2.Enoc.EZOffset > App.M2.Enoc.Enoc_Mid || App.M2.Enoc.EZOffset < -App.M2.Enoc.Enoc_Mid){
				App.M2.Err = E_ENOC;
				App.BitErr |= ERR_WL_ENOC_Loss;//威力电机编码器错误
			}
			
		}else {//前面上个编码器I轴信号滤除
			
		}
		
	}
}

























