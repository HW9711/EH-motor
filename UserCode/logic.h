#ifndef __LOGIC_H
#define __LOGIC_H
#include  "userpara.h"
#include "at32f4xx.h"

#define STOP_STATUS		0
#define CW_STATUS		1
#define CCW_STATUS		2
#define CW_CCW_STATUS	3



void Ctl_Logic(void);
extern unsigned short MCPara_485Mode[10];
extern unsigned short SMCPara_485Mode[10];

void SerUart_01(int16_t updata);
void SerUart_02(int16_t updata);
void SerUart_03(int16_t updata);
void SerUart_04(int16_t updata);
void SerUart_05(int16_t updata);
void Logic_Sotf_Init(void);

void Logic_500ms(void);
void Logic_1ms(void);
void Pos_1ms(void);
void Err_Clear(uint16_t clear_errtim,uint16_t clear_cnt,uint16_t cnt_cnt);

void Logic_Fangbo(void);
void Logic_Fangbo2(void);
void Control_State_Scan(void);
extern void  STA_Led(uint8_t fault_num);
#endif

