#ifndef __CAN_CTL_H
#define __CAN_CTL_H
#include "mcctl.h"

void Can_Init(void);
void Can_Ctl(void);

typedef struct
{ 
  __IO uint16_t cob_id;
	__IO uint8_t len;
	__IO uint8_t rtr;
	__IO uint8_t Rxflag;
	__IO uint8_t state;
	uint8_t Rxdata[8];
	uint8_t Txdata[8];
}CANOPEN_TypeDef;
extern CANOPEN_TypeDef RxMSG;

void CanOpen_Rx(void);
uint8_t CAN1_Send_Msg(uint8_t* msg,uint8_t len,uint16_t cobid);
void CAN_Configuration(uint16_t setaddr);
#endif
