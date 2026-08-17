#ifndef _MCUART_H    
#define _MCUART_H
#include  "stdint.h"
#include  "userpara.h"
#include "usart.h"

#define MAX_RXBUF  70

typedef struct
{
	uint8_t    R_DATA[MAX_RXBUF];   
  uint8_t    T_DATA[MAX_RXBUF];
	uint8_t    RxEnd;
  uint16_t   RxLen;
	uint16_t   RxCRC;
	uint16_t   TxCRC;
  uint16_t   CalcCRC;
	uint8_t    Addr;
	uint8_t    FunCode;
	uint8_t    TxLen;
  uint16_t   UpdateRegH;
	uint16_t   UpdateRegL;
  int16_t    UpdatePara;
	uint16_t TempAll;
}MCUART_Type2;

extern MCUART_Type2 SerUart;
extern volatile uint8_t YSorWSFlag;		//有刷还是无刷 1-无刷 2有刷

uint16_t CRC_Calc(unsigned char *snd, unsigned char num);
extern void Modbus_Ctl(USART_Type* USARTx,MCUART_Type *seruart);
void Customize_Return(USART_Type* USARTx,MCUART_Type* seruart);
void ModbusErrorResponse(USART_Type* USARTx,MCUART_Type* seruart,uint8_t type,uint8_t err);
extern void UartDealResponse(void);
void Modbus_20_Return(void);
void ModbusResponse(USART_Type* USARTx,MCUART_Type* seruart);
void YSCustomize_Return(USART_Type* USARTx,MCUART_Type* seruart);

/*
 * 函数功能：控制租约超时后立即关闭无刷和有刷全部功率输出，并要求零速帧重新武装。
 * 输入参数：无。
 * 返回参数：无。
 */
void MotorCommand_ForceCommunicationStop(void);

#endif 
