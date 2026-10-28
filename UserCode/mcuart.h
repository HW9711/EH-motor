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

/* 函数功能：查询外部停止约束；输入参数：无；返回参数：非零禁止重新启动和普通调速。 */
uint8_t MotorCommand_IsStopRequested(void);
/* 函数功能：查询全部输出锁定；输入参数：无；返回参数：非零时换相和开桥入口也必须关闭。 */
uint8_t MotorCommand_IsOutputLocked(void);
/* 函数功能：监督外部停机期限；输入参数：无；返回参数：非零时定时任务跳过正常控制。 */
uint8_t MotorCommand_ServiceStop500us(void);
/* 函数功能：检查停机相关参数；输入参数：协议电机类型1~4；返回参数：参数有效为1。 */
uint8_t MotorCommand_AreStopParametersValid(uint8_t motor_kind);

#endif 
