#ifndef __DRIVER_PARAM_H
#define __DRIVER_PARAM_H

#include "mcuart.h"

/*
 * 函数功能：从独立Flash页装载两路有刷运行参数，空白页自动回落到编译默认值。
 * 输入参数：无。
 * 返回参数：无。
 */
void DriverParam_Init(void);

/*
 * 函数功能：识别并处理固定地址0xFD的内部参数维护Modbus帧。
 * 输入参数：USARTx为当前串口；seruart为已经完成长度、地址和CRC字段拆解的串口状态。
 * 返回参数：本帧属于0xFD维护协议返回1；其它原有协议返回0并继续旧分支。
 */
uint8_t DriverParam_TryHandle(USART_Type *USARTx, MCUART_Type *seruart);

#endif
