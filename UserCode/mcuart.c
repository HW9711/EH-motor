#include <math.h>
#include "mcctl.h"
#include "mcuart.h"
#include "mcflash.h"
#include "adc.h"
#include "tim.h"
#include "at32_board.h"
#include "fangbo.h"
#include "youshua.h"
#include "YSmcctl.h"
#include "YSstatemachine.h"
#define IDNE_TIM    20

extern uint16_t SPStatus[SpeNum];
extern void TIM_Motor_PWM(TMR_Type* TMRx);
volatile uint8_t YSorWSFlag = 1;		//有刷还是无刷 1-无刷 2有刷
	
//所有串口数据处理
void UartDealResponse(void){
	#if _Uart1 == 1
	Modbus_Ctl(USART1,&SerUart1);
	#endif
	
	#if _Uart2 == 1
	Modbus_Ctl(USART2,&SerUart2);
	#endif
	
	#if _Uart3 == 1
	Modbus_Ctl(USART3,&SerUart3);
	#endif
	
	#if _Uart4 == 1
	Modbus_Ctl(UART4,&SerUart4);
	#endif
	
	#if _Uart5 == 1
	Modbus_Ctl(UART5,&SerUart5);
	#endif
	
	#if _Uart6 == 1
	Modbus_Ctl(USART6,&SerUart6);
	#endif
	
	}

	
//串口Modbus协议解析
void Modbus_Ctl(USART_Type* USARTx,MCUART_Type *seruart){
	uint16_t i = 0;
	uint8_t temp = 0;
	float temp1 = 0;
	if(seruart->RxEnd == 1){		
		seruart->RxEnd = 0;
		if(seruart->RxLen < 2 || seruart->RxLen > MAX_RXBUF){
			return;
		}
		seruart->Addr = seruart->R_DATA[0];
		seruart->FunCode = seruart->R_DATA[1];
		seruart->RxCRCH = seruart->R_DATA[seruart->RxLen - 1] << 8;
		seruart->RxCRCL = seruart->R_DATA[seruart->RxLen - 2];
		seruart->RxCRC = seruart->RxCRCH + seruart->RxCRCL;		
		seruart->CalcCRC = CRC_Calc(seruart->R_DATA,seruart->RxLen - 2);	
		//从模式（上位机发送指令，当前板子接收并处理）
		if(
			(seruart->Addr == MCPara[0] || seruart->Addr == 0xEE) 
			&& (seruart->CalcCRC == seruart->RxCRC || seruart->RxCRC == 0xAABB)
			&& seruart->RxLen <= 8
		){
			switch(seruart->FunCode){
				case 3://查询
					seruart->Re3num = seruart->R_DATA[5];		//查询个数
					seruart->ReAddrH = seruart->R_DATA[2];	//查询内容类型
					seruart->ReAddrL = seruart->R_DATA[3]; 	//查询起始地址

					if(seruart->ReAddrH == 0x10){//查询 MCPara[]			//查询0x10xx，无刷1参数
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= ParaNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数

							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCPara[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCPara[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}

							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);

							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}else if(seruart->ReAddrH == 0x11){//查询 MCPara2[]			//查询0x11xx，无刷2参数
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= ParaNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数

							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCPara2[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCPara2[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}

							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);

							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

					else if(seruart->ReAddrH == 0x20){//查询 MCStatus[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= StatusNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{

							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数
							Modbus_20_Return();
							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCStatus[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCStatus[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}
							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

					else if(seruart->ReAddrH == 0x60 || seruart->ReAddrH == 0x30){//查询 SPStatus[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= SpeNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数
							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = SPStatus[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = SPStatus[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}
							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

				break;
				case 6://更改				
					seruart->UpdateRegH = seruart->R_DATA[2];
					seruart->UpdateRegL = seruart->R_DATA[3];
					seruart->UpdatePara = (seruart->R_DATA[4] << 8) + (seruart->R_DATA[5]);
					//更改参数
					if(
						seruart->UpdateRegH == 0x10				//更改0x10xx，无刷1参数
					){
						if(seruart->UpdateRegL >= ParaNum){
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else {
							if(seruart->UpdatePara > 32767){
								ModbusErrorResponse(USARTx,seruart,0x83,3);
							}else {
								MCPara[seruart->UpdateRegL] = seruart->UpdatePara;
								ModbusResponse(USARTx,seruart);
								Pos_Uart_Init();	//更新一下最新刀头减速比数据
							}
						}
						
					}else	if(
						seruart->UpdateRegH == 0x11				//更改0x11xx，无刷2参数
					){
						if(seruart->UpdateRegL >= ParaNum){
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else {
							if(seruart->UpdatePara > 32767){
								ModbusErrorResponse(USARTx,seruart,0x83,3);
							}else {
								MCPara2[seruart->UpdateRegL] = seruart->UpdatePara;
								ModbusResponse(USARTx,seruart);
								Pos_Uart_Init();	//更新一下最新刀头减速比数据

							}
							
						}
					}
					//特殊指令
					else if(
						seruart->UpdateRegH == 0x60
						|| seruart->UpdateRegH == 0x30
					){    
						switch(seruart->UpdateRegL){
							case 0x00://启停

								ModbusResponse(USARTx,seruart);
							break;
							case 0x01://方向
								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x02://清除错误
								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x03://保存
								if((App.FB.AllRun== 0)&&(App.FB2.AllRun== 0)){
									if(seruart->UpdatePara == 1){
										FLASH_Write(TEST_FLASH_ADDRESS_START,MCPara,ParaNum);
									}else if(seruart->UpdatePara == 3){
										F4Flash_SMCPara_Erase(TEST_FLASH_ADDRESS_START);
									}
									ModbusResponse(USARTx,seruart);
								}else {
									ModbusErrorResponse(USARTx,seruart,0x83,5);
								}
								
							break;
								
							case 0x04://退出指令控制

								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x0A:
								
								ModbusResponse(USARTx,seruart);
							break;
							
							
							default:
								ModbusErrorResponse(USARTx,seruart,0x83,4);
							break;	
						}
					} 
				break;
				default:
					ModbusErrorResponse(USARTx,seruart,0x83,1);
				break;			 
			}
		}
		if(
			(seruart->Addr == MCPara2[0] || seruart->Addr == 0xEE) 
			&& (seruart->CalcCRC == seruart->RxCRC || seruart->RxCRC == 0xAABB)
			&& seruart->RxLen <= 8
		){
			switch(seruart->FunCode){
				case 3://查询
					seruart->Re3num = seruart->R_DATA[5];		//查询个数
					seruart->ReAddrH = seruart->R_DATA[2];	//查询内容类型
					seruart->ReAddrL = seruart->R_DATA[3]; 	//查询起始地址

					if(seruart->ReAddrH == 0x10){//查询 MCPara[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= ParaNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数

							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCPara2[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCPara2[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}

							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);

							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

					else if(seruart->ReAddrH == 0x20){//查询 MCStatus[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= StatusNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{

							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数
							Modbus_20_Return();
							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = MCStatus[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = MCStatus[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}
							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

					else if(seruart->ReAddrH == 0x60 || seruart->ReAddrH == 0x30){//查询 SPStatus[]
						if( 
							(seruart->Re3num + seruart->ReAddrL) >= SpeNum//查询数量不能超过数组上限
							|| seruart->Re3num > 30//查询数量不能超过30个
						){
							//错误 查询的地址过多 或 过大 导致溢出错误
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else{
							seruart->T_DATA[0] = seruart->Addr;
							seruart->T_DATA[1] = 0x03;							
							seruart->T_DATA[2] = seruart->Re3num << 1;//数量是需要返回的数据2倍				
							seruart->SendLen = 3;							     		//需要发送的字节数
							for(i = 0; i < seruart->Re3num; i ++){
								seruart->SendVa = i << 1;      					//*2,位置起始
								seruart->T_DATA[seruart->SendVa + 3] = SPStatus[seruart->ReAddrL + i] >> 8;
								seruart->T_DATA[seruart->SendVa + 4] = SPStatus[seruart->ReAddrL + i] & 0x00FF;
								seruart->SendLen += 2;
							}
							seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
							seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
							seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;  
							seruart->TxLen = seruart->SendLen + 2;
							Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
						}
					}

				break;
				case 6://更改				
					seruart->UpdateRegH = seruart->R_DATA[2];
					seruart->UpdateRegL = seruart->R_DATA[3];
					seruart->UpdatePara = (seruart->R_DATA[4] << 8) + (seruart->R_DATA[5]);
					//更改参数
					if(
						seruart->UpdateRegH == 0x10
					){
						if(seruart->UpdateRegL >= ParaNum){
							ModbusErrorResponse(USARTx,seruart,0x83,2);
						}else {
							if(seruart->UpdatePara > 32767){
								ModbusErrorResponse(USARTx,seruart,0x83,3);
							}else {
								MCPara2[seruart->UpdateRegL] = seruart->UpdatePara;
								ModbusResponse(USARTx,seruart);
							}
						}
					}
					//特殊指令
					else if(
						seruart->UpdateRegH == 0x60
						|| seruart->UpdateRegH == 0x30
					){    
						switch(seruart->UpdateRegL){
							case 0x00://启停

								ModbusResponse(USARTx,seruart);
							break;
							case 0x01://方向
								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x02://清除错误
								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x03://保存
								if((App.FB.AllRun== 0)&&(App.FB2.AllRun== 0)){
									if(seruart->UpdatePara == 1){
										FLASH_Write(TEST_FLASH_ADDRESS_START2,MCPara2,ParaNum);
									}else if(seruart->UpdatePara == 3){
										F4Flash_SMCPara_Erase(TEST_FLASH_ADDRESS_START2);
									}
									ModbusResponse(USARTx,seruart);
								}else {
									ModbusErrorResponse(USARTx,seruart,0x83,5);
								}
								
							break;
								
							case 0x04://退出指令控制

								ModbusResponse(USARTx,seruart);
							break;
							
							case 0x0A:
								
								ModbusResponse(USARTx,seruart);
							break;
							
							
							default:
								ModbusErrorResponse(USARTx,seruart,0x83,4);
							break;	
						}
					} 
				break;
				default:
					ModbusErrorResponse(USARTx,seruart,0x83,1);
				break;			 
			}
		}		
		
		else if(	//私有协议：判断有无刷、有无Hall、刹车类型、保护电流
			(seruart->Addr == 0xAA)//地址校验 
			&& (seruart->CalcCRC == seruart->RxCRC || seruart->RxCRC == 0xAABB)
			&& seruart->RxLen == 11
		){

			if(App.FB.Err == E_HandShake || App.FB2.Err == E_HandShake || App2.Err == E_HandShake){
				App2.Err = E_NONE;		//错误清除
				App.FB.Err = E_NONE;	//清除错误
				App.FB2.Err =E_NONE;	//清除错误
			}
			App.Logic.HandShakeProtCnt = 0;			//收到一帧正确数据，清除通讯握手计时
			App.Logic.Set_Spd =(uint32_t)(((seruart->R_DATA[4]<<8)+seruart->R_DATA[5])*10);		//计算转速
			if(App.Logic.Set_Spd == 0){		//停机判断
				App2.Err = E_NONE;								//错误清除
				App.FB.Err = E_NONE;							//清除错误
				App.FB2.Err = E_NONE;							//清楚错误							
			}
			if(YSorWSFlag == 1){						//无刷
				if(seruart->R_DATA[3] != 1 && seruart->R_DATA[3] != 2){
					App.Logic.Set_Spd = 0;
					if(App.FB.Status == HS_WAIT && App.FB2.Status == HS_WAIT &&(seruart->R_DATA[3] == 3|| seruart->R_DATA[3] == 4)){
						YSorWSFlag = 2;
						App.Logic.Set_Spd =(uint32_t)(((seruart->R_DATA[4]<<8)+seruart->R_DATA[5])*10);		//设置转速
					}
				}
			}else if (YSorWSFlag == 2){			//有刷
				if(seruart->R_DATA[3] != 3 && seruart->R_DATA[3] != 4)
				{
					App.Logic.Set_Spd = 0;
					if(App2.Ch1.Status == CTLS_WAIT && App2.Ch2.Status == CTLS_WAIT  &&(seruart->R_DATA[3] == 1|| seruart->R_DATA[3] == 2)  ){
						YSorWSFlag = 1;
						App.Logic.Set_Spd =(uint32_t)(((seruart->R_DATA[4]<<8)+seruart->R_DATA[5])*10);		//设置转速
					}
				}
			}
			//设置保护电流
			seruart->tempcurrent = (float)(((seruart->R_DATA[7]<<8) +seruart->R_DATA[8])/100.0f);

				if(YSorWSFlag == 1){	//对无刷进行操作
				if(App.Logic.Set_Spd >=MCPara[13]*10){
					App.Logic.Set_Spd = MCPara[13]*10;
				}
				App.Logic.u8WfFru = seruart->R_DATA[2]*2;		//更改速率

				//首先判断电机是否在运行---不在运行就赋值启动，在运行就判断是否要刹车
				if((App.FB.AllRun == 0)&&(App.FB2.AllRun == 0)){		//两个电机都不在运动
						App.Logic.CtlMode 	 = seruart->R_DATA[1];			//赋值电机模式
						App.Logic.u8WfFru 	 = seruart->R_DATA[2]*2;			//赋值往返频率
						App.Logic.Motor_Kind = seruart->R_DATA[3];			//赋值电机类型
						App.Logic.LogicHall_or_pll = seruart->R_DATA[6];//赋值霍尔类型
						App.Logic.Brake_Sign = 0;
						App.Logic.Brake_Kind = 0;
					
						if(App.Logic.Set_Spd > MCPara[13]*100){
								App.Logic.Set_Spd = MCPara[13]*100;			
							}

				}else{	//有其中一个电机正在运动	
					if(App.Logic.Set_Spd != 0){			//判定为正在运行刹车
						if(App.Logic.Motor_Kind == seruart->R_DATA[3]){		//判定电机类型不变
							if(App.Logic.CtlMode == seruart->R_DATA[1]){
								if(App.Logic.CtlMode !=3){										
									if(App.Logic.LogicHall_or_pll != seruart->R_DATA[6]){
										App.Logic.Change_Kind = seruart->R_DATA[3];		//刹车完更改电机类型
										App.Logic.Change_Mode= seruart->R_DATA[1];	//刹车完变换的模式类型
										App.Logic.Change_Hall =seruart->R_DATA[6];	//刹车完变换的Hall类型
										App.Logic.Brake_Sign  = 1;			//去刹车一次
										App.Logic.Brake_Kind  = 4;			//刹车种类为：切换Hall刹车
									}
								}else{
									if(App.Logic.Set_Spd > MCPara[8]*100){
										App.Logic.Set_Spd = MCPara[8]*100;				//模式3限制最高转速
									}
									App.Pos.PosFlag = 1;		//产生一次位置刹车
								}
							}else{
								App.Logic.Change_Kind = seruart->R_DATA[3];		//刹车完更改电机类型
								App.Logic.Change_Mode = seruart->R_DATA[1];	//刹车完变换的模式类型
								App.Logic.Change_Hall = seruart->R_DATA[6];	//刹车完变换的Hall类型
								App.Logic.CtlMode =	App.Logic.Change_Mode;	//提前切换模式
								App.Logic.Brake_Sign = 1;			//去刹车一次
								App.Logic.Brake_Kind = 3;			//刹车种类为：切换电机模式刹车
							}
						}else{																						//判定电机类型改变
							App.Logic.Change_Kind = seruart->R_DATA[3];		//刹车完更改电机类型
							App.Logic.Change_Mode= seruart->R_DATA[1];	//刹车完变换的模式类型
							App.Logic.Change_Hall =seruart->R_DATA[6];	//刹车完变换的Hall类型
							App.Logic.Brake_Sign  = 1;			//去刹车一次		
							App.Logic.Brake_Kind  = 2;			//刹车种类为：切换电机种类刹车				
						}
					}else{													//判断为停机指令
						App.Logic.Change_Kind = App.Logic.Motor_Kind;
						App.Logic.Change_Mode = 0;
						App.Logic.Change_Hall = App.Logic.LogicHall_or_pll;
						if(App.Logic.CtlMode !=3){
							App.Logic.Motor_Stop_StartFlag = 1;
						}
						App.Logic.Brake_Sign = 1;			//去刹车一次								
						App.Logic.Brake_Kind = 1;			//刹车种类为：停机刹车
						App.FB.Err = E_NONE;							//清除错误
						App.FB2.Err = E_NONE;							//清楚错误
						//刹车指令就需要用位置刹车
						App.Pos.PosFlag = 1;		//产生一次位置刹车
						if(App.Logic.LogicHall_or_pll == 1){
							App.Pos.PosFlag = 0;
						}		
					}
				}

				if(App.Logic.CtlMode == 4 || App.Logic.CtlMode ==5 ){
					App.Logic.CtlMode 	 = seruart->R_DATA[1];			//赋值电机模式
					if(App.Logic.CtlMode != 4 && App.Logic.CtlMode != 5){
						PWM_3l_Stop();
						PWM_3l_Stop2();
					} 
					if(App.Pos.LockPosStatus == 0){		//如果是等待拖动状态就可以拖动，并且锁定拖动的步数
						App.Pos.LockPosStatus = 1;	//准备去拖动
						App.Pos.PosSetSteps = (uint32_t)((seruart->R_DATA[4]<<8)+seruart->R_DATA[5]);
						if(App.Pos.PosSetSteps  > 65535 ){
							App.Pos.PosSetSteps = 65535;
						}
						if(App.Pos.PosSetSteps > 0){
							App.Pos.Start = 1;  					//允许拖动启动
						}
					}else{
					
					}
				}
				if(seruart->tempcurrent == 0){
					//这里可以加入选择电流
				}else{
						App.FB.Prot.OverCurrent = seruart->tempcurrent;
						App.FB2.Prot.OverCurrent = seruart->tempcurrent;
				}
			//无刷返回一帧数据
			Customize_Return(USARTx,seruart);
			}
			else if(YSorWSFlag == 2){		//对有刷的操作
				//控制模式 01-正转 02-反转 03-往复模式
				App2.Log.CtlMode = seruart->R_DATA[1];		
				if(App2.Log.CtlMode > 3){
					App2.Log.CtlMode = 0;
				}
				//往复模式下频率
				temp = seruart->R_DATA[2];
				App2.Log.u8WfFru = temp*2 ;
				if(App2.Log.u8WfFru > 200){
					App2.Log.u8WfFru = 200;
				}else if(App2.Log.u8WfFru < 1){
					App2.Log.u8WfFru = 1;
				}
				App2.Log.Channel = seruart->R_DATA[3];			//通道选择
				if((App2.Log.Channel != 3)&&(App2.Log.Channel !=4)){			//防止主机发送过来错误的通道
					App2.Log.Channel = 3;
				}
				if(seruart->R_DATA[6] == 3){
					App2.Log.Drill = 1;
				}else if(seruart->R_DATA[6] == 4){
					App2.Log.Drill = 2;
				}else{
					App2.Log.Drill = 1;
				}
				//设定转速
//				App2.Log.u32SetSpd_temp = ((seruart->R_DATA[4] << 8) + seruart->R_DATA[5]);
				App2.Log.u32SetSpd = App.Logic.Set_Spd ;
				if(App2.Log.u32SetSpd > YSPara1[1]*10){
					App2.Log.u32SetSpd = YSPara1[1]*10;
				}else if(App2.Log.u32SetSpd == 0){
					App2.Err = E_NONE;						//清除错误u
					App2.Log.BreakSta = 0;			//清除刹车
				}
				temp1 = ((float)((seruart->R_DATA[7]<<8)+seruart->R_DATA[8])/100.0f);
				if(App2.Log.Channel == 3){
					if(temp1 !=0){
						App2.Ch1.Prot.OverCurrent = temp1;		//通道1过流保护
					}
				}else if(App2.Log.Channel == 4){
					if(temp1 !=0){
						App2.Ch2.Prot.OverCurrent =temp1;		//通道2过流保护
					}
				}else{
					//根据系统而定
				}
				//有刷返回一帧指令
				YSCustomize_Return(USARTx,seruart);
			}else{
				//没有选择电机
  		}
		}

	}
}


//CRC校验函数
uint16_t CRC_Calc(unsigned char *snd, unsigned char num)
{
    unsigned char i, j;
    unsigned int c,crc=0xFFFF;
    for(i = 0; i < num; i ++)
    {
        c = snd[i] & 0x00FF;
        crc ^= c;
        for(j = 0;j < 8; j ++)
        { 
            if (crc & 0x0001)
            {
                crc>>=1;
                crc^=0xA001;
            }
            else crc>>=1;
        }
    }    
    return(crc);
}



//特殊指令返回
void Customize_Return(USART_Type* USARTx,MCUART_Type* seruart){
	uint32_t temp = 0;
//	uint32_t temp1 = 0;
//	uint32_t temp2 = 0;
	uint32_t temp3 = 0;
//	temp1 = (uint32_t)(App.FB.Prot.OverCurrent*100);
//	temp2 = (uint32_t)(App.FB2.Prot.OverCurrent*100);
	temp3 = (uint32_t)(App.FB.Prot.AllCur	*100);
	seruart->T_DATA[0] = 0xAA;									//帧头
	seruart->T_DATA[1] = App.Logic.CtlMode;			//控制模式
	seruart->T_DATA[2] = App.Logic.u8WfFru;			//往复频率
	seruart->T_DATA[3] = App.Logic.Motor_Kind;	//电机选择
	if(App.Logic.Motor_Kind == 1){
		seruart->T_DATA[7] = App.FB.Err;
		if(App.FB.Spd_Now >= 0){
			temp = App.FB.Spd_Now/10;							//倍率/10
		}else{
			temp = -App.FB.Spd_Now/10;						//倍率/10
		}
	}else if(App.Logic.Motor_Kind == 2){ 
		seruart->T_DATA[7] = App.FB2.Err;
		if(App.FB2.Spd_Now >= 0){
			temp = App.FB2.Spd_Now/10;					//倍率/10
		}else{
			temp = -App.FB2.Spd_Now/10;					//倍率/10
		}
	}
	seruart->T_DATA[4] = (temp>>8);
	seruart->T_DATA[5] = temp & 0x00FF;
	seruart->T_DATA[6] = App.Logic.LogicHall_or_pll;
//	if(App.Logic.Motor_Kind == 1){
//		seruart->T_DATA[8] =	(uint16_t) ((temp1&0x0000FF00)>>8);	
//		seruart->T_DATA[9] =	(uint16_t) ((temp1)&0x000000FF);
//	}else if(App.Logic.Motor_Kind ==2){
//		seruart->T_DATA[8] =	(uint16_t) ((temp2&0x0000FF00)>>8);
//		seruart->T_DATA[9] =	(uint16_t) ((temp2)&0x000000FF);		
//	}
	seruart->T_DATA[8] =	(uint16_t) ((temp3&0x0000FF00)>>8);
  seruart->T_DATA[9] =	(uint16_t) ((temp3)&0x000000FF);	

	seruart->SendLen = 10;
	seruart->TxCRC = CRC_Calc(seruart->T_DATA,10);
	seruart->T_DATA[10] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
	seruart->T_DATA[11] = seruart->TxCRC >> 8;     

	Modbus_Send(USARTx,seruart->T_DATA,12);
}


//有刷特殊指令返回
void YSCustomize_Return(USART_Type* USARTx,MCUART_Type* seruart){
	uint32_t temp = 0;
	uint32_t temp1 = 0;
	uint32_t temp2 = 0;
	temp1 = (uint32_t)(App2.Ch1.Prot.OverCurrent*100);
	temp2 = (uint32_t)(App2.Ch2.Prot.OverCurrent*100);
	
	seruart->T_DATA[0] = 0xAA;									//帧头
	seruart->T_DATA[1] = App2.Log.CtlMode;			//控制模式
	seruart->T_DATA[2] = App2.Log.u8WfFru;			//往复频率
	seruart->T_DATA[3] = App2.Log.Channel;	//电机选择
	seruart->T_DATA[7] = App2.Err;
	
	temp = App2.SysCtl.SpdNowLPF/10;					//倍率/10
	
	seruart->T_DATA[4] = (temp>>8);
	seruart->T_DATA[5] = temp & 0x00FF;
	seruart->T_DATA[6] = App2.Log.Drill;
	if(App2.Log.Channel == 3){
		seruart->T_DATA[8] =	(uint16_t) ((temp1&0x0000FF00)>>8);	
		seruart->T_DATA[9] =	(uint16_t) ((temp1)&0x000000FF);
	}else if(App2.Log.Channel == 4){
		seruart->T_DATA[8] =	(uint16_t) ((temp2&0x0000FF00)>>8);
		seruart->T_DATA[9] =	(uint16_t) ((temp2)&0x000000FF);		
	}

	seruart->SendLen = 10;
	seruart->TxCRC = CRC_Calc(seruart->T_DATA,10);
	seruart->T_DATA[10] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
	seruart->T_DATA[11] = seruart->TxCRC >> 8;     

	Modbus_Send(USARTx,seruart->T_DATA,12);
}

//modbus 指令错误返回
void ModbusErrorResponse(USART_Type* USARTx,MCUART_Type* seruart,uint8_t type,uint8_t err){
	seruart->T_DATA[0] = seruart->Addr;
	seruart->T_DATA[1] = type;
	seruart->T_DATA[2] = err;

	seruart->SendLen = 3;
	seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
	seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
	seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;     
	seruart->TxLen = seruart->SendLen + 2;
	Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
	
}

void Modbus_20_Return(void){
	MCStatus[0] = App.FB.AllRun;//0，运行状态
	MCStatus[1] = myabs(App.FB.Spd_Now);//1，
	MCStatus[2] = App.FB.Prot.BusVol * 10.0f;//2，
	MCStatus[3] = App.FB.Prot.AllCur * 10.f;//3，
	MCStatus[4] = App.FB.Err;//4，
	MCStatus[5] = 0;//5，
	MCStatus[6] = App.FB.Dir.Now;//6，
	MCStatus[7] = 0;//7，
	MCStatus[8] = 0;//8，
	MCStatus[9] = 0;//9,
	
}



//modbus指令成功返回
void ModbusResponse(USART_Type* USARTx,MCUART_Type* seruart){
	seruart->T_DATA[0] = seruart->Addr;
	seruart->T_DATA[1] = seruart->FunCode;
	seruart->T_DATA[2] = seruart->R_DATA[2];
	seruart->T_DATA[3] = seruart->R_DATA[3];
	seruart->T_DATA[4] = seruart->R_DATA[4];
	seruart->T_DATA[5] = seruart->R_DATA[5];

	seruart->SendLen = 6;
	seruart->TxCRC = CRC_Calc(seruart->T_DATA,seruart->SendLen);
	seruart->T_DATA[seruart->SendLen] = seruart->TxCRC & 0x00FF;  //CRC 先低位在开位
	seruart->T_DATA[seruart->SendLen + 1] = seruart->TxCRC >> 8;     
	seruart->TxLen = seruart->SendLen + 2;
	
	Modbus_Send(USARTx,seruart->T_DATA,seruart->TxLen);
}


