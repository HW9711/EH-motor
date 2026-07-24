#include "canctl.h"
#include "mcctl.h"
#include <stdio.h>
#include "at32f4xx.h"
#include "at32_board.h"
#include "mcflash.h"

CANOPEN_TypeDef RxMSG={0};

/** The following one of the define AT32_USE_CAN1 must be choose*/
#define AT32_USE_CAN1      ///<CAN1 selected to be use

#ifdef AT32_USE_CAN1

	/** The following one of the define CAN1_NOREMAP and CAN1_REMAP10 and CAN1_REMAP11 must be choose*/
	#define CAN1_NOREMAP//CAN1_NOREMAP ///<CAN1_REMAP10 mean CAN1 use pins PB8 and PB9;CAN1_NOREMAP mean CAN1 use pins PA11 and PA12;CAN1_REMAP11 mean CAN1 use pins PD0 and PD1
	#define CANx        CAN1 
	#define CANx_CLK_ENABLE()              	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_CAN1, ENABLE);
	#ifdef CAN1_REMAP10
		#define CANx_TX    GPIO_Pins_9
		#define CANx_RX    GPIO_Pins_8
		#define CANx_TX_GPIO_CLK_ENABLE()       RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
		#define CANx_RX_GPIO_CLK_ENABLE()       RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
		#define CANx_TX_GPIO_PORT              GPIOB
		#define CANx_RX_GPIO_PORT              GPIOB
	#elif defined CAN1_REMAP11
		#define CANx_TX    GPIO_Pins_1
		#define CANx_RX    GPIO_Pins_0
		#define CANx_TX_GPIO_CLK_ENABLE()       RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOD, ENABLE);
		#define CANx_RX_GPIO_CLK_ENABLE()       RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOD, ENABLE);
		#define CANx_TX_GPIO_PORT              GPIOD
		#define CANx_RX_GPIO_PORT              GPIOD
	#else
		#define CANx_TX    GPIO_Pins_12
		#define CANx_RX    GPIO_Pins_11
		#define CANx_TX_GPIO_CLK_ENABLE()       RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
		#define CANx_RX_GPIO_CLK_ENABLE()     	 RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
		#define CANx_TX_GPIO_PORT              GPIOA
		#define CANx_RX_GPIO_PORT              GPIOA
	#endif
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitType GPIO_InitStructure;
RCC_ClockType RccClkSource = {0};
CAN_InitType  CAN_InitStructure;
CAN_FilterInitType  	CAN_FilterInitStructure = {0};
NVIC_InitType  NVIC_InitStructure = {0};
CanRxMsg RxMessage = {0};

/* Private function prototypes -----------------------------------------------*/
static void CAN_GPIO_Configuration(void);
static void CAN_NVIC_Config(void);


static void CAN_GPIO_Configuration(void){
	#ifdef AT32_USE_CAN1
		#ifdef CAN1_REMAP10
			RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);
			GPIO_PinsRemapConfig(GPIO_Remap1_CAN1,ENABLE);
		#elif defined CAN1_REMAP11
			RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);
			GPIO_PinsRemapConfig(GPIO_Remap2_CAN1,ENABLE);
		#endif
	#endif
	CANx_TX_GPIO_CLK_ENABLE(); 
	CANx_RX_GPIO_CLK_ENABLE(); 
	GPIO_InitStructure.GPIO_Pins = CANx_TX;
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_Init(CANx_TX_GPIO_PORT, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pins = CANx_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PU;	
	GPIO_Init(CANx_RX_GPIO_PORT, &GPIO_InitStructure);
	CANx_CLK_ENABLE(); 
}


void CAN_Configuration(uint16_t setaddr)
{    
	
	CAN_ClearINTPendingBit(CANx,CAN_INT_RFP0);
  CAN_INTConfig(CANx,CAN_INT_RFP0,DISABLE);
	
	CAN_InitStructure.CAN_TTC = DISABLE;			
	CAN_InitStructure.CAN_ABO = ENABLE;			
	CAN_InitStructure.CAN_AWU = ENABLE;			
	CAN_InitStructure.CAN_NART = DISABLE;			
	CAN_InitStructure.CAN_RFL = DISABLE;		 	
	CAN_InitStructure.CAN_TFP = DISABLE;			
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	
	
	if(MCPara[82] == 0){//125
		CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;				
		CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
		CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;	
		CAN_InitStructure.CAN_Prescaler = 60;
	}else if(MCPara[82] == 1){//250
		CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;				
		CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
		CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;	
		CAN_InitStructure.CAN_Prescaler = 30;
	}else if(MCPara[82] == 2){//500
		CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;				
		CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
		CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;	
		CAN_InitStructure.CAN_Prescaler = 15;
	}else if(MCPara[82] == 3){//800
		CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;				
		CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq;
		CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;	
		CAN_InitStructure.CAN_Prescaler = 15;
	}else if(MCPara[82] == 4){//1000
		CAN_InitStructure.CAN_SJW = CAN_SJW_3tq;				
		CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
		CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;	
		CAN_InitStructure.CAN_Prescaler = 10;
	}else{//500
		CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;				
		CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
		CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;	
		CAN_InitStructure.CAN_Prescaler = 15;
	}
	
	CAN_Init(CANx, &CAN_InitStructure);       
	
	//过滤
	CAN_FilterInitStructure.CAN_FilterNumber = 0;	
	
	
//	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;//工作在列表模式 	
//	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; 	
//	
//	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0;	
//	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0;
//	CAN_FilterInitStructure.CAN_FilterMskIdHigh = 0x0;
//	CAN_FilterInitStructure.CAN_FilterMskIdLow = 0x0;
//  CAN_FilterInitStructure.CAN_FilterNumber = 0;	
	
	
	
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;//工作在列表模式 	
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit; 	
	
	CAN_FilterInitStructure.CAN_FilterIdHigh = 					(0x000 << 5) 								+ (0x00 << 4);//数据帧
	CAN_FilterInitStructure.CAN_FilterIdLow = 					((0x700 + setaddr) << 5) 	+ (0x01 << 4);//远程帧
	CAN_FilterInitStructure.CAN_FilterMskIdHigh = 			((0x600 + setaddr) << 5) 	+ (0x00 << 4);//数据帧
	CAN_FilterInitStructure.CAN_FilterMskIdLow = 				((0x600 + setaddr) << 5) 	+ (0x00 << 4);//保留
	
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(CANx, &CAN_FilterInitStructure);
	
	//开启filter0 接收数据中断
	CAN_ClearINTPendingBit(CANx,CAN_INT_RFP0);
  CAN_INTConfig(CANx,CAN_INT_RFP0,ENABLE);
}




static void CAN_NVIC_Config(void)
{
#ifdef AT32_USE_CAN1
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//Can中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}



int32_t canrxcnt = 0;

void USB_LP_CAN1_RX0_IRQHandler (void)
{
	uint8_t i = 0;
	
	if (CAN_GetINTStatus (CANx,CAN_INT_RFOV0) == SET ) 
	{
			CAN_ClearINTPendingBit (CANx, CAN_INT_RFOV0);
	}
	if (CAN_GetINTStatus (CANx,CAN_INT_RFFU0) == SET ) 
	{
			CAN_ClearINTPendingBit (CANx, CAN_INT_RFFU0);
	}
	
	//接收完成中断
	if (CAN_GetINTStatus (CANx,CAN_INT_RFP0) == SET )
	{
			CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
			
			RxMSG.cob_id = RxMessage.StdId;//id
			RxMSG.rtr = RxMessage.RTR ;//rtr
			if(RxMSG.rtr == 2){
				RxMSG.rtr = 1;
			}
			RxMSG.len = RxMessage.DLC ;//len
			for(i = 0;i < RxMSG.len; i ++){
				RxMSG.Rxdata[i] = RxMessage.Data[i];
			}
			//App.Log.Can1msCnt = 0;
			RxMSG.Rxflag = 1;//接收标志位
	}
}


//客户要求的隔一段时间返回一段数据
//void CAN_transmit_Status_data(void)
//{
//	uint8_t TransmitMailbox;
//	uint16_t temp = 0;
//	uint32_t i = 0;
//	CanTxMsg TxMessage;
//	TxMessage.Data[0] = App.Cal.Vbus;
//	TxMessage.Data[1] = App.Cal.SCVoltage;
//	TxMessage.Data[2] = App.Cal.SCCurLPF * 10.0f;
//	TxMessage.Data[3] = (App.Log.DCDC_Sta_Can << 4) + App.Log.CtlSta_Can;
//	TxMessage.Data[4] = App.Cal.MosTemper + 0.5f;
//	temp = App.Spd.RpmNowLPF;
//	TxMessage.Data[5] = temp & 0x00ff;
//	TxMessage.Data[6] = temp >> 8;
//	TxMessage.Data[7] = App.Err;
//    
//	TxMessage.StdId = 0x400;			
//	TxMessage.ExtId = 0;			
//	TxMessage.IDT = CAN_Id_Standard;//标准帧
//	TxMessage.RTR = CAN_RTR_Data;//数据帧
//	TxMessage.DLC = 8;//数据长度
//	TransmitMailbox = CAN_Transmit(CANx, &TxMessage);
//	while((CAN_TransmitStatus(CANx, TransmitMailbox) != CANTXOK) && (i != 0xFFFF)){
//		i++;
//	}
//}





//Can 初始化
void Can_Init(void){
	CAN_StructInit(&CAN_InitStructure);
	CAN_GPIO_Configuration();
	CAN_Configuration(MCPara[0]);
	CAN_NVIC_Config();
}

void Can_Ctl(void){
	CanOpen_Rx();
}


uint8_t CAN1_Send_Msg(uint8_t* msg,uint8_t len,uint16_t cobid){
	uint8_t TransmitMailbox;
	uint32_t i = 0;
	CanTxMsg TxMessage;
 
	TxMessage.StdId = cobid + App.M1.Log.CanOpenAddr;        //标准标识符
  TxMessage.ExtId = 0;			
	TxMessage.IDT = CAN_Id_Standard;//标准帧
	TxMessage.RTR = CAN_RTR_Data;//数据帧
  TxMessage.DLC = len;         //数据长度   
	
	for(i = 0; i < len ; i ++){
		TxMessage.Data[i] = RxMSG.Txdata[i];
	}
	
	i = 0;
	
	TransmitMailbox = CAN_Transmit(CANx, &TxMessage);
	while((CAN_TransmitStatus(CANx, TransmitMailbox) != CANTXOK) && (i != 0xFFFF))
	{
			i++;
	}
	
	return 0;
}



uint8_t CAN1_Send_Msg_Hurt(uint8_t* msg,uint8_t len,uint16_t cobid){
	uint8_t TransmitMailbox;
	uint32_t i = 0;
	CanTxMsg TxMessage;
 
	TxMessage.StdId = cobid;        //标准标识符
  TxMessage.ExtId = 0;			
	TxMessage.IDT = CAN_Id_Standard;//标准帧
	TxMessage.RTR = CAN_RTR_Data;//数据帧
  TxMessage.DLC = len;         //数据长度   
	
	for(i = 0; i < len ; i ++){
		TxMessage.Data[i] = RxMSG.Txdata[i];
	}
	
	i = 0;
	
	TransmitMailbox = CAN_Transmit(CANx, &TxMessage);
	while((CAN_TransmitStatus(CANx, TransmitMailbox) != CANTXOK) && (i != 0xFFFF))
	{
			i++;
	}
	
	return 0;
}


extern void Return_M1Status(void);
extern void Return_M2Status(void);
extern void Return_SysStatus(void);

void CanOpen_Rx(void){
	static uint8_t t = 0;     //翻转
	uint16_t index = 0;//索引
	uint8_t sudex = 0; //子索引
	
	uint32_t temp = 0;
	if(RxMSG.Rxflag ==1){
		RxMSG.Rxflag = 0;
		
		
		index = (RxMSG.Rxdata[2] << 8) + RxMSG.Rxdata[1];//索引
		sudex = RxMSG.Rxdata[3];
		
		//NMT--opencan 
		if(RxMSG.cob_id == 0x000
			&& RxMSG.rtr == 0
			&& (RxMSG.Rxdata[1] == App.M1.Log.CanOpenAddr || RxMSG.Rxdata[1] == 0x00)
		){
			App.Ctl.CanRxOKFg = 1;
								switch (RxMSG.Rxdata[0])
								{
									case  0x01://operational 启动命令（让节点进入操作状态）
									 App.M1.Log.OperaFg = 1;
									 //NMT Module Control 消息不需要应答
										break;
									case  0x02://stop 停止命令（让节点进入停止状态）
									 App.M1.Log.OperaFg = 0;
									//NMT Module Control 消息不需要应答
										break;
									case  0x80://pre-operational 为进入预操作状态（让节点进入预操作状态）
										App.M1.Log.OperaFg = 2;
									 //NMT Module Control 消息不需要应答
										break;
									case  0x81://reset-application 为复位节点应用层（让节点的应用恢复初始状态，比如列车门都恢复打开状态）
									 //NMT Module Control 消息不需要应答
										break;
									case  0x82://reset-communication 为复位节点通讯（让节点的 CAN 和 CANopen 通讯重新初始化，一般用于总线收到干扰，导致节点总线错误被动，或者总线关闭时）
									 
									 
										break;
									
									default :
										//通讯错误
										break;
								}					
		}
		//SDO //SDO 写控制设置
		else if(
			RxMSG.cob_id == (0x600 + App.M1.Log.CanOpenAddr) 
			&&RxMSG.rtr ==	0
		){
								index = (RxMSG.Rxdata[2] << 8) + RxMSG.Rxdata[1];//索引
								sudex = RxMSG.Rxdata[3];
								//Canffor00=(RxMSG.Rxdata[6]<<8)+RxMSG.Rxdata[7];
								//特殊指令
								//保存   索引        子索引           //4bit
								if(index == 0x1010 && sudex == 1 && RxMSG.Rxdata[0] == 0x23)
								{		
														//0x65 76 61 73
														temp = RxMSG.Rxdata[4] + (RxMSG.Rxdata[5] << 8)  + (RxMSG.Rxdata[6] << 16)  + (RxMSG.Rxdata[7] << 24);
														//保存
														//         暗号
														if(
															temp == 0x65766173 
															&& App.M1.AllowRun == 0
															&& App.M2.AllowRun == 0
														){
															FLASH_Write(TEST_FLASH_ADDRESS_START,MCPara,ParaNum);
															FLASH_Write(TEST_FLASH_ADDRESS_START2,MCPara2,ParaNum);
															
															RxMSG.Txdata[0]=0x60;
															RxMSG.Txdata[1]=RxMSG.Rxdata[1];
															RxMSG.Txdata[2]=RxMSG.Rxdata[2];
															RxMSG.Txdata[3]=RxMSG.Rxdata[3];
															RxMSG.Txdata[4]=0x00;
															RxMSG.Txdata[5]=0x00;
															RxMSG.Txdata[6]=0x00;
															RxMSG.Txdata[7]=0x00;
															CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回

														}else{
															RxMSG.Txdata[0]=0x80;
															RxMSG.Txdata[1]=RxMSG.Rxdata[1];
															RxMSG.Txdata[2]=RxMSG.Rxdata[2];
															RxMSG.Txdata[3]=RxMSG.Rxdata[3];
															RxMSG.Txdata[4]=0x06;
															RxMSG.Txdata[5]=0x02;
															RxMSG.Txdata[6]=0x00;
															RxMSG.Txdata[7]=0x00;
															CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
														}
								}
								
								//恢复
								else if(index == 0x1011 && sudex == 1 && RxMSG.Rxdata[0] == 0x23)
								{
														//0x6C 6F 61 64  
														temp = RxMSG.Rxdata[4] + (RxMSG.Rxdata[5] << 8)  + (RxMSG.Rxdata[6] << 16)  + (RxMSG.Rxdata[7] << 24);
														//擦除
														//         暗号
														if(
															temp == 0x64616F6C 
															&& App.M1.AllowRun == 0
															&& App.M2.AllowRun == 0
														){
															F4Flash_SMCPara_Erase(TEST_FLASH_ADDRESS_START);
															F4Flash_SMCPara_Erase(TEST_FLASH_ADDRESS_START2);
															
															RxMSG.Txdata[0]=0x60;
															RxMSG.Txdata[1]=RxMSG.Rxdata[1];
															RxMSG.Txdata[2]=RxMSG.Rxdata[2];
															RxMSG.Txdata[3]=RxMSG.Rxdata[3];
															RxMSG.Txdata[4]=0x00;
															RxMSG.Txdata[5]=0x00;
															RxMSG.Txdata[6]=0x00;
															RxMSG.Txdata[7]=0x00;
															CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回

														}else{
															RxMSG.Txdata[0]=0x80;
															RxMSG.Txdata[1]=RxMSG.Rxdata[1];
															RxMSG.Txdata[2]=RxMSG.Rxdata[2];
															RxMSG.Txdata[3]=RxMSG.Rxdata[3];
															RxMSG.Txdata[4]=0x06;
															RxMSG.Txdata[5]=0x02;
															RxMSG.Txdata[6]=0x00;
															RxMSG.Txdata[7]=0x00;
															CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
														}
								}
								
								//变威力电机挡位设定 0-4 1-2F
								else if(index==0x1012 && sudex == 1 && RxMSG.Rxdata[0] == 0x2F)
								{																	//地位在前，高位在后				
													//已经回原的情况下才接收这个指令
													if(
														RxMSG.Rxdata[4] <= 4
														&& RxMSG.Rxdata[4] >= 1
														//&& App.M1.Log.OperaFg >= 1 //Can开机状态
														&& App.M2.Pos.PosLocaFg == 1//变威力电机回原完成
														//&& App.M1.AllowRun == 0//供弹电机在停止状态下
														&& App.M1.Pos.PosStatus != CTLS_SPD
														&& App.M1.Pos.PosStatus != CTLS_RUN
														&& App.M2.Pos.PosStatus != POS_INIT
													
														&& App.M1.Err == E_NONE//无错误情况下
														&& App.M2.Err == E_NONE//无错误情况下
													){
														
														App.Ctl.SetWeiNum = RxMSG.Rxdata[4];//设置挡位
														if(MCPara2[99] == 0){
															if(App.Ctl.SetWeiNum == 0){
																//App.M2.Pos.SetCnt = 0;
															}else if(App.Ctl.SetWeiNum == 1){
																App.M2.Pos.SetCnt = MCPara2[100] * 10;
																//App.M2.Pos.SetCnt = 0;
																App.M2.Pos.PosArriFg = 0;
																App.M2.Pos.PosStart = 1;
															}else if(App.Ctl.SetWeiNum == 2){
																App.M2.Pos.SetCnt = MCPara2[101] * 10;
																App.M2.Pos.PosArriFg = 0;
																App.M2.Pos.PosStart = 1;
															}else if(App.Ctl.SetWeiNum == 3){
																App.M2.Pos.SetCnt = MCPara2[102] * 10;
																App.M2.Pos.PosArriFg = 0;
																App.M2.Pos.PosStart = 1;
															}else if(App.Ctl.SetWeiNum == 4){
																App.M2.Pos.SetCnt = MCPara2[103] * 10;
																App.M2.Pos.PosArriFg = 0;
																App.M2.Pos.PosStart = 1;
															}
														}else {
															if(App.Ctl.SetWeiNum == 0){
																//App.M2.Pos.SetCnt = 0;
															}else if(App.Ctl.SetWeiNum == 1){
																App.M2.Pos.SetCnt = -MCPara2[100] * 10;
																//App.M2.Pos.SetCnt = -0;
																App.M2.Pos.PosArriFg = 0;
																App.M2.Pos.PosStart = 1;
															}else if(App.Ctl.SetWeiNum == 2){
																App.M2.Pos.SetCnt = -MCPara2[101] * 10;
																App.M2.Pos.PosArriFg = 0;
																App.M2.Pos.PosStart = 1;
															}else if(App.Ctl.SetWeiNum == 3){
																App.M2.Pos.SetCnt = -MCPara2[102] * 10;
																App.M2.Pos.PosArriFg = 0;
																App.M2.Pos.PosStart = 1;
															}else if(App.Ctl.SetWeiNum == 4){
																App.M2.Pos.SetCnt = -MCPara2[103] * 10;
																App.M2.Pos.PosArriFg = 0;
																App.M2.Pos.PosStart = 1;
															}
														}
														
														
														//返回
														RxMSG.Txdata[0]=0x60;
														RxMSG.Txdata[1]=RxMSG.Rxdata[1];
														RxMSG.Txdata[2]=RxMSG.Rxdata[2];
														RxMSG.Txdata[3]=RxMSG.Rxdata[3];
														RxMSG.Txdata[4]=0x00;
														RxMSG.Txdata[5]=0x00;
														RxMSG.Txdata[6]=0x00;
														RxMSG.Txdata[7]=0x00;
														CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
													}else {
														RxMSG.Txdata[0]=0x80;
														RxMSG.Txdata[1]=RxMSG.Rxdata[1];
														RxMSG.Txdata[2]=RxMSG.Rxdata[2];
														RxMSG.Txdata[3]=RxMSG.Rxdata[3];
														RxMSG.Txdata[4]=0x06;
														RxMSG.Txdata[5]=0x02;
														RxMSG.Txdata[6]=0x00;
														RxMSG.Txdata[7]=0x00;
														CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
														
													}
								}
								
								//供弹电机圈数设定                                     2bit
								else if(index==0x1013 && sudex == 1 && RxMSG.Rxdata[0] == 0x2B)
								{
									
														//威力电机回原命令-只有在等待状态下才能回原
														if(
															App.M1.Log.OperaFg >= 1 //Can开机状态
															&& App.M2.Pos.PosLocaFg == 1//变威力电机回原完成
															&& App.Ctl.SetWeiNum >= 1//变威力电机设定了挡位
															&& App.M2.Pos.PosArriFg == 1//如果已经到了设定的挡位
															//&& App.M2.Pos.PosStatus == POS_WAIT
															//&& App.M2.AllowRun == 0//供弹电机在停止状态下
															&& App.M1.Err == E_NONE//无错误情况下
															&& App.M2.Err == E_NONE//无错误情况下
														){
															//供弹电机设定圈数
															App.Ctl.NowTurns = 0;
															App.Ctl.SetTurns = RxMSG.Rxdata[4] + (RxMSG.Rxdata[5] << 8);
															
															RxMSG.Txdata[0]=0x60;
															RxMSG.Txdata[1]=RxMSG.Rxdata[1];
															RxMSG.Txdata[2]=RxMSG.Rxdata[2];
															RxMSG.Txdata[3]=RxMSG.Rxdata[3];
															RxMSG.Txdata[4]=0x00;
															RxMSG.Txdata[5]=0x00;
															RxMSG.Txdata[6]=0x00;
															RxMSG.Txdata[7]=0x00;
															CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回

															
														}else {
															RxMSG.Txdata[0]=0x80;
															RxMSG.Txdata[1]=RxMSG.Rxdata[1];
															RxMSG.Txdata[2]=RxMSG.Rxdata[2];
															RxMSG.Txdata[3]=RxMSG.Rxdata[3];
															RxMSG.Txdata[4]=0x06;
															RxMSG.Txdata[5]=0x02;
															RxMSG.Txdata[6]=0x00;
															RxMSG.Txdata[7]=0x00;
															CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
														}
								}
								
								//结束击发指令                                     4bit
								else if(index==0x1014 && sudex == 1 && RxMSG.Rxdata[0] == 0x23)
								{
													if(
														RxMSG.Rxdata[4] == 0x0A
														&& RxMSG.Rxdata[5] == 0x0B
														&& RxMSG.Rxdata[6] == 0x0A
														&& RxMSG.Rxdata[7] == 0x0B
													){
														App.Ctl.NowTurns = 0;
														App.Ctl.SetTurns = 0;
														
														RxMSG.Txdata[0]=0x60;
														RxMSG.Txdata[1]=RxMSG.Rxdata[1];
														RxMSG.Txdata[2]=RxMSG.Rxdata[2];
														RxMSG.Txdata[3]=RxMSG.Rxdata[3];
														RxMSG.Txdata[4]=0x00;
														RxMSG.Txdata[5]=0x00;
														RxMSG.Txdata[6]=0x00;
														RxMSG.Txdata[7]=0x00;
														CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
													}else {
														RxMSG.Txdata[0]=0x80;
														RxMSG.Txdata[1]=RxMSG.Rxdata[1];
														RxMSG.Txdata[2]=RxMSG.Rxdata[2];
														RxMSG.Txdata[3]=RxMSG.Rxdata[3];
														RxMSG.Txdata[4]=0x06;
														RxMSG.Txdata[5]=0x02;
														RxMSG.Txdata[6]=0x00;
														RxMSG.Txdata[7]=0x00;
														CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
													}
								}
								
								//威力电机回原指令                                 				1bit
								else if(index==0x1015 && sudex==1 && RxMSG.Rxdata[0] == 0x2F)
								{
														//威力电机回原命令-只有在等待状态下才能回原
														if(
															RxMSG.Rxdata[4] == 1
															//&& App.M1.Log.OperaFg >= 1 //Can开机状态
															//&& App.M2.Pos.PosStatus == POS_WAIT
															//&& App.M1.Pos.PosStatus == POS_WAIT
															//&& App.M1.AllowRun == 0//供弹电机在停止状态下
															&& App.M1.Pos.PosStatus != CTLS_SPD
															&& App.M1.Pos.PosStatus != CTLS_RUN
														
															&& App.M1.Err == E_NONE//无错误情况下
															&& App.M2.Err == E_NONE//无错误情况下
														){
															App.M2.Pos.PosLocaFg = 0;
															App.Ctl.SetWeiNum = 0;//归零挡位设定
															App.M2.Pos.PosStart = 2;//回原
														
														
															RxMSG.Txdata[0]=0x60;
															RxMSG.Txdata[1]=RxMSG.Rxdata[1];
															RxMSG.Txdata[2]=RxMSG.Rxdata[2];
															RxMSG.Txdata[3]=RxMSG.Rxdata[3];
															RxMSG.Txdata[4]=0x00;
															RxMSG.Txdata[5]=0x00;
															RxMSG.Txdata[6]=0x00;
															RxMSG.Txdata[7]=0x00;
															CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
														
													}else {
														RxMSG.Txdata[0]=0x80;
														RxMSG.Txdata[1]=RxMSG.Rxdata[1];
														RxMSG.Txdata[2]=RxMSG.Rxdata[2];
														RxMSG.Txdata[3]=RxMSG.Rxdata[3];
														RxMSG.Txdata[4]=0x06;
														RxMSG.Txdata[5]=0x02;
														RxMSG.Txdata[6]=0x00;
														RxMSG.Txdata[7]=0x00;
														CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
													}
								}
								
								
								//电磁阀开合指令                                     1bit
								else if(index==0x1016 && sudex==1 && RxMSG.Rxdata[0] == 0x2F)
								{
									#if _Relay_SW == 0
												if(RxMSG.Rxdata[4] == 1){
													Relay_ON;
												}else {
													Relay_OFF;
												}
									#endif
												
												RxMSG.Txdata[0]=0x60;
												RxMSG.Txdata[1]=RxMSG.Rxdata[1];
												RxMSG.Txdata[2]=RxMSG.Rxdata[2];
												RxMSG.Txdata[3]=RxMSG.Rxdata[3];
												RxMSG.Txdata[4]=0x00;
												RxMSG.Txdata[5]=0x00;
												RxMSG.Txdata[6]=0x00;
												RxMSG.Txdata[7]=0x00;
												CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
												
													
								}
								
								//驱动器状态修改
								else if(index==0x1017 && sudex==1 && RxMSG.Rxdata[0] == 0x2F)
								{
												if(
													RxMSG.Rxdata[4] == 1
													&& App.M1.Err == E_NONE//无错误情况下
													&& App.M2.Err == E_NONE//无错误情况下
												){
													App.M1.Log.OperaFg = 1;
												}else {
													App.M1.Log.OperaFg = 0;
												}
												
												RxMSG.Txdata[0]=0x60;
												RxMSG.Txdata[1]=RxMSG.Rxdata[1];
												RxMSG.Txdata[2]=RxMSG.Rxdata[2];
												RxMSG.Txdata[3]=RxMSG.Rxdata[3];
												RxMSG.Txdata[4]=0x00;
												RxMSG.Txdata[5]=0x00;
												RxMSG.Txdata[6]=0x00;
												RxMSG.Txdata[7]=0x00;
												CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
								}
								
								//清除错误指令
								else if(index==0x1018 && sudex==1 && RxMSG.Rxdata[0] == 0x2F)
								{
									
									if(
										RxMSG.Rxdata[4] == 0x0C
									){
										
//										if(
//											App.M1.Err == E_ENOC
//											|| App.M2.Err == E_ENOC
//											|| App.M1.Err == E_READ
//											|| App.M2.Err == E_READ
//										){
//											RxMSG.Txdata[0]=0x80;
//											RxMSG.Txdata[1]=RxMSG.Rxdata[1];
//											RxMSG.Txdata[2]=RxMSG.Rxdata[2];
//											RxMSG.Txdata[3]=RxMSG.Rxdata[3];
//											RxMSG.Txdata[4]=0x06;
//											RxMSG.Txdata[5]=0x02;
//											RxMSG.Txdata[6]=0x00;
//											RxMSG.Txdata[7]=0x00;
//											CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
//										}else {
//											App.M1.Err = E_NONE;
//											App.M2.Err = E_NONE;
//											
//											App.Ctl.SetTurns = 0;
//											App.Ctl.NowTurns = 0;
//											
//											App.M1.Pos.SetCnt = App.M1.Pos.NowCnt;
//											App.M2.Pos.SetCnt = App.M2.Pos.NowCnt;
//											
//											RxMSG.Txdata[0]=0x60;
//											RxMSG.Txdata[1]=RxMSG.Rxdata[1];
//											RxMSG.Txdata[2]=RxMSG.Rxdata[2];
//											RxMSG.Txdata[3]=RxMSG.Rxdata[3];
//											RxMSG.Txdata[4]=0x00;
//											RxMSG.Txdata[5]=0x00;
//											RxMSG.Txdata[6]=0x00;
//											RxMSG.Txdata[7]=0x00;
//											CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回

//										}

										if(
											((App.BitErr & ERR_WL_ENOC_Loss) != 0)
											|| ((App.BitErr & ERR_GD_ENOC_Loss) != 0)
											
											|| ((App.BitErr & ERR_WL_Hall_Loss) != 0)
											|| ((App.BitErr & ERR_GD_Hall_Loss) != 0)
											
											|| ((App.BitErr & ERR_WL_FAIL) != 0)
											|| ((App.BitErr & ERR_GD_FAIL) != 0)
										
											|| ((App.BitErr & ERR_XT_Save_MCPara) != 0)
											
										){
											RxMSG.Txdata[0]=0x80;
											RxMSG.Txdata[1]=RxMSG.Rxdata[1];
											RxMSG.Txdata[2]=RxMSG.Rxdata[2];
											RxMSG.Txdata[3]=RxMSG.Rxdata[3];
											RxMSG.Txdata[4]=0x06;
											RxMSG.Txdata[5]=0x02;
											RxMSG.Txdata[6]=0x00;
											RxMSG.Txdata[7]=0x00;
											CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
										}else {
											App.M1.Err = E_NONE;
											App.M2.Err = E_NONE;
											App.BitErr = 0;
											
											App.Ctl.SetTurns = 0;
											App.Ctl.NowTurns = 0;
											
											App.M1.Pos.SetCnt = App.M1.Pos.NowCnt;
											App.M2.Pos.SetCnt = App.M2.Pos.NowCnt;
											
											RxMSG.Txdata[0]=0x60;
											RxMSG.Txdata[1]=RxMSG.Rxdata[1];
											RxMSG.Txdata[2]=RxMSG.Rxdata[2];
											RxMSG.Txdata[3]=RxMSG.Rxdata[3];
											RxMSG.Txdata[4]=0x00;
											RxMSG.Txdata[5]=0x00;
											RxMSG.Txdata[6]=0x00;
											RxMSG.Txdata[7]=0x00;
											CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回

										}
										
										
									}else {
										RxMSG.Txdata[0]=0x80;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x06;
										RxMSG.Txdata[5]=0x02;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
									}
								}
								
								//驱动器芯片复位重启指令
								else if(index==0x1019 && sudex == 1 && RxMSG.Rxdata[0] == 0x23)
								{
									//0x12345678
									temp = RxMSG.Rxdata[4] + (RxMSG.Rxdata[5] << 8)  + (RxMSG.Rxdata[6] << 16)  + (RxMSG.Rxdata[7] << 24);
									if(
										temp == 0x12345678 
										&& App.M1.AllowRun == 0
										&& App.M2.AllowRun == 0
									){
										RxMSG.Txdata[0]=0x60;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x00;
										RxMSG.Txdata[5]=0x00;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
					
										//直接复位
										
										__set_FAULTMASK(1);
										NVIC_SystemReset();
									}else {
										//错误
										RxMSG.Txdata[0]=0x80;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x06;
										RxMSG.Txdata[5]=0x02;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
									}
								}
								
								//编码器校正指令
								else if(index==0x101A && sudex == 1 && RxMSG.Rxdata[0] == 0x2F)
								{
									
									if(
										RxMSG.Rxdata[4] == 0x01
										&& App.M1.Err == E_NONE//无错误情况下
										&& App.M2.Err == E_NONE//无错误情况下
									){
										
										if(
											myabs(App.M1.Spd.Now) < 100 
											&& App.M1.Hpll.HallStadySta != 2
										){
											App.M1.Hpll.HallStadySta = 1;//hall学习
											App.Ctl.SetTurns = 0;
											
											App.M1.Log.OperaFg = 0;
											App.M1.Pos.PosLocaFg = 0;//清除位置信息
											App.M1.Enoc.Sta = 0;
											App.M1.Enoc.Enoc_I_Cnt = 0;
										}	
										
										RxMSG.Txdata[0]=0x60;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x00;
										RxMSG.Txdata[5]=0x00;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
					
									}
									
									else if(
										RxMSG.Rxdata[4] == 0x02
										&& App.M1.Err == E_NONE//无错误情况下
										&& App.M2.Err == E_NONE//无错误情况下
									){
										
										if(
											myabs(App.M2.Spd.Now) < 100 
											&& App.M2.Hpll.HallStadySta != 2
										){
											App.M2.Pos.PosLocaFg = 0;
											App.M2.Hpll.HallStadySta = 1;//hall学习
											App.Ctl.SetTurns = 0;
											
											App.M1.Log.OperaFg = 0;
											App.M2.Pos.PosLocaFg = 0;
											App.Ctl.SetWeiNum = 0;
											
											App.M2.Pos.PosLocaFg = 0;//清除位置信息
											App.M2.Enoc.Sta = 0;
											App.M2.Enoc.Enoc_I_Cnt = 0;
											
										}	
										
										RxMSG.Txdata[0]=0x60;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x00;
										RxMSG.Txdata[5]=0x00;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
					
									}
									
									//清除相角信息
									else if(
										RxMSG.Rxdata[4] == 0xFF
										&& App.M1.AllowRun == 0
										&& App.M2.AllowRun == 0
									){
										
										App.M1.Hall.SdSunXuCnt = 0;
										App.M1.Hpll.HallAllRun = 0;
										App.M1.Hpll.HallStadySta = 0;
										
										App.M1.Log.OperaFg = 0;
										App.M1.Pos.PosLocaFg = 0;//清除位置信息
										App.M1.Enoc.Sta = 0;
										App.M1.Enoc.Enoc_I_Cnt = 0;
										
										
										App.M2.Hall.SdSunXuCnt = 0;
										App.M2.Hpll.HallAllRun = 0;
										App.M2.Hpll.HallStadySta = 0;
												 
										App.M2.Log.OperaFg = 0;
										App.M2.Pos.PosLocaFg = 0;//清除位置信息
										App.M2.Enoc.Sta = 0;
										App.M2.Enoc.Enoc_I_Cnt = 0;
										
										
										F4Flash_SMCPara_Erase(TEST_FLASH_ADDRESS_START_HALL);
										F4Flash_SMCPara_Erase(TEST_FLASH_ADDRESS_START_HALL2);
										
										
										
										RxMSG.Txdata[0]=0x60;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x00;
										RxMSG.Txdata[5]=0x00;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
										
									}
									
									else {
										//错误
										RxMSG.Txdata[0]=0x80;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x06;
										RxMSG.Txdata[5]=0x02;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
									}
								}
								
								
								
								//测试模式供弹电机转速设定
								else if(index==0x101B && sudex == 1 && RxMSG.Rxdata[0] == 0x2B)
								{
									App.M1.Log.TestMode = 1;
									App.M2.Log.TestMode = 1;
									
									App.M1.Log.TestSpd = RxMSG.Rxdata[4] + (RxMSG.Rxdata[5] << 8);
									App.M2.Log.TestSpd = RxMSG.Rxdata[6] + (RxMSG.Rxdata[7] << 8);
									
									RxMSG.Txdata[0]=0x60;
									RxMSG.Txdata[1]=RxMSG.Rxdata[1];
									RxMSG.Txdata[2]=RxMSG.Rxdata[2];
									RxMSG.Txdata[3]=RxMSG.Rxdata[3];
									RxMSG.Txdata[4]=0x00;
									RxMSG.Txdata[5]=0x00;
									RxMSG.Txdata[6]=0x00;
									RxMSG.Txdata[7]=0x00;
									CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
									
								}
								
								//急停指令
								else if(index==0x101C && sudex == 1 && RxMSG.Rxdata[0] == 0x2F)
								{
									
									//解除急停
									if(
										RxMSG.Rxdata[4] == 0x0C
									){
										
										RxMSG.Txdata[0]=0x60;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x00;
										RxMSG.Txdata[5]=0x00;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
									
									
										//直接复位
										
										__set_FAULTMASK(1);
										NVIC_SystemReset();
										
									}
									//急停
									else if(
										RxMSG.Rxdata[4] == 0x0E
									){
										
										App.EmergencyStop = 1;
										
										RxMSG.Txdata[0]=0x60;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x00;
										RxMSG.Txdata[5]=0x00;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
									
									}
									else {
										RxMSG.Txdata[0]=0x80;
										RxMSG.Txdata[1]=RxMSG.Rxdata[1];
										RxMSG.Txdata[2]=RxMSG.Rxdata[2];
										RxMSG.Txdata[3]=RxMSG.Rxdata[3];
										RxMSG.Txdata[4]=0x06;
										RxMSG.Txdata[5]=0x02;
										RxMSG.Txdata[6]=0x00;
										RxMSG.Txdata[7]=0x00;
										CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
									}
									
								}
								
								
								
								
								
								
								//更改参数
								else 
								{
									//判定字节
									if(
										RxMSG.Rxdata[0] == 0x2B //2bit 
//										RxMSG.Rxdata[0] == 0x2F    //1bit
//										|| RxMSG.Rxdata[0] == 0x2B //2bit 
//										|| RxMSG.Rxdata[0] == 0x27 //3bit
//										|| RxMSG.Rxdata[0] == 0x23 //4bit
									){
														if((index&0xff00) == 0x4100)//设置MCPara[]
														{
															//  子索引不限制
															if((index&0x00ff)<ParaNum)//确保要写的数据不能超出数组的范围，以免发生位置错误
															{
																	MCPara[(index&0x00ff)] = (RxMSG.Rxdata[5] << 8) + RxMSG.Rxdata[4];
																	
																	//如果修改了地址 还有修改了Can速度
																	if((index&0x00ff) == 0 || (index&0x00ff) == 82){
																		App.M1.Log.CanOpenAddr = MCPara[0];
																		CAN_Configuration(App.M1.Log.CanOpenAddr);//重新更新Can地址和波特率
																	}
																	
																	RxMSG.Txdata[0] = 0x60;
																	RxMSG.Txdata[1] = RxMSG.Rxdata[1];
																	RxMSG.Txdata[2] = RxMSG.Rxdata[2];
																	RxMSG.Txdata[3] = RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x00;
																	RxMSG.Txdata[5]=0x00;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
															else//对象不在字典中
															{
																	RxMSG.Txdata[0]=0x80;
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x06;
																	RxMSG.Txdata[5]=0x02;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
														}
														else if((index&0xff00) == 0x4200)//设置MCPara2[]
														{
															//  子索引不限制
															if((index&0x00ff)<ParaNum)//确保要写的数据不能超出数组的范围，以免发生位置错误
															{
																	MCPara2[(index&0x00ff)] = (RxMSG.Rxdata[5] << 8) + RxMSG.Rxdata[4];
																	
																	RxMSG.Txdata[0] = 0x60;
																	RxMSG.Txdata[1] = RxMSG.Rxdata[1];
																	RxMSG.Txdata[2] = RxMSG.Rxdata[2];
																	RxMSG.Txdata[3] = RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x00;
																	RxMSG.Txdata[5]=0x00;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
															else//对象不在字典中
															{
																	RxMSG.Txdata[0]=0x80;
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x06;
																	RxMSG.Txdata[5]=0x02;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
														}
														else //SUB-INDEX 不存在;
														{
																	RxMSG.Txdata[0]=0x80;
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x06;
																	RxMSG.Txdata[5]=0x09;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x11;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
														}
									}
									
									//查询
									else if (RxMSG.Rxdata[0]==0x40)  
									{
										//返回：
										//4F-1bit
										//4B-2bit
										//47-3bit
										//43-4bit
										
														if((index&0xff00) == 0x2000)//系统状态
														{
															if(sudex==1 && (index&0x00ff)<StatusNum)//确保要写的数据不能超出数组的范围，以免发生位置错误
															{
																	Return_SysStatus();
																	RxMSG.Txdata[0]=0x43;//状态寄存器全部返回int32_t的数据类型
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																
																	RxMSG.Txdata[4]=MCStatus[(index&0x00ff)];
																	RxMSG.Txdata[5]=MCStatus[(index&0x00ff)]>>8;
																	RxMSG.Txdata[6]=MCStatus[(index&0x00ff)]>>16;
																	RxMSG.Txdata[7]=MCStatus[(index&0x00ff)]>>24;
																
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
															else//对象不在字典中
															{
																	RxMSG.Txdata[0]=0x80;
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x06;
																	RxMSG.Txdata[5]=0x02;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
														}
														
														else if((index&0xff00) == 0x2100)//供弹电机状态
														{
															if(sudex==1 && (index&0x00ff)<StatusNum)//确保要写的数据不能超出数组的范围，以免发生位置错误
															{
																	Return_M1Status();
																	RxMSG.Txdata[0]=0x43;//状态寄存器全部返回int32_t的数据类型
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																
																	RxMSG.Txdata[4]=MCStatus[(index&0x00ff)];
																	RxMSG.Txdata[5]=MCStatus[(index&0x00ff)]>>8;
																	RxMSG.Txdata[6]=MCStatus[(index&0x00ff)]>>16;
																	RxMSG.Txdata[7]=MCStatus[(index&0x00ff)]>>24;
																
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
															else//对象不在字典中
															{
																	RxMSG.Txdata[0]=0x80;
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x06;
																	RxMSG.Txdata[5]=0x02;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
														}
														
														
														else if((index&0xff00) == 0x2200)//变威力电机状态
														{
															if(sudex==1 && (index&0x00ff)<StatusNum)//确保要写的数据不能超出数组的范围，以免发生位置错误
															{
																	Return_M2Status();
																	RxMSG.Txdata[0]=0x43;//状态寄存器全部返回int32_t的数据类型
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																
																	RxMSG.Txdata[4]=MCStatus[(index&0x00ff)];
																	RxMSG.Txdata[5]=MCStatus[(index&0x00ff)]>>8;
																	RxMSG.Txdata[6]=MCStatus[(index&0x00ff)]>>16;
																	RxMSG.Txdata[7]=MCStatus[(index&0x00ff)]>>24;
																
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
															else//对象不在字典中
															{
																	RxMSG.Txdata[0]=0x80;
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x06;
																	RxMSG.Txdata[5]=0x02;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
														}
														
														else if((index&0xff00) == 0x4100)//MCPara 参数
														{
															if(sudex==1 && (index&0x00ff)<ParaNum)//确保要写的数据不能超出数组的范围，以免发生位置错误
															{
																	RxMSG.Txdata[0]=0x4B;//状态寄存器全部返回uint16_t的数据类型
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=MCPara[(index&0x00ff)];
																	RxMSG.Txdata[5]=MCPara[(index&0x00ff)]>>8;
																	RxMSG.Txdata[6]=0;
																	RxMSG.Txdata[7]=0;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
															else//对象不在字典中
															{
																	RxMSG.Txdata[0]=0x80;
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x06;
																	RxMSG.Txdata[5]=0x02;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
														}
														
														else if((index&0xff00) == 0x4200)//MCPara2 参数
														{
															if(sudex==1 && (index&0x00ff)<ParaNum)//确保要写的数据不能超出数组的范围，以免发生位置错误
															{
																	RxMSG.Txdata[0]=0x4B;//状态寄存器全部返回uint16_t的数据类型
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=MCPara2[(index&0x00ff)];
																	RxMSG.Txdata[5]=MCPara2[(index&0x00ff)]>>8;
																	RxMSG.Txdata[6]=0;
																	RxMSG.Txdata[7]=0;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
															else//对象不在字典中
															{
																	RxMSG.Txdata[0]=0x80;
																	RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																	RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																	RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																	RxMSG.Txdata[4]=0x06;
																	RxMSG.Txdata[5]=0x02;
																	RxMSG.Txdata[6]=0x00;
																	RxMSG.Txdata[7]=0x00;
																	CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
															}
														}
														
														//错误
														else{
																RxMSG.Txdata[0]=0x80;
																RxMSG.Txdata[1]=RxMSG.Rxdata[1];
																RxMSG.Txdata[2]=RxMSG.Rxdata[2];
																RxMSG.Txdata[3]=RxMSG.Rxdata[3];
																RxMSG.Txdata[4]=0x06;
																RxMSG.Txdata[5]=0x07;
																RxMSG.Txdata[6]=0x00;
																RxMSG.Txdata[7]=0x10;
																CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
														}
											}
							
											//错误
											else{
													RxMSG.Txdata[0]=0x80;
													RxMSG.Txdata[1]=RxMSG.Rxdata[1];
													RxMSG.Txdata[2]=RxMSG.Rxdata[2];
													RxMSG.Txdata[3]=RxMSG.Rxdata[3];
													RxMSG.Txdata[4]=0x06;
													RxMSG.Txdata[5]=0x07;
													RxMSG.Txdata[6]=0x00;
													RxMSG.Txdata[7]=0x10;
													CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
											}	
							}								
		}
		//状态查询
		else if(
			RxMSG.cob_id == (0x700 + App.M1.Log.CanOpenAddr)
			&& RxMSG.rtr == 1//远程帧
		){//Node guard protocol
								if(App.M1.Log.OperaFg == 1){//可操作状态
									RxMSG.state = 0x05;
								}else if(App.M1.Log.OperaFg == 0){//不可操作状态
									RxMSG.state = 0x04;
								}else if(App.M1.Log.OperaFg == 2){//预操作
									RxMSG.state = 0x7F;
								}
								
								RxMSG.Txdata[0] = RxMSG.state + t;
								RxMSG.Txdata[1]=0x00;
								RxMSG.Txdata[2]=0x00;
								RxMSG.Txdata[3]=0x00;
								RxMSG.Txdata[4]=0x00;
								RxMSG.Txdata[5]=0x00;
								RxMSG.Txdata[6]=0x00;
								RxMSG.Txdata[7]=0x00;
								
								CAN1_Send_Msg(RxMSG.Txdata,1,0x700);//返回
								if(t == 0){t = 0x80;}else{t = 0;}
								
//								RxMSG.Txdata[0] = t;
//								RxMSG.Txdata[1] = RxMSG.state;
//								CAN1_Send_Msg(RxMSG.Txdata,8,0x700);//返回
//								if(t == 0){t = 1;}else{t = 0;}
		}
		else //clien server 命令标识符无效或未知
		{
								RxMSG.Txdata[0]=0x80;
								RxMSG.Txdata[1]=RxMSG.Rxdata[1];
								RxMSG.Txdata[2]=RxMSG.Rxdata[2];
								RxMSG.Txdata[3]=RxMSG.Rxdata[3];
								RxMSG.Txdata[4]=0x05;
								RxMSG.Txdata[5]=0x04;
								RxMSG.Txdata[6]=0x00;
								RxMSG.Txdata[7]=0x01;
		}
	}
	
	//接近开关触发一次就发送一次数据
	if(App.Ctl.PrsINSendFg == 1){
		App.Ctl.PrsINSendFg = 0;
		
		//放在中断中加
		//App.Ctl.PrsINSendCnt ++;
		
		RxMSG.Txdata[0] = App.Ctl.PrsINSendCnt;
		RxMSG.Txdata[1]	=	App.Ctl.PrsINSendCnt >> 8;
		RxMSG.Txdata[2]	=	App.Ctl.PrsINSendCnt >> 16;
		RxMSG.Txdata[3]	=	App.Ctl.PrsINSendCnt >> 24;
		RxMSG.Txdata[4]	=	App.Ctl.PrsINSendCnt >> 38;
		RxMSG.Txdata[5]	=	App.Ctl.PrsINSendCnt >> 40;
		RxMSG.Txdata[6]	=	App.Ctl.PrsINSendCnt >> 48;
		RxMSG.Txdata[7]	=	App.Ctl.PrsINSendCnt >> 56;
		CAN1_Send_Msg_Hurt(RxMSG.Txdata,8,0x1AA);//返回
	}
	
	//心跳
	else if(App.M1.Tim.CanHeart1msFg == 1){
		App.M1.Tim.CanHeart1msFg = 0;
		
		App.M1.Tim.CanHeart1msCnt ++;
		
		if(App.M1.Tim.CanHeart1msCnt >= MCPara[71] && MCPara[71] != 0){
			App.M1.Tim.CanHeart1msCnt = 0;
			
			//驱动器心跳
			App.Ctl.CanHear ++;
			RxMSG.Txdata[0] = App.Ctl.CanHear;//RxMSG.state;
			
			//系统错误
			RxMSG.Txdata[1] = App.BitErr >> 16;
			
			//反馈电机相角学习状态
			temp = App.M2.Hpll.HallStadySta * 16 + App.M1.Hpll.HallStadySta;
			RxMSG.Txdata[2] = temp;
			
			//击发挡位设置+回原状态+驱动器状态+收到主控消息+击发状态
			temp = 0;
			if(App.Ctl.SetWeiNum >= 1 && App.M2.Pos.PosArriFg == 1){
				temp += (App.Ctl.SetWeiNum << 5);//击发挡位设置
			}else {
				temp += (0 << 5);//击发挡位设置
			}
			if(App.M2.Pos.PosLocaFg == 1){//回原状态
				temp += (1 << 3);
			}else if(App.M2.Pos.PosStatus == POS_INIT){
				temp += (2 << 3);
			}else {
				temp += (0 << 3);
			}
			if(App.M1.Log.OperaFg == 1){//驱动器状态
				temp += (1 << 2);
			}
			if(App.Ctl.CanRxOKFg == 1){//收到主控消息
				temp += (1 << 1);
			}
			if(App.M1.Pos.PosStatus == CTLS_SPD || App.M1.Pos.PosStatus == CTLS_RUN){//收到主控消息
				temp += 1;
			}
			RxMSG.Txdata[3] = temp;
			
			//供弹电机错误
			RxMSG.Txdata[4] = App.BitErr >> 8;
			//威力电机错误
			RxMSG.Txdata[5] = App.BitErr;			
			
			
			//气瓶气压状态
			temp = App.Ctl.AirPress * 1000;
			RxMSG.Txdata[6] = temp;
			RxMSG.Txdata[7] = temp >> 8;
			
			
			CAN1_Send_Msg_Hurt(RxMSG.Txdata,8,0x1AB);//返回
		}
	}
	
	
	
	//开机之后，发送节点启动消息
	if(App.M1.Tim.CanStartFg == 0){
		App.M1.Tim.CanStartFg = 1;
		
		RxMSG.state = 0x00;
		
		RxMSG.Txdata[0] = RxMSG.state;
		RxMSG.Txdata[1]=0x00;
		RxMSG.Txdata[2]=0x00;
		RxMSG.Txdata[3]=0x00;
		RxMSG.Txdata[4]=0x00;
		RxMSG.Txdata[5]=0x00;
		RxMSG.Txdata[6]=0x00;
		RxMSG.Txdata[7]=0x00;
		CAN1_Send_Msg(RxMSG.Txdata,1,0x700);//返回
	}
	
//	//错误发生之后，自动上传一帧数据
//	if(App.M1.Tim.CanErrSendFg == 0 && App.M1.Err > 0){
//		App.M1.Tim.CanErrSendFg = 1;
//		
//		
//		RxMSG.Txdata[0]=0x43;//状态寄存器全部返回int32_t的数据类型
//		RxMSG.Txdata[1]=0x0A;
//		RxMSG.Txdata[2]=0x21;
//		RxMSG.Txdata[3]=0x01;
//	
//		RxMSG.Txdata[4]=MCStatus[10];
//		RxMSG.Txdata[5]=MCStatus[10]>>8;
//		RxMSG.Txdata[6]=MCStatus[10]>>16;
//		RxMSG.Txdata[7]=MCStatus[10]>>24;
//	
//		CAN1_Send_Msg(RxMSG.Txdata,8,0x580);//返回
//	}
	
	
}

