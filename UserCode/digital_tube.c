#include "digital_tube.h"

#ifdef use_digital_tube
#include "mcctl.h"
//0~9显示代码 //0:0-9:9,10:A,11:b,12:C,13:d;14:E,
#define CODE_NUM 16
const volatile uint8_t  CODE[CODE_NUM]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};	
void Delayus(uint32_t us)
{
  uint32_t i,j;
	for(i=0; i<us; i++)
  {
    for(j=0; j<10; j++);
  }
}
#if defined(tm1650)



void I2C_Start(void)
{   
  I2C_SDA_H;
  I2C_SCL_H;
  Delayus(2);
  I2C_SDA_L;
  Delayus(2);
  I2C_SCL_L;
}

void I2C_Stop(void)
{    
  I2C_SCL_L;
  I2C_SDA_L;
  Delayus(2);
  I2C_SCL_H;
  Delayus(2);
  I2C_SDA_H;
  Delayus(2);
}

void I2C_Ack(void)
{    
  I2C_SCL_L;
  I2C_SDA_L;
  Delayus(2);
  I2C_SCL_H;
  Delayus(5);
  I2C_SCL_L;
}

void I2C_NAck(void)
{ 
  I2C_SCL_L;
  I2C_SDA_H;
  Delayus(2);
  I2C_SCL_H;
  Delayus(5);
  I2C_SCL_L;    
}

// 返回1 代表非应答信号
// 返回0 代表有应答信号
uint8_t I2C_Wait_Ack(void)
{
  uint16_t timeOut = 0;
  
  Delayus(2);
  I2C_SCL_H;
  Delayus(5);
  #if defined(STM32_FOC)
  while(HAL_GPIO_ReadPin(LED_SDA_GPIO_Port, LED_SDA_Pin) == GPIO_PIN_SET)
  {
    timeOut++;
    if(timeOut > 1000)
    {
      I2C_Stop();
      return 1;
    }
  }
	#elif defined(AT32_FOC)
  while(LED_SDA_IN==1)
  {
    timeOut++;
    if(timeOut > 1000)
    {
      I2C_Stop();
      return 1;
    }
  }
	#endif
	
  
  I2C_SCL_L;
  
  return 0;
}

void I2C_Send_Byte(uint8_t txd)
{
	uint8_t i;
	
  I2C_SCL_L;
  
  for(i=0; i<8; i++)
  {
    if((txd&0x80) > 0)
    {
      I2C_SDA_H;
    }
    else
    {
      I2C_SDA_L;
    }
    
    txd <<= 1;
    I2C_SCL_H;
    Delayus(2);
    I2C_SCL_L;
    Delayus(2);        
  }
}
//ack=0 发送非应答信号，代表接收完成
//ack=1 发送应答信号，代表接收未完成，继续接收
uint8_t I2C_Read_Byte(uint8_t ack)
{
  uint8_t receive = 0;
  uint8_t i;
  for(i=0; i<8; i++)
  {
    I2C_SCL_L; 
    Delayus(2);
    I2C_SCL_H;
    Delayus(2);
    
    receive <<= 1;
    #if defined(STM32_FOC)
    if(HAL_GPIO_ReadPin(LED_SDA_GPIO_Port, LED_SDA_Pin) == GPIO_PIN_SET)
    {
      receive |= 1;
    }
		#elif defined(AT32_FOC)
    if(LED_SDA_IN == 1)
    {
      receive |= 1;
    }
		#endif
		
    
    Delayus(1);
  }
  
  if(ack == 0) 
  {
    I2C_NAck();
  }
  else
  {
    I2C_Ack();
  }
  
  return receive;
}

void TM1650_Set(uint8_t addr,uint8_t data)
{
  I2C_Start();
  
  I2C_Send_Byte(addr);
  I2C_Wait_Ack();
  
  I2C_Send_Byte(data);
  I2C_Wait_Ack();
  
  I2C_Stop(); 
}

uint8_t TM1650_ReadKey(void)
{
  uint8_t key = 0;
  
  I2C_Start();
  I2C_Send_Byte(SCANKEY_CMD);
  I2C_Wait_Ack();
  
  key = I2C_Read_Byte(0);
  I2C_Stop();
  
  return key;
}

//8级亮度设置，7/8段显示，开关显示位
void TM1650_DisplaySet(uint8_t cmd)
{
  TM1650_Set(CONTROL_CMD, cmd);
}

void TM1650_GPIOInit(void)
{
	#if defined(STM32_FOC)
	
	#elif defined(AT32_FOC)
		GPIO_InitType GPIO_InitStructure;
		GPIO_StructInit(&GPIO_InitStructure);//填充默认值     
		GPIO_WriteBit(GPIO_I2C,SCL_PIN,STA_LED_INIT_STA); //初始化       //改1-2
		GPIO_InitStructure.GPIO_Pins = SCL_PIN;		                   //改2-1		 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
		GPIO_Init(GPIO_I2C, &GPIO_InitStructure);                     //改3-1
		GPIO_StructInit(&GPIO_InitStructure);//填充默认值     
		GPIO_WriteBit(GPIO_I2C,SDA_PIN,STA_LED_INIT_STA); //初始化       //改1-2
		GPIO_InitStructure.GPIO_Pins = SDA_PIN;		                   //改2-1		 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 
		GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;	
		GPIO_Init(GPIO_I2C, &GPIO_InitStructure);                     //改3-1
	#endif
	
}

void DigLED_Ctl(const uint8_t disnum,const uint8_t onoff){//数码管控制
	static uint8_t templed = 0;
//	if(App.M1.Tim.DigLEDCyc == 1){
		if(onoff == 1){//开
			templed |= disnum;
		}else{//关
			templed &= ~disnum;
		}
		TM1650_Set(DATA_ADDR4, templed);
//	}
}

void TM1650_Init(void)
{
  //开显示，1级亮度，8段显示
	TM1650_GPIOInit();
	TM1650_DisplaySet(DISPLAY_OFF);
  TM1650_DisplaySet(DISPLAY_ON);
  
  //显示0.123
  TM1650_Set(DATA_ADDR1, SEG_CODE[0]);//个位    
  TM1650_Set(DATA_ADDR2, SEG_CODE[0]);//十位
  TM1650_Set(DATA_ADDR4, 0x10);
}

void Digital_Ctl(const int16_t disnum){//数码管控制
	static int16_t olddisnum;
	if(App.M1.Tim.DigCyc == 1){
		TM1650_DisplaySet(DISPLAY_ON);
		App.M1.Tim.DigCyc = 0;
		if(App.M1.Err == 0){
			if(olddisnum > disnum){
				if(olddisnum - disnum > 1000){//速度变化小于1000 不显示
					olddisnum = disnum;
					TM1650_Set(DATA_ADDR1, SEG_CODE[(disnum/1000)%10]);//个位    
					TM1650_Set(DATA_ADDR2, SEG_CODE[(disnum/1000)/10]);//十位
				}
			}else if(olddisnum < disnum){
				if(disnum - olddisnum > 1000){
					olddisnum = disnum;
					TM1650_Set(DATA_ADDR1, SEG_CODE[(disnum/1000)%10]);//个位    
					TM1650_Set(DATA_ADDR2, SEG_CODE[(disnum/1000)/10]);//十位
				}
			}
		}else {
			TM1650_Set(DATA_ADDR1, SEG_CODE[(App.M1.Err)%10]);//个位    
			TM1650_Set(DATA_ADDR2, SEG_CODE[14]);//十位
		}
	}
}
#elif defined(tm1639)
//共阴数码管
//段码数据表
//0~F


uint8_t  Key[4]={0};	

void TM1639_Init(void)
{
 
}

void tm1639_send_8bit(unsigned char dat)	
{
  unsigned char i;
  for(i=0;i<8;i++)
  {	 
    HAL_GPIO_WritePin(I2CSCL_GPIO_Port, I2CSCL_Pin, GPIO_PIN_RESET);
    if(dat&0x01){			
			HAL_GPIO_WritePin(I2CSDA_GPIO_Port, I2CSDA_Pin, GPIO_PIN_SET);
		}else{ 
			HAL_GPIO_WritePin(I2CSDA_GPIO_Port, I2CSDA_Pin, GPIO_PIN_RESET);
		}
		Delayus(2);
		HAL_GPIO_WritePin(I2CSCL_GPIO_Port, I2CSCL_Pin, GPIO_PIN_SET);
		dat>>=1;
  }
  HAL_GPIO_WritePin(I2CSCL_GPIO_Port, I2CSCL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(I2CSDA_GPIO_Port, I2CSDA_Pin, GPIO_PIN_RESET);
}

void tm1639_send_command(unsigned char word)
{
 HAL_GPIO_WritePin(TMSTB_GPIO_Port, TMSTB_Pin, GPIO_PIN_SET);
 Delayus(1);
 HAL_GPIO_WritePin(TMSTB_GPIO_Port, TMSTB_Pin, GPIO_PIN_RESET);//在STB下降沿后由DIO输入的第一个字节作为一条指令。经过译码，取最高B7、B6两位比特位以区别不同的指令
 tm1639_send_8bit(word);
}

uint8_t Left_4bit[8]={0xC4 ,0xC2 ,0xC0 ,0xC6 ,0xCC ,0xCA ,0xC8 ,0xCE };

//数码管显示 首地址 几位 数据
// adrr 首地址 从左到右 0-7
// bit_num 显示占几个位
// num 要显示的数据

int16_t Pow_int16(int16_t x,int16_t y){
	int8_t i;
	if(y == 0){
		x = 1;
	}else if(y == 1){
		x = x;
	}else {
		for(i=1 ; i < y ; i++){
			x = x * x;
		}
	}
	
	return x;
}

void tm1639_Display_4bit(uint8_t adrr,uint8_t bit_num,uint32_t num){
	uint8_t i,temp[bit_num];
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = I2CSDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	tm1639_send_command(0x40);            //40地址自增 44固定地址
	
	if(bit_num == 1){                     //显示一位，有可能是字母
		temp[0] = num ;
	}else{
		for(int8_t i=1 ; i < bit_num ; i++){
			temp[bit_num - i] = (num % Pow_int16(10,i) ) / Pow_int16(10,i-1);
		}
		temp[0] = num / Pow_int16(10,bit_num - 1);
		//temp[ 4-3 ] = (num % Pow_int16(10, 3) ) / Pow_int16(10,3 - 1);
		temp[1] = (num % 1000)/ Pow_int16(10,2);
		temp[0] = (num  )   /1000 ;
	}

//			temp[3] = (num % 10)   /1;
//			temp[2] = (num % 100)  /10;
//			temp[1] = (num % 1000) /100;
//			temp[0] = (num  )   /1000 ;
	
	for(i=0;i<bit_num;i++){//防止查找溢出，导致程序运行出错
		if(temp[i] > CODE_NUM){
			temp[i] = CODE_NUM - 2;
		}
	}
	
	for(i=0;i<bit_num;i++){
		tm1639_send_command(Left_4bit[i+adrr]);//地址命令  C0 C1 和起来组成一位
		tm1639_send_8bit(CODE[temp[i]]&0x0F);
		tm1639_send_8bit((CODE[temp[i]]>>4)&0x0F);
	}
	tm1639_send_command(0x8A);//显示命令 开显示 调亮度	
	HAL_GPIO_WritePin(TMSTB_GPIO_Port, TMSTB_Pin, GPIO_PIN_SET);
}


//读一个字节
uint8_t read_data(void){
	uint8_t i,data=0;
	HAL_GPIO_WritePin(TMSTB_GPIO_Port, TMSTB_Pin, GPIO_PIN_RESET);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = I2CSDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	Delayus(2);
	
	for(i=0;i<8;i++){
		HAL_GPIO_WritePin(I2CSCL_GPIO_Port, I2CSCL_Pin, GPIO_PIN_RESET);
		Delayus(2);
		HAL_GPIO_WritePin(I2CSCL_GPIO_Port, I2CSCL_Pin, GPIO_PIN_SET);
		Delayus(2);
		data = data >>1;
		if(HAL_GPIO_ReadPin(I2CSDA_GPIO_Port, I2CSDA_Pin) == 0){
			data = data & 0x7f;
		}else {
			data = data | 0x80;
		}
	}
	return data;
}

uint16_t read_key(void){
	uint8_t key1,key2;
	tm1639_send_command(0x42);//数据命令 读按键命令
	Delayus(2);
	key1 = read_data();
	key2 = read_data();
	//读按键结束
	return (key1<<8)|key2;
}
#endif

void Digital_Ctl(const int16_t disnum){//数码管控制
	static int16_t olddisnum;
	static uint8_t tick=0;
	
	if(App.M1.Tim.DigCyc == 1){
		olddisnum++;
		if(olddisnum >=50){
			olddisnum =0;
			tm1639_Display_4bit(0,4,5555);
			tm1639_Display_4bit(4,4,6666);
			tick++;
			if(tick>=9){
				tick = 0;
			}
		}
	}
	
}
#endif
