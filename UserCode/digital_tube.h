#ifndef __TM1650_H__
#define __TM1650_H__
#include  "userpara.h"
#if defined (STM32_FOC)
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#elif defined(AT32_FOC)
#include "at32f4xx.h"
#endif
#include "gpio.h"
#ifdef use_digital_tube
#if defined(tm1650)
//控制命令，显示设置，一个字节
//1Byte：B7：X，B6B5B4：亮度等级，B3：8段显示0，7段显示1，B2B1：XX，B0：关显示0，开显示1
#define CONTROL_CMD  0x48
#define SCANKEY_CMD  0x49

//开显示，1级亮度，8段显示
#define DISPLAY_ON  0x51
#define DISPLAY_OFF 0x10

//显存地址
//#define DATA_ADDR1 0x68
//#define DATA_ADDR2 0x6A
//#define DATA_ADDR3 0x6C
//#define DATA_ADDR4 0x6E

#define DATA_ADDR1 0x6A
#define DATA_ADDR2 0x68
#define DATA_ADDR3 0x6C
#define DATA_ADDR4 0x6E

void TM1650_Init(void);
uint8_t TM1650_ReadKey(void);
void TM1650_Set(uint8_t addr,uint8_t data);
void TM1650_DisplaySet(uint8_t cmd);

#elif defined(tm1639)
void TM1639_Init(void);
void tm1639_send_8bit(unsigned char dat);
void tm1639_send_command(unsigned char word);
void tm1639_Display_4bit(uint8_t adrr,uint8_t bit_num,uint32_t num);
uint16_t read_key(void);
#endif
//接口定义
#if defined (STM32_FOC)
	#define SCL_PIN			GPIO_PIN_10
	#define SDA_PIN			GPIO_PIN_11
	
	#define STB_PIN     GPIO_PIN_5
	
	#define GPIO_I2C		GPIOB

	#define LED_SDA_GPIO_Port		GPIO_I2C
	#define LED_SDA_Pin					SDA_PIN


	#define I2C_SCL_L (GPIO_I2C->BSRR = (uint32_t)SCL_PIN << 16U) 
	#define I2C_SCL_H (GPIO_I2C->BSRR = SCL_PIN)

	#define I2C_SDA_L (GPIO_I2C->BSRR = (uint32_t)SDA_PIN << 16U) 
	#define I2C_SDA_H (GPIO_I2C->BSRR = SDA_PIN)
#elif defined(AT32_FOC)
	#define SCL_PIN			GPIO_Pins_10
	#define SDA_PIN			GPIO_Pins_11
	#define GPIO_I2C		GPIOB

	#define LED_SDA_GPIO_Port		GPIO_I2C
	#define LED_SDA_Pin					SDA_PIN
	#define LED_SDA_IN          PBin(11)

	#define I2C_SCL_L PBout(10)=0
	#define I2C_SCL_H PBout(10)=1

	#define I2C_SDA_L PBout(11)=0
	#define I2C_SDA_H PBout(11)=1
#endif

void Digital_Ctl(const int16_t disnum);
void DigLED_Ctl(const uint8_t disnum,const uint8_t onoff);

#endif
#endif
