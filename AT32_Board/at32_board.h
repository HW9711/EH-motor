/**
  **************************************************************************
  * File   : at32_board.h
  * Version: V1.2.2
  * Date   : 2020-07-01
  * Brief  : Header file for AT-START board
  *          1. Set of firmware functions to manage Leds, push-button and COM ports.
  *          2. initialize Delay Function and USB
  */

#ifndef __AT32_BOARD_H
#define __AT32_BOARD_H	 
#include <at32f4xx.h>




/*Delay function*/
void Delay_init(void);
void Delay_us(u32 nus);
void Delay_ms(u16 nms);
void Delay_sec(u16 sec);
#endif

/****************** (C) COPYRIGHT 2018 ArteryTek *********END OF FILE*********/
