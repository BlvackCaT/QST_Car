#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h"

/* 编码器定时器自动重装值（16位定时器满量程） */
#define ENCODER_TIM_PERIOD 0xFFFF

void Encoder_Init_TIM2(void);  /* 初始化TIM2为编码器接口模式 */
void Encoder_Init_TIM3(void);  /* 初始化TIM3为编码器接口模式 */
int  Read_Encoder(u8 TIMX);    /* 读取编码器计数值（速度） */

#endif
