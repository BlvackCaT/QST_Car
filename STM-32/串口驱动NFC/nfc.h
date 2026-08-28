#ifndef __NFC_H
#define __NFC_H
#include "sys.h"

void UART2_Init(u32 bound);           // 串口2(NFC)初始化
void UART2SendFrame(u8 *buf, u16 len); // 串口2发送一帧
void NFC_Init(void);                  // 唤醒NFC模块
void NFC_Handler(void);               // 循环寻卡(放主循环)
void FoundCard_Handler(void);         // 找到卡后的用户函数
void USART2_IRQHandler(void);         // 串口2接收中断

#endif
