#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "nfc.h"

int main(void)
{
    Stm32_Clock_Init(9);              // 系统时钟初始化 72MHz
    MY_NVIC_PriorityGroupConfig(2);   // 中断优先级分组
    delay_init();                     // 延时初始化
    uart_init(115200);                // 串口1: printf / 电脑串口助手
    JTAG_Set(JTAG_SWD_DISABLE);       // 关闭JTAG
    JTAG_Set(SWD_ENABLE);             // 打开SWD(保留下载口)
    colorful_led_Init();              // 彩灯初始化
    UART2_Init(115200);               // 串口2: NFC模块
    NFC_Init();                       // 唤醒NFC模块

    printf("NFC Ready\r\n");

    while(1)
      {
          NFC_Handler();

          delay_ms(500);                                       // 每0.5秒发一次
          while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
          USART_SendData(USART1, 'A');                         // 直接发字母 A
      }
}
