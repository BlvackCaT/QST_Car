#include "nfc.h"
#include <string.h>

/* ==================== 指令数组 ==================== */
static const u8 NFC_WakeUp[] =
    {0x55,0x55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00}; // 唤醒命令
static const u8 NFC_SearchCard[] =
    {0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};                         // 寻卡命令

/* ==================== 标志位 ==================== */
static u8 NFC_WakeUp_Ok = 0;     // NFC 唤醒标志
static u8 NFC_find_Card = 0;     // 寻到卡标志
static u8 NFC_sendcmd_find = 1;  // 允许发送寻卡指令

/* ==================== 串口2 接收帧 ==================== */
typedef struct {
    u8  RxBuffer[64];
    u16 RxCounter;
} UARTFrame_TypeDef;
static UARTFrame_TypeDef UART2Frame;

// 串口2 初始化: PA2=TX, PA3=RX, 接 NFC 模块, 开启接收中断
void UART2_Init(u32 bound)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // TX PA2
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RX PA3
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   // 使能接收中断

    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
    UART2Frame.RxCounter = 0;
}

// 串口2 发送一帧
void UART2SendFrame(u8 *buf, u16 len)
{
    u16 i;
    for(i = 0; i < len; i++)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, buf[i]);
    }
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

// NFC 初始化: 唤醒模块
void NFC_Init(void)
{
    UART2SendFrame((u8*)NFC_WakeUp, sizeof(NFC_WakeUp));
    delay_ms(100);
    NFC_WakeUp_Ok = 1;   // 标记唤醒完成, 进入寻卡流程
}

// 循环寻卡(放主循环 while(1) 中)
void NFC_Handler(void)
{
    if(NFC_WakeUp_Ok)
    {
        if(NFC_find_Card == 1)   // 已寻到卡
        {
            FoundCard_Handler();
        }
        else if(NFC_find_Card == 0 && NFC_sendcmd_find == 1)
        {
            UART2Frame.RxCounter = 0;
            UART2SendFrame((u8*)NFC_SearchCard, sizeof(NFC_SearchCard)); // 发寻卡指令
            NFC_sendcmd_find = 0;
            delay_ms(200);
        }
    }
}

// 右流水灯: LED 从 1->6 再 6->1 依次点亮, 形成交替亮灭的流水效果
void R_runingled(void)
{
    u8 i, k;
    for(k = 0; k < 2; k++)              // 来回 2 趟
    {
        for(i = 1; i <= 6; i++)         // 正向 1 -> 6
        {
            R_ws2812_rgb(1, WS_DARK);
            R_ws2812_rgb(2, WS_DARK);
            R_ws2812_rgb(3, WS_DARK);
            R_ws2812_rgb(4, WS_DARK);
            R_ws2812_rgb(5, WS_DARK);
            R_ws2812_rgb(6, WS_DARK);
            R_ws2812_rgb(i, WS_WHITE);
            R_ws2812_refresh(led_num);
            delay_ms(80);
        }
        for(i = 6; i >= 1; i--)         // 反向 6 -> 1
        {
            R_ws2812_rgb(1, WS_DARK);
            R_ws2812_rgb(2, WS_DARK);
            R_ws2812_rgb(3, WS_DARK);
            R_ws2812_rgb(4, WS_DARK);
            R_ws2812_rgb(5, WS_DARK);
            R_ws2812_rgb(6, WS_DARK);
            R_ws2812_rgb(i, WS_WHITE);
            R_ws2812_refresh(led_num);
            delay_ms(80);
        }
    }
    R_led_CLC();                        // 结束全部熄灭
}

// 找到卡后: 播放流水灯动画 + 继续寻卡
void FoundCard_Handler(void)
{
    NFC_find_Card = 0;   // 清除标识
    R_runingled();       // 播放流水灯(交替亮灭)
    NFC_sendcmd_find = 1; // 允许再次发送寻卡指令
    delay_ms(200);
}

// 串口2 接收中断: 判断帧并校验卡片
void USART2_IRQHandler(void)
{
    u8 i;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)   // 接收中断
    {
        UART2Frame.RxBuffer[UART2Frame.RxCounter++] = USART_ReceiveData(USART2);

        if(NFC_WakeUp_Ok == 0)   // 未唤醒: 唤醒应答
        {
            if(UART2Frame.RxCounter >= 15)
                UART2Frame.RxCounter = 0;
        }
        else   // 已唤醒: 寻卡应答
        {
            if(UART2Frame.RxCounter >= 25)
            {
                // 通过串口1(printf)输出到电脑串口助手
                for(i = 0; i < 25; i++)
                    printf("%02X ", UART2Frame.RxBuffer[i]);
                printf("\r\n");

                // 校验三张卡的 ID(字节 19~22)
                if(((0xAC==UART2Frame.RxBuffer[19])&&(0x4E==UART2Frame.RxBuffer[20])&&(0x42==UART2Frame.RxBuffer[21])&&(0x06==UART2Frame.RxBuffer[22]))
                || ((0x50==UART2Frame.RxBuffer[19])&&(0x84==UART2Frame.RxBuffer[20])&&(0xFC==UART2Frame.RxBuffer[21])&&(0x23==UART2Frame.RxBuffer[22]))
                || ((0x40==UART2Frame.RxBuffer[19])&&(0x74==UART2Frame.RxBuffer[20])&&(0x80==UART2Frame.RxBuffer[21])&&(0x23==UART2Frame.RxBuffer[22])))
                {
                    NFC_find_Card = 1;
                }
                UART2Frame.RxCounter = 0;
            }
        }
    }
}
