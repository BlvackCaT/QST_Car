#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_pwm.h"
#include "hi_pwm.h"
#include "hi_uart.h"
#include "wifiiot_gpio_ex.h"

static void motion_thread(void);

uint8_t uart_sendbuf[20];

/***通信协议***/
/*
函数功能：发送至stm32的数据协议
参数    ：电机实际转速的一百倍。例如：设置转速为1rad/s，则传入100
*/
void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;

    //小车运动方向 前进（正转）：0  后退（反转） 1
    if(motorA<0){
        A_dir=1;
        motorA = -motorA;
    }else{
        A_dir=0;
    }

    if(motorB<0){
        B_dir=1;
        motorB = -motorB;
    }else{
        B_dir=0;
    }

    //限制幅度 -150 ~150
    if (motorA > 150)
    {
        motorA = 150;
    }
    if (motorB > 150)
    {
        motorB = 150;
    }

    //数据协议
    uart_sendbuf[0] = 0xFC;   // 帧头
    uart_sendbuf[1] = A_dir;  // 左轮方向   0正转，1反转
    uart_sendbuf[2] = motorA; // 左轮速度
    uart_sendbuf[3] = B_dir;  // 右轮方向   0正转，1反转
    uart_sendbuf[4] = motorB; // 右轮速度
    uart_sendbuf[5] = 0xFD;   // 帧尾
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

// 边缘停止指令（DirA=2, DirB=2 触发STM32红灯+停车）
void car_edge_stop(void)
{
    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = 2;       // DirA=2
    uart_sendbuf[2] = 0;
    uart_sendbuf[3] = 2;       // DirB=2
    uart_sendbuf[4] = 0;
    uart_sendbuf[5] = 0xFD;
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

// 小车后退
void car_backward(void)
{
    stm32motor_control(-80, -80);
}

// 小车前进
void car_forward(void)
{
    stm32motor_control(100, 100);
}

// 小车左转
void car_left(void)
{
    stm32motor_control(50, 150);
}

// 小车右转
void car_right(void)
{
    stm32motor_control(150, 50);
}

// 小车停止
void car_stop(void)
{
    stm32motor_control(0, 0);
}

/***** TCRT5000 边缘传感器 *****/
/* GPIO13=左路, GPIO14=右路 */
/* 桌面上(有反射)=0, 悬空(无反射)=1 */

static void TCRT_Init(void)
{
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_GPIO_DIR_IN);
}

/* 返回1=检测到边缘, 0=安全 */
static uint8_t TCRT_Edge(void)
{
    WifiIotGpioValue left, right;
    GpioGetInputVal(WIFI_IOT_IO_NAME_GPIO_13, &left);
    GpioGetInputVal(WIFI_IOT_IO_NAME_GPIO_14, &right);
    /* 任意一个为1即边缘（悬空无反射） */
    return (left == WIFI_IOT_GPIO_VALUE1 || right == WIFI_IOT_GPIO_VALUE1);
}

/* 带消抖的边缘检测，连续3次确认 */
static uint8_t Edge_Detected(void)
{
    static uint8_t debounce = 0;

    if (TCRT_Edge()) {
        if (++debounce >= 3) {
            debounce = 3;
            return 1;
        }
    } else {
        debounce = 0;
    }
    return 0;
}

/***** 运动任务：Hi3861作为大脑 *****/
/*
 * 持续前进，TCRT5000检测边缘。
 * 检测到边缘 → 边缘停止指令(红灯) → 后退 → 转向 → 恢复前进
 */
static void motion_thread(void)
{
    const int tick_ms = 100;

    printf("\r\n===== Edge Detection Car =====\r\n");
    printf("Brain : Hi3861 (TCRT5000 GPIO13/14)\r\n");
    printf("Muscle: STM32  (Motor + LED)\r\n");
    printf("==============================\r\n\r\n");

    while (1)
    {
        /* 持续前进 */
        car_forward();
        usleep(tick_ms * 1000);

        /* 检测边缘 */
        if (Edge_Detected())
        {
            printf("[EDGE] Detected! Stopping...\r\n");

            /* 发送边缘停止 → STM32停车+12红灯 */
            for (int i = 0; i < 30; i++) {
                car_edge_stop();
                usleep(tick_ms * 1000);
            }

            /* 后退远离边缘 */
            printf("[EDGE] Backing up...\r\n");
            for (int i = 0; i < 15; i++) {
                car_backward();
                usleep(tick_ms * 1000);
            }

            /* 左转调头 */
            printf("[EDGE] Turning...\r\n");
            for (int i = 0; i < 10; i++) {
                car_left();
                usleep(tick_ms * 1000);
            }

            printf("[EDGE] Resuming forward...\r\n");
        }
    }
}

/*****任务创建*****/
static void correspondence(void)
{
    GpioInit(); // GPIO功能初始化

    /****************通讯串口初始化******************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD); // GPIO_11复用为UART2_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD); // GPIO_12复用为UART2_RXD

    /****************串口参数***********************/
    WifiIotUartAttribute uart_attr2 = {
        // 波特率：115200
        .baudRate = 115200,
        // 数据位：8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);

    /****************边缘传感器初始化******************/
    TCRT_Init();

    osThreadAttr_t attr;
    attr.attr_bits = 0U;      // 设置osThreadJoin是否可以使用
    attr.cb_mem = NULL;       // 控制块指针设置
    attr.cb_size = 0U;        // 控制块指针大小
    attr.stack_mem = NULL;    // 任务栈设置
    attr.stack_size = 1024 * 4; // 任务栈大小
    attr.name = "motion_thread"; // 创建任务名称
    attr.priority = 25;    // 任务优先级
    if (osThreadNew((osThreadFunc_t)motion_thread, NULL, &attr) == NULL)
    {
        printf("Failed to create motion_thread!\n");
    }
}

APP_FEATURE_INIT(correspondence); // 启动任务