#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_watchdog.h"
#include "wifiiot_uart.h"
#include "hi_io.h"
#include "hi_time.h"
#include "hi_uart.h"
#include "wifiiot_errno.h"

/* ========== 硬件引脚定义 ========== */
#define TRIG_PIN        7       // HC-SR04 Trig
#define ECHO_PIN        8       // HC-SR04 Echo
#define SERVO_PIN       2       // SG90 舵机PWM
#define GPIO_FUNC       0

/* ========== 避障参数 ========== */
#define OBSTACLE_THRESHOLD  20.0f
#define BACKWARD_TIME       8
#define TURN_TIME           10
#define SCAN_TURN_TIME      6
#define TICK_MS             100

/* ========== 舵机角度对应的占空比(us) ========== */
#define SERVO_LEFT          1000    // 左  ~45°
#define SERVO_CENTER        1500    // 中  ~90°
#define SERVO_RIGHT         2000    // 右  ~135°

/* ========== 电机控制协议（与13.0一致） ========== */
static uint8_t uart_sendbuf[20];

void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;

    if (motorA < 0) { A_dir = 1; motorA = -motorA; }
    if (motorB < 0) { B_dir = 1; motorB = -motorB; }
    if (motorA > 150) motorA = 150;
    if (motorB > 150) motorB = 150;

    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = A_dir;
    uart_sendbuf[2] = motorA;
    uart_sendbuf[3] = B_dir;
    uart_sendbuf[4] = motorB;
    uart_sendbuf[5] = 0xFD;
    printf("[UART] FC %02X %02X %02X %02X FD\r\n",
           A_dir, motorA, B_dir, motorB);
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

void car_forward(void)  { stm32motor_control(60, 60); }
void car_backward(void) { stm32motor_control(-80, -80); }
void car_left(void)     { stm32motor_control(-50, 150); }
void car_right(void)    { stm32motor_control(150, -50); }
void car_stop(void)     { stm32motor_control(0, 0); }

/* ========== HC-SR04 超声波测距 ========== */
float GetDistance(void)
{
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    unsigned int timeout = 0;

    hi_io_set_func(ECHO_PIN, GPIO_FUNC);
    GpioSetDir(ECHO_PIN, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(TRIG_PIN, WIFI_IOT_GPIO_DIR_OUT);

    GpioSetOutputVal(TRIG_PIN, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(TRIG_PIN, WIFI_IOT_GPIO_VALUE0);

    while (1) {
        GpioGetInputVal(ECHO_PIN, &value);
        if (value == WIFI_IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }
        timeout++;
        if (timeout > 50000) {
            return -1.0;
        }
    }

    distance = time * 0.034 / 2;
    return distance;
}

/* ========== SG90 舵机控制 ========== */
void ServoSetAngle(unsigned int duty)
{
    for (int i = 0; i < 10; i++) {
        GpioSetOutputVal(SERVO_PIN, WIFI_IOT_GPIO_VALUE1);
        hi_udelay(duty);
        GpioSetOutputVal(SERVO_PIN, WIFI_IOT_GPIO_VALUE0);
        hi_udelay(20000 - duty);
    }
}

/* ========== 舵机扫描：寻找最佳方向 ========== */
/* 返回: 0=左转, 1=右转, 2=后退(全部堵死) */
static int ScanBestDirection(void)
{
    float dist_left, dist_center, dist_right;

    /* 扫描左侧 */
    ServoSetAngle(SERVO_LEFT);
    usleep(400 * 1000);
    dist_left = GetDistance();
    printf("[SCAN] Left  (45°):  %.1f cm\r\n", dist_left);

    /* 扫描中央 */
    ServoSetAngle(SERVO_CENTER);
    usleep(400 * 1000);
    dist_center = GetDistance();
    printf("[SCAN] Center(90°): %.1f cm\r\n", dist_center);

    /* 扫描右侧 */
    ServoSetAngle(SERVO_RIGHT);
    usleep(400 * 1000);
    dist_right = GetDistance();
    printf("[SCAN] Right (135°): %.1f cm\r\n", dist_right);

    /* 舵机回中 */
    ServoSetAngle(SERVO_CENTER);

    /* 判断最佳方向 */
    if (dist_left < 0) dist_left = 0;
    if (dist_center < 0) dist_center = 0;
    if (dist_right < 0) dist_right = 0;

    if (dist_left >= dist_right && dist_left > dist_center) {
        printf("[SCAN] => Best: LEFT (%.1f cm)\r\n", dist_left);
        return 0;
    } else if (dist_right > dist_left && dist_right > dist_center) {
        printf("[SCAN] => Best: RIGHT (%.1f cm)\r\n", dist_right);
        return 1;
    } else {
        printf("[SCAN] => All blocked, back up (L=%.1f C=%.1f R=%.1f)\r\n",
               dist_left, dist_center, dist_right);
        return 2;
    }
}

/* ========== 自动避障主任务（舵机扫描版） ========== */
static void auto_avoid_thread(void *arg)
{
    (void)arg;
    float distance = 0.0;
    int loop_count = 0;
    int best_dir;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  Auto Avoidance (Servo Scanning)\r\n");
    printf("========================================\r\n");
    printf("[HW] HC-SR04: Trig=GPIO7 Echo=GPIO8\r\n");
    printf("[HW] SG90   : PWM=GPIO2\r\n");
    printf("[HW] STM32  : UART2 (GPIO11/12, 115200)\r\n");
    printf("[CFG] Threshold=%.0fcm, Speed=60\r\n", OBSTACLE_THRESHOLD);
    printf("========================================\r\n");

    /* 舵机归中 */
    printf("[INIT] Servo centering...\r\n");
    ServoSetAngle(SERVO_CENTER);
    usleep(200 * 1000);
    printf("[INIT] Servo ready.\r\n");

    /* UART自检 */
    printf("[TEST] UART self-test...\r\n");
    car_stop();
    usleep(100 * 1000);
    printf("[TEST] UART OK.\r\n");

    /* 测距自检 */
    printf("[TEST] HC-SR04 self-test...\r\n");
    distance = GetDistance();
    if (distance < 0) {
        printf("[TEST] WARNING: HC-SR04 no response!\r\n");
    } else {
        printf("[TEST] distance = %.1f cm\r\n", distance);
    }

    printf("========================================\r\n");
    printf("[READY] Moving forward.\r\n");
    printf("========================================\r\n\r\n");

    while (1) {
        loop_count++;
        distance = GetDistance();

        if (distance < 0) {
            if (loop_count % 10 == 0) {
                printf("[WARN] Sensor timeout, keep forward...\r\n");
            }
            car_forward();
        } else if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
            printf("\r\n[!!!] OBSTACLE at %.1f cm!\r\n", distance);

            /* 第1步：停车 */
            printf("[AVOID-1/5] Stop.\r\n");
            car_stop();
            usleep(300 * 1000);

            /* 第2步：后退留出扫描空间 */
            printf("[AVOID-2/5] Back up.\r\n");
            for (int i = 0; i < BACKWARD_TIME; i++) {
                car_backward();
                usleep(TICK_MS * 1000);
            }
            car_stop();
            usleep(200 * 1000);

            /* 第3步：舵机扫描三个方向 */
            printf("[AVOID-3/5] Scanning directions...\r\n");
            best_dir = ScanBestDirection();

            /* 第4步：根据扫描结果转向 */
            printf("[AVOID-4/5] Turn to best direction...\r\n");
            if (best_dir == 0) {
                for (int i = 0; i < SCAN_TURN_TIME; i++) {
                    car_left();
                    usleep(TICK_MS * 1000);
                }
            } else if (best_dir == 1) {
                for (int i = 0; i < SCAN_TURN_TIME; i++) {
                    car_right();
                    usleep(TICK_MS * 1000);
                }
            } else {
                printf("[AVOID] All blocked! Back up more...\r\n");
                for (int i = 0; i < BACKWARD_TIME; i++) {
                    car_backward();
                    usleep(TICK_MS * 1000);
                }
                for (int i = 0; i < SCAN_TURN_TIME + 3; i++) {
                    car_right();
                    usleep(TICK_MS * 1000);
                }
            }

            /* 第5步：恢复前进 */
            printf("[AVOID-5/5] Resume forward.\r\n\r\n");
            car_stop();
            usleep(200 * 1000);
        } else {
            if (loop_count % 5 == 0) {
                printf("[OK] loop=%d, dist=%.1f cm\r\n", loop_count, distance);
            }
            car_forward();
        }

        usleep(TICK_MS * 1000);
    }
}

/* ========== 任务入口 ========== */
static void AutoAvoid(void)
{
    printf("\r\n[BOOT] AutoAvoid (Servo Scan) loading...\r\n");

    WatchDogDisable();
    printf("[BOOT] Watchdog disabled.\r\n");

    GpioInit();
    printf("[BOOT] GPIO initialized.\r\n");

    /* UART2初始化 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    printf("[BOOT] UART2 pins configured.\r\n");

    WifiIotUartAttribute uart_attr2 = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);
    printf("[BOOT] UART2 initialized (115200 8N1).\r\n");

    /* 验证UART2：发送测试帧，串口助手上应能看到 [UART] 输出 */
    printf("[BOOT] UART2 test: sending stop command...\r\n");
    car_stop();
    usleep(50 * 1000);
    car_forward();
    usleep(50 * 1000);
    car_stop();
    printf("[BOOT] UART2 test done. Check [UART] lines above.\r\n");

    /* 舵机初始化 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(SERVO_PIN, WIFI_IOT_GPIO_DIR_OUT);
    printf("[BOOT] Servo GPIO2 configured.\r\n");

    osThreadAttr_t attr;
    attr.name = "AutoAvoid";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)auto_avoid_thread, NULL, &attr) == NULL) {
        printf("[BOOT] ERROR: Failed to create task!\r\n");
    } else {
        printf("[BOOT] Task created.\r\n");
    }
}

APP_FEATURE_INIT(AutoAvoid);