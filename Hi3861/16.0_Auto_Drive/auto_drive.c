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
#define TRIG_PIN            7       // HC-SR04 Trig
#define ECHO_PIN            8       // HC-SR04 Echo
#define SERVO_PIN           2       // SG90 舵机PWM
#define TCRT_LEFT_PIN       WIFI_IOT_IO_NAME_GPIO_13  // TCRT5000 左
#define TCRT_RIGHT_PIN      WIFI_IOT_IO_NAME_GPIO_14  // TCRT5000 右
#define GPIO_FUNC           0

/* ========== 避障参数 ========== */
#define OBSTACLE_THRESHOLD  16.0f   // 障碍物阈值(cm)
#define SIDE_CHECK_DIST     10.0f   // 侧面检测阈值(cm)
#define POST_TURN_CHECK     20.0f   // 转向后验证阈值(cm)
#define FORWARD_SPEED       65      // 前进速度
#define BACKWARD_SPEED      40      // 后退速度
#define BACKWARD_TIME       5       // 超声波避障后退时间(tick)
#define SCAN_TURN_TIME      8       // 扫描后转向时间(tick)
#define EDGE_BACKWARD_TIME  5       // 黑线后退时间(tick)
#define EDGE_TURN_TIME      12      // 黑线转向时间(tick)
#define TICK_MS             80      // 每tick毫秒数
#define SIDE_SCAN_INTERVAL  37      // 侧面扫描间隔(~3秒: 37*80ms)

/* ========== 舵机角度(us) ========== */
#define SERVO_LEFT          1000    // 左  ~45°
#define SERVO_CENTER        1500    // 中  ~90°
#define SERVO_RIGHT         2000    // 右  ~135°

/* ========== 电机控制协议 ========== */
static uint8_t uart_sendbuf[6];

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
    uart_sendbuf[2] = (uint8_t)motorA;
    uart_sendbuf[3] = B_dir;
    uart_sendbuf[4] = (uint8_t)motorB;
    uart_sendbuf[5] = 0xFD;
    UartWrite(WIFI_IOT_UART_IDX_2, uart_sendbuf, 6);
}

void car_edge_stop(void)
{
    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = 2;
    uart_sendbuf[2] = 0;
    uart_sendbuf[3] = 2;
    uart_sendbuf[4] = 0;
    uart_sendbuf[5] = 0xFD;
    UartWrite(WIFI_IOT_UART_IDX_2, uart_sendbuf, 6);
}

void car_forward(void)  { stm32motor_control(FORWARD_SPEED, FORWARD_SPEED); }
void car_backward(void) { stm32motor_control(-BACKWARD_SPEED, -BACKWARD_SPEED); }
void car_left(void)     { stm32motor_control(-30, 80); }
void car_right(void)    { stm32motor_control(80, -30); }
void car_stop(void)     { stm32motor_control(0, 0); }

/* ========== HC-SR04 超声波测距 ========== */
static void HC_SR04_Init(void)
{
    /* 显式初始化Trig和Echo引脚，避免复位后引脚状态残留 */
    hi_io_set_func(TRIG_PIN, GPIO_FUNC);
    hi_io_set_func(ECHO_PIN, GPIO_FUNC);
    GpioSetDir(TRIG_PIN, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(ECHO_PIN, WIFI_IOT_GPIO_DIR_IN);
    GpioSetOutputVal(TRIG_PIN, WIFI_IOT_GPIO_VALUE0);
    /* 等待传感器稳定 */
    usleep(300 * 1000);
}

float GetDistance(void)
{
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    unsigned int timeout = 0;

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
        if (timeout > 50000) return -1.0;
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
static int ScanBestDirection(void)
{
    float dist_left, dist_center, dist_right;

    ServoSetAngle(SERVO_LEFT);
    usleep(400 * 1000);
    dist_left = GetDistance();
    printf("[SCAN] Left  (45°):  %.1f cm\r\n", dist_left);

    ServoSetAngle(SERVO_CENTER);
    usleep(400 * 1000);
    dist_center = GetDistance();
    printf("[SCAN] Center(90°): %.1f cm\r\n", dist_center);

    ServoSetAngle(SERVO_RIGHT);
    usleep(400 * 1000);
    dist_right = GetDistance();
    printf("[SCAN] Right (135°): %.1f cm\r\n", dist_right);

    ServoSetAngle(SERVO_CENTER);

    if (dist_left < 0) dist_left = 0;
    if (dist_center < 0) dist_center = 0;
    if (dist_right < 0) dist_right = 0;

    /*
     * 只有左右明显更通畅才转弯，否则（中方向最好或持平）直接前进。
     * 避免"全通畅→前进→又遇到同一障碍物"的死循环：
     * 如果中方向还被挡着，左右必然有一个更通畅，会正常转弯绕开。
     */
    if (dist_left > dist_center && dist_left >= dist_right) {
        printf("[SCAN] => Best: LEFT (L=%.1f C=%.1f R=%.1f)\r\n",
               dist_left, dist_center, dist_right);
        return 0;
    } else if (dist_right > dist_center && dist_right > dist_left) {
        printf("[SCAN] => Best: RIGHT (L=%.1f C=%.1f R=%.1f)\r\n",
               dist_left, dist_center, dist_right);
        return 1;
    } else {
        /* 中方向最好，或三方向都差不多（包括全堵死）→ 前进 */
        printf("[SCAN] => Center is best, go forward (L=%.1f C=%.1f R=%.1f)\r\n",
               dist_left, dist_center, dist_right);
        return 3;
    }
}

/* ========== 侧面快速扫描：检测斜向贴墙 ========== */
static void SideScan(void)
{
    float dist_left, dist_right;

    ServoSetAngle(SERVO_LEFT);
    usleep(200 * 1000);
    dist_left = GetDistance();

    ServoSetAngle(SERVO_RIGHT);
    usleep(200 * 1000);
    dist_right = GetDistance();

    ServoSetAngle(SERVO_CENTER);

    printf("[SIDE] Left=%.1fcm Right=%.1fcm\r\n", dist_left, dist_right);

    if (dist_left > 0 && dist_left < SIDE_CHECK_DIST) {
        printf("[SIDE] Wall on LEFT! Turn right slightly...\r\n");
        for (int i = 0; i < 3; i++) {
            car_right();
            usleep(TICK_MS * 1000);
        }
    } else if (dist_right > 0 && dist_right < SIDE_CHECK_DIST) {
        printf("[SIDE] Wall on RIGHT! Turn left slightly...\r\n");
        for (int i = 0; i < 3; i++) {
            car_left();
            usleep(TICK_MS * 1000);
        }
    }
}

/* ========== TCRT5000 红外黑线检测 ========== */
static uint8_t tcrt_left_val = 0;
static uint8_t tcrt_right_val = 0;

static void TCRT_Init(void)
{
    IoSetFunc(TCRT_LEFT_PIN, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(TCRT_RIGHT_PIN, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(TCRT_LEFT_PIN, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(TCRT_RIGHT_PIN, WIFI_IOT_GPIO_DIR_IN);
}

static void TCRT_ReadBoth(void)
{
    WifiIotGpioValue l, r;
    GpioGetInputVal(TCRT_LEFT_PIN, &l);
    GpioGetInputVal(TCRT_RIGHT_PIN, &r);
    tcrt_left_val = (l == WIFI_IOT_GPIO_VALUE1) ? 1 : 0;
    tcrt_right_val = (r == WIFI_IOT_GPIO_VALUE1) ? 1 : 0;
}

static uint8_t TCRT_Triggered(void)
{
    return (tcrt_left_val || tcrt_right_val);
}

/*
 * 边缘/黑线检测：单次触发即响应（更快检测黑线）
 * 如果误触发太多，把 >= 1 改回 >= 2
 * 注意：如果检测不到黑线但能检测到桌面边缘，
 * 需要调节TCRT5000模块上的电位器，降低比较器阈值
 */
static uint8_t Edge_Detected(void)
{
    static uint8_t debounce = 0;

    TCRT_ReadBoth();

    if (TCRT_Triggered()) {
        if (++debounce >= 1) {
            debounce = 1;
            return 1;
        }
    } else {
        debounce = 0;
    }
    return 0;
}

/* ========== 自动避障+避黑线主任务 ========== */
static void auto_drive_thread(void *arg)
{
    (void)arg;
    float distance = 0.0;
    int loop_count = 0;
    int best_dir;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  Auto Drive v2.2 - Avoid + Edge Stop  \r\n");
    printf("========================================\r\n");
    printf("[HW] HC-SR04 : Trig=GPIO7 Echo=GPIO8\r\n");
    printf("[HW] SG90    : PWM=GPIO2\r\n");
    printf("[HW] TCRT5000: Left=GPIO13 Right=GPIO14\r\n");
    printf("[HW] STM32   : UART2 (GPIO11/12, 115200)\r\n");
    printf("[CFG] Obstacle=%.0fcm, SideCheck=%.0fcm, Speed=%d\r\n",
           OBSTACLE_THRESHOLD, SIDE_CHECK_DIST, FORWARD_SPEED);
    printf("[TIP] If black line not detected, adjust TCRT5000 potentiometer!\r\n");
    printf("========================================\r\n");

    /* 舵机归中 */
    printf("[INIT] Servo centering...\r\n");
    ServoSetAngle(SERVO_CENTER);
    usleep(200 * 1000);
    printf("[INIT] Servo ready.\r\n");

    /* TCRT5000初始化 */
    TCRT_Init();
    printf("[INIT] TCRT5000 sensors ready.\r\n");

    /* HC-SR04初始化：显式配置引脚 + 稳定延迟 */
    HC_SR04_Init();
    printf("[INIT] HC-SR04 initialized.\r\n");

    /* UART同步：发送多次停止帧，确保STM32接收正确 */
    printf("[TEST] UART sync with STM32...\r\n");
    for (int i = 0; i < 5; i++) {
        car_stop();
        usleep(50 * 1000);
    }
    printf("[TEST] UART OK.\r\n");

    /* 超声波自检：丢弃前3次读数，取第4次 */
    printf("[TEST] HC-SR04 warm-up...\r\n");
    for (int i = 0; i < 3; i++) {
        GetDistance();
        usleep(100 * 1000);
    }
    distance = GetDistance();
    if (distance < 0) {
        printf("[TEST] WARNING: HC-SR04 no response!\r\n");
    } else {
        printf("[TEST] Distance = %.1f cm\r\n", distance);
    }

    /* TCRT5000自检 */
    printf("[TEST] TCRT5000 self-test (5 readings)...\r\n");
    for (int i = 0; i < 5; i++) {
        TCRT_ReadBoth();
        printf("[TEST]   Reading %d: Left=%d Right=%d  -> %s\r\n",
               i + 1, tcrt_left_val, tcrt_right_val,
               TCRT_Triggered() ? "BLACK/EDGE!" : "WHITE/SAFE");
        usleep(200 * 1000);
    }
    printf("[TEST] TCRT self-test done.\r\n");

    printf("========================================\r\n");
    printf("[READY] Auto drive started. Moving forward...\r\n");
    printf("========================================\r\n\r\n");

    while (1) {
        loop_count++;
        TCRT_ReadBoth();

        /* ===== 优先级1：黑线/边缘检测 ===== */
        if (Edge_Detected()) {
            printf("\r\n========================================\r\n");
            printf("[BLACK LINE!] TCRT Left=%d Right=%d\r\n",
                   tcrt_left_val, tcrt_right_val);
            printf("========================================\r\n");

            /* 红灯闪一下 + 立刻后退 */
            printf("[EDGE-1/3] RED LED + Back up (%dx%dms)...\r\n",
                   EDGE_BACKWARD_TIME, TICK_MS);
            car_edge_stop();
            usleep(TICK_MS * 1000);
            for (int i = 0; i < EDGE_BACKWARD_TIME; i++) {
                car_backward();
                usleep(TICK_MS * 1000);
            }

            printf("[EDGE-2/3] Turn left (%dx%dms)...\r\n",
                   EDGE_TURN_TIME, TICK_MS);
            for (int i = 0; i < EDGE_TURN_TIME; i++) {
                car_left();
                usleep(TICK_MS * 1000);
            }

            printf("[EDGE-3/3] Resume forward.\r\n");
            printf("========================================\r\n\r\n");
            car_stop();
            usleep(200 * 1000);
            continue;
        }

        /* ===== 优先级2：超声波障碍物检测 ===== */
        distance = GetDistance();

        if (distance < 0) {
            if (loop_count % 10 == 0) {
                printf("[WARN] HC-SR04 timeout! Keep forward.\r\n");
            }
            car_forward();
        } else if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
            printf("\r\n========================================\r\n");
            printf("[OBSTACLE!] Distance = %.1f cm (< %.0f cm)\r\n",
                   distance, OBSTACLE_THRESHOLD);
            printf("========================================\r\n");

            /* 红灯闪一下 + 立刻后退 */
            printf("[AVOID-1/5] RED LED + Back up (%dx%dms)...\r\n",
                   BACKWARD_TIME, TICK_MS);
            car_edge_stop();
            usleep(TICK_MS * 1000);
            for (int i = 0; i < BACKWARD_TIME; i++) {
                car_backward();
                usleep(TICK_MS * 1000);
            }
            car_stop();
            usleep(200 * 1000);

            /* 舵机扫描 */
            printf("[AVOID-2/5] Scanning directions...\r\n");
            best_dir = ScanBestDirection();

            /* 转向 */
            printf("[AVOID-3/5] Turn to best direction (%dx%dms)...\r\n",
                   SCAN_TURN_TIME, TICK_MS);
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
                /* best_dir==3: 中方向最好，直接前进，不转弯 */
                printf("[AVOID] Center is best, go straight.\r\n");
            }

            /* 转向后验证：检查前方是否真的畅通 */
            printf("[AVOID-4/5] Verify path clear...\r\n");
            car_stop();
            usleep(200 * 1000);
            distance = GetDistance();
            printf("[AVOID] Post-turn distance = %.1f cm\r\n", distance);
            if (distance > 0 && distance < POST_TURN_CHECK) {
                printf("[AVOID] Still too close! Turn more...\r\n");
                for (int i = 0; i < 4; i++) {
                    if (best_dir == 0) car_left();
                    else car_right();
                    usleep(TICK_MS * 1000);
                }
            }

            /* 恢复前进 */
            printf("[AVOID-5/5] Resume forward.\r\n");
            printf("========================================\r\n\r\n");
            car_stop();
            usleep(200 * 1000);
        } else {
            /* 正常前进 */
            car_forward();

            /* 定期侧面扫描：检测斜向贴墙 */
            if (loop_count % SIDE_SCAN_INTERVAL == 0) {
                SideScan();
            }
        }

        /* 每循环打印红外状态 */
        printf("[IR] L=%d R=%d | Dis=%.1fcm | %s\r\n",
               tcrt_left_val, tcrt_right_val, distance,
               TCRT_Triggered() ? "<<< BLACK LINE! >>>" : "SAFE");

        usleep(TICK_MS * 1000);
    }
}

/* ========== 任务入口 ========== */
static void AutoDrive(void)
{
    printf("\r\n[BOOT] AutoDrive v2.2 loading...\r\n");

    WatchDogDisable();
    printf("[BOOT] Watchdog disabled.\r\n");

    GpioInit();
    printf("[BOOT] GPIO initialized.\r\n");

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

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(SERVO_PIN, WIFI_IOT_GPIO_DIR_OUT);
    printf("[BOOT] Servo GPIO2 configured.\r\n");

    osThreadAttr_t attr;
    attr.name = "AutoDrive";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)auto_drive_thread, NULL, &attr) == NULL) {
        printf("[BOOT] ERROR: Failed to create task!\r\n");
    } else {
        printf("[BOOT] Task created successfully.\r\n");
    }
}

APP_FEATURE_INIT(AutoDrive);