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
#define TRIG_PIN            7
#define ECHO_PIN            8
#define SERVO_PIN           2
#define TCRT_LEFT_PIN       WIFI_IOT_IO_NAME_GPIO_13
#define TCRT_RIGHT_PIN      WIFI_IOT_IO_NAME_GPIO_14
#define GPIO_FUNC           0

/* ========== 循迹参数 ========== */
#define LINE_SPEED         100      // 直行速度
#define TURN_SPEED          40      // 转弯时外侧轮速度
#define TURN_DIFF_MIN       25      // 初始修正力度(内侧轮减速量)
#define TURN_DIFF_MAX       40      // 最大修正力度(内侧轮几乎停转)
#define TURN_INCREMENT      5       // 每tick增量(持续压线时逐步加大)
#define SHARP_TURN_DIFF     25      // 急转向时内侧轮减速量
#define LOST_TURN_SPEED     35      // 丢线搜索时的基础速度
#define LOST_TIMEOUT        20      // 丢线超时(tick)，超时后扩大搜索
#define OBSTACLE_THRESHOLD  12.0f   // 紧急停车距离(cm)
#define TICK_MS             50      // 每tick毫秒数

/* ========== 舵机角度(us) ========== */
#define SERVO_CENTER        1500

/* ========== 运动状态枚举 ========== */
typedef enum {
    STATE_FORWARD = 0,      // 直行
    STATE_TURN_LEFT,        // 左转
    STATE_TURN_RIGHT,       // 右转
    STATE_LOST_SEARCH,      // 丢线搜索(短期)
    STATE_LOST_SPIN,        // 丢线旋转(长期)
    STATE_STOP,             // 停车
} MotionState;

static const char *StateName(MotionState s)
{
    switch (s) {
    case STATE_FORWARD:     return "FORWARD";
    case STATE_TURN_LEFT:   return "TURN_LEFT";
    case STATE_TURN_RIGHT:  return "TURN_RIGHT";
    case STATE_LOST_SEARCH: return "LOST_SEARCH";
    case STATE_LOST_SPIN:   return "LOST_SPIN";
    case STATE_STOP:        return "STOP";
    default:                return "UNKNOWN";
    }
}

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

void car_stop(void) { stm32motor_control(0, 0); }

/* 双黑丢线时发送: 红灯+转向, 协议 DirA=2 DirB=2 且电机值非零 */
void stm32motor_lost(int motorA, int motorB)
{
    if (motorA > 150) motorA = 150;
    if (motorB > 150) motorB = 150;

    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = 2;           // DirA=2 双黑信号
    uart_sendbuf[2] = (uint8_t)motorA;
    uart_sendbuf[3] = 2;           // DirB=2 双黑信号
    uart_sendbuf[4] = (uint8_t)motorB;
    uart_sendbuf[5] = 0xFD;
    UartWrite(WIFI_IOT_UART_IDX_2, uart_sendbuf, 6);
}

/* ========== HC-SR04 超声波(仅正前方) ========== */
static void HC_SR04_Init(void)
{
    hi_io_set_func(TRIG_PIN, GPIO_FUNC);
    hi_io_set_func(ECHO_PIN, GPIO_FUNC);
    GpioSetDir(TRIG_PIN, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(ECHO_PIN, WIFI_IOT_GPIO_DIR_IN);
    GpioSetOutputVal(TRIG_PIN, WIFI_IOT_GPIO_VALUE0);
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

/* ========== SG90 舵机(仅固定正前方) ========== */
void ServoSetAngle(unsigned int duty)
{
    for (int i = 0; i < 10; i++) {
        GpioSetOutputVal(SERVO_PIN, WIFI_IOT_GPIO_VALUE1);
        hi_udelay(duty);
        GpioSetOutputVal(SERVO_PIN, WIFI_IOT_GPIO_VALUE0);
        hi_udelay(20000 - duty);
    }
}

/* ========== TCRT5000 红外循迹 ========== */
static uint8_t tcrt_left_val = 0;
static uint8_t tcrt_right_val = 0;

static void TCRT_Init(void)
{
    IoSetFunc(TCRT_LEFT_PIN, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(TCRT_RIGHT_PIN, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(TCRT_LEFT_PIN, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(TCRT_RIGHT_PIN, WIFI_IOT_GPIO_DIR_IN);
}

/*
 * 读取两个传感器:
 *   tcrt_val = 0 -> 黑线 (GPIO_VALUE0)
 *   tcrt_val = 1 -> 白底 (GPIO_VALUE1)
 */
static void TCRT_ReadBoth(void)
{
    WifiIotGpioValue l, r;
    GpioGetInputVal(TCRT_LEFT_PIN, &l);
    GpioGetInputVal(TCRT_RIGHT_PIN, &r);
    tcrt_left_val = (l == WIFI_IOT_GPIO_VALUE1) ? 0 : 1;
    tcrt_right_val = (r == WIFI_IOT_GPIO_VALUE1) ? 0 : 1;
}

/*
 * 窄线循迹: 线宽 < 传感器间距, 线在两个传感器之间
 *   0 = 都在白底 -> 线在中间, 直行
 *  -1 = 左黑右白 -> 线偏左, 左转修正
 *  +1 = 左白右黑 -> 线偏右, 右转修正
 *  99 = 都黑 -> 丢线
 */
static int CalcLineError(void)
{
    if (tcrt_left_val == 1 && tcrt_right_val == 1) {
        return 0;   // 都在白底 -> 线在中间, 直行
    }
    if (tcrt_left_val == 0 && tcrt_right_val == 1) {
        return -1;  // 左黑右白 -> 线偏左, 左转
    }
    if (tcrt_left_val == 1 && tcrt_right_val == 0) {
        return 1;   // 左白右黑 -> 线偏右, 右转
    }
    return 99;      // 都黑 -> 丢线
}

/*
 * 循迹转向控制:
 *   根据误差值调整左右轮速度, 实现平滑转向
 *   返回当前运动状态, 并通过指针返回左右轮速度
 */
static MotionState LineFollowSteer(int error, int turn_diff, int *out_left, int *out_right)
{
    int left_speed, right_speed;
    MotionState state;

    switch (error) {
    case 0:   // 直行: 全速
        left_speed = LINE_SPEED;
        right_speed = LINE_SPEED;
        state = STATE_FORWARD;
        break;
    case -1:  // 左转: 降速 + 右轮快左轮慢
        left_speed = TURN_SPEED - turn_diff;
        right_speed = TURN_SPEED;
        state = STATE_TURN_LEFT;
        break;
    case 1:   // 右转: 降速 + 左轮快右轮慢
        left_speed = TURN_SPEED;
        right_speed = TURN_SPEED - turn_diff;
        state = STATE_TURN_RIGHT;
        break;
    default:  // 丢线
        left_speed = LINE_SPEED;
        right_speed = LINE_SPEED;
        state = STATE_FORWARD;
        break;
    }

    stm32motor_control(left_speed, right_speed);
    *out_left = left_speed;
    *out_right = right_speed;
    return state;
}

/* ========== 循迹主任务 ========== */
static void line_follow_thread(void *arg)
{
    (void)arg;
    float distance = 0.0;
    int error = 0;
    int default_turn_dir = 1;    // 首次双黑翻转为-1(左转), 之后交替
    int last_nonzero_error = 0;
    int error_duration = 0;
    int turn_diff = 0;
    int lost_count = 0;
    int loop_count = 0;
    int left_speed = 0, right_speed = 0;
    MotionState state = STATE_STOP;
    MotionState prev_state = STATE_STOP;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  Line Follow v1.1 - TCRT5000 Tracking \r\n");
    printf("========================================\r\n");
    printf("[HW] TCRT5000: Left=GPIO13 Right=GPIO14\r\n");
    printf("[HW] HC-SR04 : Trig=GPIO7 Echo=GPIO8 (emergency stop)\r\n");
    printf("[HW] STM32   : UART2 (GPIO11/12, 115200)\r\n");
    printf("[CFG] Speed=%d, TurnDiff=%d~%d, SharpTurn=%d, Tick=%dms\r\n",
           LINE_SPEED, TURN_DIFF_MIN, TURN_DIFF_MAX, SHARP_TURN_DIFF, TICK_MS);
    printf("[CFG] ObstacleStop=%.0fcm, LostTimeout=%d\r\n",
           OBSTACLE_THRESHOLD, LOST_TIMEOUT);
    printf("[TIP] 黑线=GPIO0, 白底=GPIO1. Adjust TCRT5000 potentiometer if needed!\r\n");
    printf("========================================\r\n");

    /* 舵机归中固定 */
    printf("[INIT] Servo centering (fixed forward)...\r\n");
    ServoSetAngle(SERVO_CENTER);
    usleep(200 * 1000);

    /* TCRT5000初始化 */
    TCRT_Init();
    printf("[INIT] TCRT5000 sensors ready.\r\n");

    /* HC-SR04初始化 */
    HC_SR04_Init();
    printf("[INIT] HC-SR04 initialized.\r\n");

    /* UART同步 */
    printf("[TEST] UART sync with STM32...\r\n");
    for (int i = 0; i < 5; i++) {
        car_stop();
        usleep(50 * 1000);
    }

    /* 超声波预热 */
    printf("[TEST] HC-SR04 warm-up...\r\n");
    for (int i = 0; i < 3; i++) {
        GetDistance();
        usleep(100 * 1000);
    }
    distance = GetDistance();
    printf("[TEST] Distance = %.1f cm\r\n", distance);

    /* TCRT自检 */
    printf("[TEST] TCRT5000 self-test...\r\n");
    for (int i = 0; i < 5; i++) {
        TCRT_ReadBoth();
        printf("[TEST]   Reading %d: L=%d R=%d\r\n",
               i + 1, tcrt_left_val, tcrt_right_val);
        usleep(200 * 1000);
    }

    printf("========================================\r\n");
    printf("[READY] Line following started!\r\n");
    printf("========================================\r\n\r\n");

    /*
     * 状态栏格式:
     * [STATE] L=电机左 R=电机右 | TCRT:L/R | 误差 | 修正力度 | 丢线计数 | 距离
     * 例如:
     * [FORWARD   ] L:+050 R:+050 | TCRT:1/1 | err= 0 | diff= 0 | lost= 0 | 35.2cm
     * [TURN_LEFT ] L:+045 R:+050 | TCRT:0/1 | err=-1 | diff= 5 | lost= 0 | 40.1cm  (刚压线)
     * [TURN_LEFT ] L:+020 R:+050 | TCRT:0/1 | err=-1 | diff=30 | lost= 0 | 38.5cm  (持续压线,力度加大)
     * [LOST_SRCH ] L:+025 R:+050 | TCRT:1/1 | err=99 | diff= 0 | lost= 3 | 28.7cm
     * [STOP      ] L:+000 R:+000 | TCRT:0/0 | err= 0 | diff= 0 | lost= 0 |  8.3cm
     */
    printf("---------- STATUS ----------\r\n");

    while (1) {
        loop_count++;

        /* 读取传感器 */
        TCRT_ReadBoth();
        error = CalcLineError();

        /* ===== 优先级1: 超声波紧急停车 ===== */
        distance = GetDistance();
        if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
            state = STATE_STOP;
            left_speed = 0;
            right_speed = 0;

            if (state != prev_state) {
                printf("\r\n[!!!] OBSTACLE at %.1f cm! STOPPING!\r\n\r\n", distance);
                prev_state = state;
            }
            car_stop();
            usleep(500 * 1000);

            /* 等待障碍物清除后继续 */
            while (1) {
                distance = GetDistance();
                if (distance < 0 || distance >= OBSTACLE_THRESHOLD) {
                    printf("[OK] Path clear (%.1f cm). Resuming...\r\n\r\n", distance);
                    break;
                }
                printf("[WAIT] Obstacle still there (%.1f cm)...\r\n", distance);
                usleep(200 * 1000);
            }
            lost_count = 0;
            prev_state = STATE_STOP;
            continue;
        }

        /* ===== 优先级2: 循迹控制 ===== */
        if (error == 99) {
            /* 丢线: 双黑, 脱轨 */
            if (lost_count == 0) {
                /* 刚进入丢线, 交替默认转向 */
                default_turn_dir = -default_turn_dir;
                printf("[LOST] Both black! Default turn: %s\r\n",
                       default_turn_dir == -1 ? "LEFT" : "RIGHT");
            }
            lost_count++;

            if (lost_count < LOST_TIMEOUT) {
                /* 短期丢线: 按交替方向搜索 */
                state = STATE_LOST_SEARCH;

                if (default_turn_dir == -1) {
                    left_speed = LINE_SPEED - SHARP_TURN_DIFF;
                    right_speed = LINE_SPEED;
                } else {
                    left_speed = LINE_SPEED;
                    right_speed = LINE_SPEED - SHARP_TURN_DIFF;
                }
                stm32motor_lost(left_speed, right_speed);
            } else {
                /* 长时间丢线: 原地旋转搜索 */
                state = STATE_LOST_SPIN;

                if (lost_count % 2 == 0) {
                    left_speed = -LOST_TURN_SPEED;
                    right_speed = LOST_TURN_SPEED;
                } else {
                    left_speed = LOST_TURN_SPEED;
                    right_speed = -LOST_TURN_SPEED;
                }
                stm32motor_lost(left_speed, right_speed);
            }
        } else {
            /* 正常循迹: 渐进式修正 */
            if (lost_count > 0) {
                printf("[FOUND] Line recovered after %d ticks!\r\n\r\n", lost_count);
            }
            lost_count = 0;

            /* 持续压同一侧 → 弯越急, 修正力度越大
             * 用 last_nonzero_error 避免直行(0)中断计数 */
            if (error != 0 && error == last_nonzero_error) {
                error_duration++;
            } else if (error != 0 && error != last_nonzero_error) {
                error_duration = 0;  // 换边, 重置
                last_nonzero_error = error;
            }
            /* error == 0 时保持计数不变 */
            turn_diff = TURN_DIFF_MIN + error_duration * TURN_INCREMENT;
            if (turn_diff > TURN_DIFF_MAX) turn_diff = TURN_DIFF_MAX;

            state = LineFollowSteer(error, turn_diff, &left_speed, &right_speed);
        }

        /* ===== 串口状态输出 ===== */
        if (state != prev_state) {
            /* 状态切换时打印醒目提示 */
            printf("\r\n>>> STATE: %s -> %s <<<\r\n",
                   StateName(prev_state), StateName(state));
            prev_state = state;
        }

        /* 每个tick打印一行状态: [状态] 电机速度 | 传感器 | 误差 | 修正 | 丢线 | 距离 */
        printf("[%-10s] L:%c%03d R:%c%03d | TCRT:%d/%d | err=%2d | diff=%2d | lost=%2d | %5.1fcm\r\n",
               StateName(state),
               (left_speed >= 0) ? '+' : '-', (left_speed >= 0) ? left_speed : -left_speed,
               (right_speed >= 0) ? '+' : '-', (right_speed >= 0) ? right_speed : -right_speed,
               tcrt_left_val, tcrt_right_val,
               error, turn_diff, lost_count,
               distance);

        usleep(TICK_MS * 1000);
    }
}

/* ========== 任务入口 ========== */
static void LineFollow(void)
{
    printf("\r\n[BOOT] LineFollow v1.0 loading...\r\n");

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
    attr.name = "LineFollow";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)line_follow_thread, NULL, &attr) == NULL) {
        printf("[BOOT] ERROR: Failed to create task!\r\n");
    } else {
        printf("[BOOT] Task created successfully.\r\n");
    }
}

APP_FEATURE_INIT(LineFollow);