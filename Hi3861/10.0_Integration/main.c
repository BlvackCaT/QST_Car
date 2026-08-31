/**
 * @file    main.c
 * @brief   QST小车传感器数据集成展示
 *          按下复位键后，在串口助手上循环输出所有传感器数据：
 *          TCRT5000红外循迹、HC-SR04超声波测距、
 *          SHT20温湿度、AP3216C三合一(红外/光照/接近)
 *          同时OLED显示关键数据，根据光照强度自动控制LEDR1亮灭
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_watchdog.h"
#include "hi_io.h"
#include "hi_time.h"
#include "hal_bsp_ap3216c.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_ssd1306.h"

/* ======== GPIO 引脚定义 ======== */
#define TCRT_LEFT   13          // TCRT5000 左路红外 (GPIO13)
#define TCRT_RIGHT  14          // TCRT5000 右路红外 (GPIO14)
#define HCSR_TRIG   7           // HC-SR04 触发引脚 (GPIO7)
#define HCSR_ECHO   8           // HC-SR04 回响引脚 (GPIO8)
#define LEDR1_GPIO  WIFI_IOT_IO_NAME_GPIO_6  // LEDR1 红色小灯 (IO06)

/* ======== 光照阈值(自动灯光) ======== */
#define LIGHT_ON_THRESHOLD   100
#define LIGHT_OFF_THRESHOLD  200

/* ======== 函数前置声明 ======== */
void SensorTask(void);

/* ======== HC-SR04 超声波测距（带超时保护） ======== */
static float GetDistance(void)
{
    static unsigned long long start_time = 0, time = 0;
    float distance = 0.0f;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    unsigned int timeout = 0;

    GpioSetDir(HCSR_TRIG, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(HCSR_ECHO, WIFI_IOT_GPIO_DIR_IN);

    GpioSetOutputVal(HCSR_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(HCSR_TRIG, WIFI_IOT_GPIO_VALUE0);

    while (1) {
        GpioGetInputVal(HCSR_ECHO, &value);
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
        if (timeout > 500000) {  // 超时，返回 -1
            distance = -1.0f;
            return distance;
        }
    }
    distance = time * 0.034f / 2.0f;
    return distance;
}

/* ======== TCRT5000 红外循迹 ======== */
static void GetTCRT(char *left, char *right)
{
    WifiIotGpioValue val;
    GpioGetInputVal(TCRT_LEFT, &val);
    *left = (val == WIFI_IOT_GPIO_VALUE0) ? 'B' : 'W';  // B=黑线 W=白底

    GpioGetInputVal(TCRT_RIGHT, &val);
    *right = (val == WIFI_IOT_GPIO_VALUE0) ? 'B' : 'W';
}

/* ======== 主传感器任务 ======== */
void SensorTask(void)
{
    /* ---- 外设初始化 ---- */
    WatchDogDisable();
    GpioInit();

    // I2C 引脚 (SHT20 / AP3216C / SSD1306 共用 I2C0)
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_10, WIFI_IOT_IO_FUNC_GPIO_10_I2C0_SDA);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9,  WIFI_IOT_IO_FUNC_GPIO_9_I2C0_SCL);
    I2cInit(WIFI_IOT_I2C_IDX_0, 400000);
    I2cSetBaudrate(WIFI_IOT_I2C_IDX_0, 400000);

    // TCRT5000 巡线（输入）
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_GPIO_DIR_IN);

    // HC-SR04 超声波
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);

    // LEDR1 灯光（GPIO6）
    IoSetFunc(LEDR1_GPIO, WIFI_IOT_IO_FUNC_GPIO_6_GPIO);
    GpioSetDir(LEDR1_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(LEDR1_GPIO, WIFI_IOT_GPIO_VALUE0);

    // 传感器初始化
    SSD1306_Init();
    SSD1306_CLS();
    SHT20_Init();
    AP3216C_Init();

    printf("\n========== QST Car Sensor Integration ==========\n\n");

    /* ---- 传感器数据变量 ---- */
    uint16_t ir = 0, als = 0, ps = 0;
    float    temperature = 0.0f, humidity = 0.0f;
    float    distance = 0.0f;
    char     tcrt_l = 'W', tcrt_r = 'W';
    uint8_t  light_on = 0;
    uint8_t  oled_buf[20] = {0};

    // OLED 标题
    SSD1306_ShowStr(0, 0, (uint8_t *)" QST CAR SENSOR ", 16);

    while (1) {
        /* ===== 1. AP3216C 三合一传感器 ===== */
        AP3216C_ReadData(&ir, &als, &ps);
        printf("[AP3216C] IR(人体红外)=%d  ALS(光照)=%d  PS(接近)=%d\r\n", ir, als, ps);

        /* ===== 2. SHT20 温湿度传感器 ===== */
        SHT20_ReadData(&temperature, &humidity);
        printf("[SHT20]   温度=%.2f C  湿度=%.2f %%\r\n", temperature, humidity);

        /* ===== 3. HC-SR04 超声波测距 ===== */
        distance = GetDistance();
        if (distance < 0) {
            printf("[HC-SR04]  距离=超时(无回响)\r\n");
        } else {
            printf("[HC-SR04]  距离=%.1f cm\r\n", distance);
        }

        /* ===== 4. TCRT5000 红外循迹 ===== */
        GetTCRT(&tcrt_l, &tcrt_r);
        printf("[TCRT5000] 左=%c(黑/白)  右=%c(黑/白)\r\n",
               tcrt_l == 'B' ? 'B' : 'W', tcrt_r == 'B' ? 'B' : 'W');

        /* ===== 5. 自动灯光控制(LEDR1) ===== */
        if (light_on == 0 && als < LIGHT_ON_THRESHOLD) {
            light_on = 1;
            GpioSetOutputVal(LEDR1_GPIO, WIFI_IOT_GPIO_VALUE1);
            printf("[LEDR1]   光照较弱(als=%d) -> 自动亮灯\r\n", als);
        } else if (light_on == 1 && als > LIGHT_OFF_THRESHOLD) {
            light_on = 0;
            GpioSetOutputVal(LEDR1_GPIO, WIFI_IOT_GPIO_VALUE0);
            printf("[LEDR1]   光照较强(als=%d) -> 自动关灯\r\n", als);
        }

        printf("----------------------------------------------\r\n\n");

        /* ===== 6. OLED 显示关键数据 ===== */
        memset(oled_buf, 0, sizeof(oled_buf));
        sprintf((char *)oled_buf, "T:%.1fC H:%.1f%%", temperature, humidity);
        SSD1306_ShowStr(0, 1, oled_buf, 16);

        memset(oled_buf, 0, sizeof(oled_buf));
        if (distance < 0) {
            sprintf((char *)oled_buf, "Dist: TIMEOUT");
        } else {
            sprintf((char *)oled_buf, "Dist:%.1f cm", distance);
        }
        SSD1306_ShowStr(0, 2, oled_buf, 16);

        memset(oled_buf, 0, sizeof(oled_buf));
        sprintf((char *)oled_buf, "ir:%d al:%d", ir, als);
        SSD1306_ShowStr(0, 3, oled_buf, 16);

        sleep(1);  // 每秒刷新一次
    }
}

/* ======== 入口 ======== */
static void SensorDemoEntry(void)
{
    osThreadAttr_t attr;
    attr.name       = "SensorTask";
    attr.attr_bits  = 0U;
    attr.cb_mem     = NULL;
    attr.cb_size    = 0U;
    attr.stack_mem  = NULL;
    attr.stack_size = 1024 * 8;
    attr.priority   = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)SensorTask, NULL, &attr) == NULL) {
        printf("Failed to create SensorTask!\n");
    }
}

APP_FEATURE_INIT(SensorDemoEntry);