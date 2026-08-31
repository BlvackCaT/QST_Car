#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"

// ========== 函数前置声明，解决undeclared报错 ==========
static void TCRTTask(void);
void Timer1_Callback(void *arg);
// =====================================================

#define GPIOL 13
#define GPIOR 14

uint32_t exec1;
osTimerId_t id1;
uint32_t timerDelay_1;
osStatus_t status;

void get_tcrt5000_value (void)
{
    WifiIotGpioValue id_status;

    GpioGetInputVal(GPIOL, &id_status);
    if (id_status == WIFI_IOT_GPIO_VALUE0) {
        printf("left black\r\n");
    }
    else
    {
        printf("left white\r\n");
    }

    GpioGetInputVal(GPIOR, &id_status);
    if (id_status == WIFI_IOT_GPIO_VALUE0) {
        printf("right black\r\n");
    }
    else
    {
        printf("right white\r\n");
    }
}

void Timer1_Callback(void *arg)
{
    (void)arg;
    get_tcrt5000_value();
}

static void TCRTTask(void)
{
    printf("start test tcrt5000\r\n");

    exec1 = 1U;
    id1 = osTimerNew(Timer1_Callback, osTimerPeriodic, &exec1, NULL);
    if (id1 != NULL)
    {
        timerDelay_1 = 50U;
        status = osTimerStart(id1, timerDelay_1);
        if (status != osOK)
        {
            printf("Timer could not be started\r\n");
        }
        else
        {
            printf("定时开启成功！\n");
        }
    }
}

static void TCRT(void)
{
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13,WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14,WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_13,WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_14,WIFI_IOT_GPIO_DIR_IN);

    osThreadAttr_t attr;
    attr.name = "TCRTTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = 25;

    if (osThreadNew((osThreadFunc_t)TCRTTask, NULL, &attr) == NULL) {
        printf("Failed to create TCRTTask!\n");
    }
}

APP_FEATURE_INIT(TCRT);
