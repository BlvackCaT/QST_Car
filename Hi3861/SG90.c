#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"

// ==========【必须放在SG90函数前面！！！】函数前置声明 ==========
static void thread1(void);
static void thread2(void);
static void thread3(void);
// ======================================================

osMutexId_t mutex_id;
uint8_t flag;

#define GPIO2 2

// 输出一路SG90脉冲，duty：高电平us，周期固定20000us
void set_angle(unsigned int duty)
{
	GpioSetDir(GPIO2, WIFI_IOT_GPIO_DIR_OUT);
	GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE1);
	hi_udelay(duty);
	GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE0);
	hi_udelay(20000 - duty);
}

void engine_run_0(void)
{
	for (int i = 0; i < 10; i++)
	{
		set_angle(500);
	}
}

void engine_run_45(void)
{
	for (int i = 0; i < 10; i++)
	{
		set_angle(1000);
	}
}

void engine_run_90(void)
{
	for (int i = 0; i < 10; i++)
	{
		set_angle(1500);
	}
}

void engine_run_135(void)
{
	for (int i = 0; i < 10; i++)
	{
		set_angle(2000);
	}
}

void engine_run_180(void)
{
	for (int i = 0; i < 10; i++)
	{
		set_angle(2500);
	}
}

static void SG90(void)
{
	printf("====NEW_BIN_TEST_001====\r\n");
	GpioInit();
	IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
	GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_DIR_OUT);
	
	osThreadAttr_t attr;
	attr.attr_bits  = 0U;
	attr.cb_mem     = NULL;
	attr.cb_size    = 0U;
	attr.stack_mem  = NULL;
	attr.stack_size = 1024 * 4;
	
	attr.name     = "thread1";
	attr.priority = 26;
	if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL)
	{
		printf("Failed to create thread1!\n");
	}
	
	attr.name     = "thread2";
	attr.priority = 25;
	if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL)
	{
		printf("Failed to create thread2!\n");
	}
	
	attr.name     = "thread3";
	attr.priority = 24;
	if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL)
	{
		printf("Failed to create thread3!\n");
	}
	
	mutex_id = osMutexNew(NULL);
	if (mutex_id == NULL)
	{
		printf("Failed to create Mutex!\n");
	}
}

static void thread1(void)
{
	osDelay(100U);
	while (1)
	{
		osMutexAcquire(mutex_id, osWaitForever);
		printf("thread1 is runing.\r\n");
		flag = 90;
		engine_run_90();
		osDelay(500U);
		osMutexRelease(mutex_id);
	}
}

static void thread2(void)
{
	osDelay(100U);
	while (1)
	{
		printf("thread2 is runing.\r\n");
		switch (flag)
		{
		case 90:
			printf("SG90 turn 90 du.\r\n");
			break;
		case 180:
			printf("SG90 turn 180 du.\r\n");
			break;
		default:
			break;
		}
		flag = 0;
		osDelay(100U);
	}
}

static void thread3(void)
{
	osDelay(200U);
	while (1)
	{
		osMutexAcquire(mutex_id, osWaitForever);
		printf("thread3 is runing.\r\n");
		flag = 180;
		engine_run_180();
		osDelay(300U);
		osMutexRelease(mutex_id);
	}
}

APP_FEATURE_INIT(SG90);

