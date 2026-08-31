#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hal_bsp_ap3216c.h"
#include "hal_bsp_ssd1306.h"

// 小车灯光控制 GPIO：LEDR1 红色小灯接 IO06（高电平亮、低电平灭）
#define CAR_LIGHT_GPIO          WIFI_IOT_IO_NAME_GPIO_6
// 光照强度(als)阈值：低于 LIGHT_ON_THRESHOLD 判为"暗"自动亮灯，
// 高于 LIGHT_OFF_THRESHOLD 判为"亮"自动关灯，两者之间保持原状态(回差防抖)
#define LIGHT_ON_THRESHOLD      100
#define LIGHT_OFF_THRESHOLD     200

// 函数前置声明，解决 Task1 未定义编译报错
void Task1(void);

static void i2c_ap3216c_demo(void)
{
	osThreadAttr_t options;
	options.name = "thread_1";
	options.attr_bits = 0;
	options.cb_mem = NULL;
	options.cb_size = 0;
	options.stack_mem = NULL;
	options.stack_size = 1024;
	options.priority = osPriorityNormal;
	
	osThreadId_t Task1_ID;
	Task1_ID = osThreadNew((osThreadFunc_t)Task1, NULL, &options);
	if (Task1_ID != NULL)
	{
		printf("ID = %d, Create Task1_ID is OK!\n", Task1_ID);
	}
}

/**
 * ir 人体红外传感器
 * als 光强传感器
 * ps 接近传感器
 */
void Task1(void)
{
	AP3216C_Init();    // 三合一传感器初始化
	SSD1306_Init();    // OLED显示屏初始化
	SSD1306_CLS();     // 清屏
	
	// 小车灯光 GPIO 初始化（默认关灯）
	IoSetFunc(CAR_LIGHT_GPIO, WIFI_IOT_IO_FUNC_GPIO_6_GPIO);
	GpioSetDir(CAR_LIGHT_GPIO, WIFI_IOT_GPIO_DIR_OUT);
	GpioSetOutputVal(CAR_LIGHT_GPIO, WIFI_IOT_GPIO_VALUE0);
	
	printf("i2c_ap3216c_demo()!\n");
	uint16_t ir = 0, als = 0, ps = 0;
	uint8_t displayBuff[20] = {0};
	uint8_t light_on = 0;   // 0=关灯  1=开灯
	
	// 显示标题
	SSD1306_ShowStr(0, 0, (uint8_t *)" AP3216C Demo ", 16);
	
	while (1)
	{
		AP3216C_ReadData(&ir, &als, &ps);
		printf("人体红外传感器(ir) = %d   光强传感器(als) = %d   接近传感器(ps) = %d\r\n", ir, als, ps);
		
		// 依据光照强度自动控制小车灯光（暗亮灯 / 亮关灯）
		if (light_on == 0 && als < LIGHT_ON_THRESHOLD)
		{
			light_on = 1;
			GpioSetOutputVal(CAR_LIGHT_GPIO, WIFI_IOT_GPIO_VALUE1);
			printf("光照较弱(als=%d), 自动开灯\r\n", als);
		}
		else if (light_on == 1 && als > LIGHT_OFF_THRESHOLD)
		{
			light_on = 0;
			GpioSetOutputVal(CAR_LIGHT_GPIO, WIFI_IOT_GPIO_VALUE0);
			printf("光照较强(als=%d), 自动关灯\r\n", als);
		}
		
		// 在OLED上打印三项检测结果
		memset(displayBuff, 0, sizeof(displayBuff));
		sprintf((char *)displayBuff, "ir  = %d", ir);
		SSD1306_ShowStr(0, 1, displayBuff, 16);
		
		memset(displayBuff, 0, sizeof(displayBuff));
		sprintf((char *)displayBuff, "als = %d", als);
		SSD1306_ShowStr(0, 2, displayBuff, 16);
		
		memset(displayBuff, 0, sizeof(displayBuff));
		sprintf((char *)displayBuff, "ps  = %d", ps);
		SSD1306_ShowStr(0, 3, displayBuff, 16);
		
		sleep(1); // 1s
	}
}

APP_FEATURE_INIT(i2c_ap3216c_demo);

