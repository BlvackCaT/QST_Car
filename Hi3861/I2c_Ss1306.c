#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ssd1306.h"

// ===== 新增：Task1前置声明 =====
void Task1(void);

static void i2c_sdd1306_demo(void)
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
		printf("ID = %d, Create Task1_ID is OK!", Task1_ID);
	}
}

void Task1(void)
{
	uint8_t displayBuff[20] = {0};
	uint8_t hour = 16, min = 0, sec = 0;
	SSD1306_Init();  // OLED显示屏初始化
	SSD1306_CLS();   // 清屏
	SSD1306_ShowStr(0, 0, (uint8_t *)" QST CAR ", 16);
	SSD1306_ShowStr(0, 3, (uint8_t *)"2025:10:08", 16);
	while (1)
	{
		sec++;
		if (sec > 59)
		{
			sec = 0;
			min++;
		}
		if (min > 59)
		{
			min = 0;
			hour++;
		}
		if (hour > 23)
		{
			hour = 0;
		}
		memset(displayBuff, 0, sizeof(displayBuff));//清除displayBuff中字符串
		sprintf((char*)displayBuff,"%02d:%02d:%02d", hour, min, sec);
		SSD1306_ShowStr(0, 2, (uint8_t *)displayBuff, 16);//写入OLED显示
		sleep(1);  // 1 s
	}
}

APP_FEATURE_INIT(i2c_sdd1306_demo);

