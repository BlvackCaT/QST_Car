#include "stm32f10x.h"
#include "sys.h"

int main(void)
{
	Stm32_Clock_Init(9);                    // 外部8MHz，9倍频，系统时钟72MHz
	MY_NVIC_PriorityGroupConfig(2);        // 中断优先级分组
	uart_init(115200);                      // 串口初始化
	JTAG_Set(JTAG_SWD_DISABLE);             // 关闭JTAG接口
	JTAG_Set(SWD_ENABLE);                   // 打开SWD接口
	
	colorful_led_Init();                    // 炫彩灯初始化
	
	printf("QST青软\r\n");
	
	/* 前灯8颗全部设置为白色 */
	L_ws2812_rgb(1, WS_WHITE);
	L_ws2812_rgb(2, WS_WHITE);
	L_ws2812_rgb(3, WS_WHITE);
	L_ws2812_rgb(4, WS_WHITE);
	L_ws2812_rgb(5, WS_WHITE);
	L_ws2812_rgb(6, WS_WHITE);
	L_ws2812_rgb(7, WS_WHITE);
	L_ws2812_rgb(8, WS_WHITE);
	
	L_ws2812_refresh(8);
	
	/* 后灯6颗全部设置为红色 */
	R_ws2812_rgb(1, WS_RED);
	R_ws2812_rgb(2, WS_RED);
	R_ws2812_rgb(3, WS_RED);
	R_ws2812_rgb(4, WS_RED);
	R_ws2812_rgb(5, WS_RED);
	R_ws2812_rgb(6, WS_RED);
	
	R_ws2812_refresh(6);
	
	/* 保持LED当前状态，不再执行跑马灯或其他灯效 */
	while(1)
	{
		delay_ms(100);
	}
}

