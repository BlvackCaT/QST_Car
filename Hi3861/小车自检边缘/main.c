#include "stm32f10x.h"
#include "sys.h"
#include "encoder.h"
#include "control_system.h"
#include "usart.h"

int main(void)
{
	Stm32_Clock_Init(9);	//外部时钟8Mhz 9倍频  8*9= 72mhz系统时钟72mhz
	MY_NVIC_PriorityGroupConfig(2);	//=====中断优先级分组
	uart_init(115200);			//=====串口初始化为115200
	JTAG_Set(JTAG_SWD_DISABLE);		//=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);			//=====打开SWD接口 如果需要下载程序要打开SWD接口的
	Encoder_Init_TIM2();			//=====初始化编码器2
	Encoder_Init_TIM3();			//=====初始化编码器3
	PWM_Init(7199,9);			//=====定时器初始化 频率1000
	colorful_led_Init();			//=====全彩灯初始化

	Motor_Init();	//电机方向引脚初始化，必须在PID_Init前
	PID_Init();		//PID积分、误差全部清零
	Edge_Sensor_Init();	//边缘传感器初始化（PA11/PA12）

	SysTick_Config(72000000/1000);	//滴答定时器：每1ms产生一次中断
	delay_ms(200);	//上电防抖等待
	//		printf("UART Control Ready\r\n");
	//		printf("Frame: [0xFC][DirA][MotorA][DirB][MotorB][0xFD]\r\n");

	/**主循环**/
	while(1)
	{
		delay_ms(100);
	}
}