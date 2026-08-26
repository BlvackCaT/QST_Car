#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"

int main(void)
{
	RCC->CSR |= 1<<24;        //清除复位标志
	Stm32_Clock_Init(9);      //外部8M，72M系统时钟
	delay_init();           //延时初始化
	MY_NVIC_PriorityGroupConfig(2);
	uart_init(115200);
	JTAG_Set(JTAG_SWD_DISABLE);
	JTAG_Set(SWD_ENABLE);
	
	// PWM初始化：ARR=7199,PSC=9，72M/(PSC+1)/(ARR+1)= 72M /10 /7200 =1000Hz PWM
	PWM_Init(7199,9);
	
	// 如果没有炫彩灯代码，把下面这行注释
	colorful_led_Init();
	
	printf("QST青软\r\n");
	
	while(1)
	{
		// 参数范围：-7199 ~ +7199；正数前进，负数后退
		Set_Pwm(2500,2500);    //左右轮速度
		delay_ms(100);
	}
}

