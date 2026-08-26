#include "motor.h"

u8  AIN  = 0;
u8  BIN  = 0;
u16 PWMA = 0;
u16 PWMB = 0;

/*********************************************************
  函数功能: 初始化电机方向引脚 PB13(AIN), PB14(BIN)
 *********************************************************/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	AIN = 0;
	BIN = 0;
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction)AIN);
	GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)BIN);
}

/*********************************************************
  函数功能: TIM4 PWM初始化 PB6 CH1 PWMA  PB7 CH2 PWMB
 *********************************************************/
void PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	Motor_Init();
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	
	// PB6 PB7 复用推挽输出 TIM4_CH1 TIM4_CH2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	
	// =====【重要删除】TIM4是通用定时器，没有BDTR！TIM_CtrlPWMOutputs只给TIM1/TIM8高级定时器使用！=====
	// TIM_CtrlPWMOutputs(TIM4, ENABLE);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	
	TIM_Cmd(TIM4, ENABLE);
}

u32 myabs(long int a)
{
	u32 temp;
	if(a < 0)
		temp = -a;
	else
		temp = a;
	return temp;
}

/**
 * @brief 设置电机PWM
 * @param moto1：右轮PWMB(TIM4_CH2 PB7) 范围 -7199 ~ 7199
 * @param moto2：左轮PWMA(TIM4_CH1 PB6) 范围 -7199 ~ 7199
 * 正数前进，负数后退
 */
void Set_Pwm(int moto1,int moto2)
{
	if(moto2 >= 0)
	{
		AIN = 0;
		PWMA = myabs(moto2);
	}
	else
	{
		AIN = 1;
		PWMA = myabs(moto2);
	}
	
	if(moto1 >= 0)
	{
		BIN = 0;
		PWMB = myabs(moto1);
	}
	else
	{
		BIN = 1;
		PWMB = myabs(moto1);
	}
	
	// 更新方向引脚电平
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction)AIN);
	GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)BIN);
	
	// 更新PWM占空比
	TIM_SetCompare1(TIM4, PWMA);
	TIM_SetCompare2(TIM4, PWMB);
}

