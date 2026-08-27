#include "control_system.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>

int L_coder,R_coder;
int Motor_A,Motor_B;
int OverflowTime=100;
#define START_PWM   2500   // 静止起转：踢一脚用的最小PWM
#define FEED_PWM    1600   // 速度前馈：基础驱动
#define MIN_RUN_PWM 2000   // 运行最小PWM兜底：防止稳态跌入死区导致周期顿挫
#define L_TRIM      0.8      // 左轮目标偏置（编码器计数/周期）
#define R_TRIM      1.2  // 右轮目标偏置，正=右轮目标更高（修正右偏）
volatile uint32_t millis = 0;
volatile uint32_t seconds = 0;
PID_TypeDef pid_left;
PID_TypeDef pid_right;
int L_speed = 0;
int R_speed = 0;

void PID_Init(void)
{
	pid_left.Kp = 4.5f;
	pid_left.Ki = 0.04f;   // 从0.08降下来，压掉2-3s的低频振荡
	pid_left.Kd = 0.003f;
	pid_left.target = 0;
	pid_left.last_err = 0;
	pid_left.integral = 0;
	
	pid_right.Kp = 4.5f;
	pid_right.Ki = 0.04f;
	pid_right.Kd = 0.003f;
	pid_right.target = 0;
	pid_right.last_err = 0;
	pid_right.integral = 0;
}

/**
 * 位置式PI：
 * 1.大误差依然允许少量积分，提供启动力矩；
 * 2.严格积分限幅，卡死不会无限累积；
 * 3.输出限幅。
 */
int PI_Pos(PID_TypeDef *pid, int actual, int target)
{
	int err;
	int p_out;
	int out;
	
	err = target - actual;
	p_out = pid->Kp * err;
	
	// 无论误差大小，都允许积分，但积分有硬上限，不会无限涨
	pid->integral += pid->Ki * err;
	
	// ==========积分硬限幅，防止卡死积分疯涨（关键）==========
	if(pid->integral > 3500)  pid->integral = 3500;
	if(pid->integral < -3500) pid->integral = -3500;
	
	out = p_out + pid->integral;
	
	//输出限幅
	if(out > 7199)  out = 7199;
	if(out < -7199) out = -7199;
	
	pid->last_err = err;
	return out;
}

int Rs_To_CPR(float rads)
{
	int CRP=0;
	CRP=rads * ((700*4)/(1000/OverflowTime));
	return CRP;
}

void System_Control(void)
{
	int TageA;
	int TageB;
	int errA;
	int errB;
	
	L_coder=Read_Encoder(2);
	R_coder=Read_Encoder(3);
	L_speed = L_coder;
	R_speed = R_coder;
	
	printf("left:%d right:%d ",L_coder,R_coder);
	
	// 左右目标加计数偏置，修正跑偏（粒度1个计数）
	TageA=Rs_To_CPR(0.5f) + L_TRIM;
	TageB=Rs_To_CPR(0.5f) + R_TRIM;
	
	errA = TageA - L_coder;
	errB = TageB - R_coder;
	
	//==== 速度前馈 + PI：前馈扛住基础驱动力，PI做速度微调
	if(TageA > 0)
		Motor_A = FEED_PWM + PI_Pos(&pid_left, L_coder, TageA);
	else if(TageA < 0)
		Motor_A = -FEED_PWM + PI_Pos(&pid_left, L_coder, TageA);
	else
		Motor_A = PI_Pos(&pid_left, L_coder, TageA);
	
	if(TageB > 0)
		Motor_B = FEED_PWM + PI_Pos(&pid_right, R_coder, TageB);
	else if(TageB < 0)
		Motor_B = -FEED_PWM + PI_Pos(&pid_right, R_coder, TageB);
	else
		Motor_B = PI_Pos(&pid_right, R_coder, TageB);
	
	//==== 运行最小PWM兜底：目标非零时输出不低于 MIN_RUN_PWM，防止稳态跌入死区周期顿挫
	if(TageA != 0)
	{
		if(Motor_A > 0 && Motor_A < MIN_RUN_PWM) Motor_A = MIN_RUN_PWM;
		else if(Motor_A < 0 && Motor_A > -MIN_RUN_PWM) Motor_A = -MIN_RUN_PWM;
	}
	if(TageB != 0)
	{
		if(Motor_B > 0 && Motor_B < MIN_RUN_PWM) Motor_B = MIN_RUN_PWM;
		else if(Motor_B < 0 && Motor_B > -MIN_RUN_PWM) Motor_B = -MIN_RUN_PWM;
	}
	
	//==== 静止起转：误差大 且 编码器没动时，踢一脚；一动起来立刻撤
	if(abs(errA) > 60 && L_coder == 0)
	{
		if(Motor_A > 0 && Motor_A < START_PWM) Motor_A = START_PWM;
		if(Motor_A < 0 && Motor_A > -START_PWM) Motor_A = -START_PWM;
	}
	if(abs(errB) > 60 && R_coder == 0)
	{
		if(Motor_B > 0 && Motor_B < START_PWM) Motor_B = START_PWM;
		if(Motor_B < 0 && Motor_B > -START_PWM) Motor_B = -START_PWM;
	}
	
	//全局输出限幅
	if(Motor_A>7199) Motor_A=7199;
	if(Motor_A<-7199) Motor_A=-7199;
	if(Motor_B>7199) Motor_B=7199;
	if(Motor_B<-7199) Motor_B=-7199;
	
	printf("tar:%d,%d pwm:%d,%d\r\n",TageA,TageB,Motor_A,Motor_B);
	
	Set_Pwm(Motor_A,Motor_B);
}
