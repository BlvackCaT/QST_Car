#include "control_system.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "colorful_led.h"

int L_coder,R_coder;
int Motor_A,Motor_B;
int OverflowTime=100;
#define START_PWM   2500
#define FEED_PWM    1600
#define MIN_RUN_PWM  2000
#define MIN_RUN_PWM_REV 1500
volatile uint32_t millis = 0;
PID_TypeDef pid_left;
PID_TypeDef pid_right;
int L_speed = 0;
int R_speed = 0;

void PID_Init(void)
{
    pid_left.Kp = 4.5f;
    pid_left.Ki = 0.04f;
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

int PI_Pos(PID_TypeDef *pid, int actual, int target)
{
    int err;
    int p_out;
    int out;

    err = target - actual;
    p_out = pid->Kp * err;

    pid->integral += pid->Ki * err;

    if(pid->integral > 3500)  pid->integral = 3500;
    if(pid->integral < -3500) pid->integral = -3500;

    out = p_out + pid->integral;

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

void CalculateAndControlMotors(int Target_MotorA, int Target_MotorB)
{
    int TageA, TageB;
    int errA, errB;
    static int prev_TageA = 0;
    static int prev_TageB = 0;

    L_coder = Read_Encoder(2);
    R_coder = Read_Encoder(3);
    L_speed = L_coder;
    R_speed = R_coder;

    TageA = Target_MotorA;
    TageB = Target_MotorB;

    if ((prev_TageA > 0 && TageA <= 0) || (prev_TageA < 0 && TageA >= 0) ||
        (prev_TageB > 0 && TageB <= 0) || (prev_TageB < 0 && TageB >= 0))
    {
        pid_left.integral = 0;
        pid_right.integral = 0;
    }
    prev_TageA = TageA;
    prev_TageB = TageB;

    errA = TageA - L_coder;
    errB = TageB - R_coder;

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

    if(TageA != 0)
    {
        int min_pwm = (TageA > 0) ? MIN_RUN_PWM : MIN_RUN_PWM_REV;
        if(Motor_A > 0 && Motor_A < min_pwm) Motor_A = min_pwm;
        else if(Motor_A < 0 && Motor_A > -min_pwm) Motor_A = -min_pwm;
    }
    if(TageB != 0)
    {
        int min_pwm = (TageB > 0) ? MIN_RUN_PWM : MIN_RUN_PWM_REV;
        if(Motor_B > 0 && Motor_B < min_pwm) Motor_B = min_pwm;
        else if(Motor_B < 0 && Motor_B > -min_pwm) Motor_B = -min_pwm;
    }

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

    if(Motor_A>7199) Motor_A=7199;
    if(Motor_A<-7199) Motor_A=-7199;
    if(Motor_B>7199) Motor_B=7199;
    if(Motor_B<-7199) Motor_B=-7199;

    Set_Pwm(Motor_A, Motor_B);
}

void Edge_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**************************************************************************
 * System_Control: 纯执行器，完全听从 Hi3861 指令
 *   DirA=2, DirB=2 → 边缘警报，停车+红灯
 *   DirA=0/1       → 正常电机指令，红灯灭
 **************************************************************************/
void System_Control(void)
{
    static int MotorA_Target = 0;
    static int MotorB_Target = 0;
    static int edge_alert = 0;

    /* ==== UART 帧解析 ==== */
    if (uart_rec_flag)
    {
        if (CAR_buff[0] == 2 && CAR_buff[2] == 2) {
            /* Hi3861 发来边缘停止指令 */
            edge_alert = 1;
            MotorA_Target = 0;
            MotorB_Target = 0;
        } else {
            /* 正常电机指令 */
            edge_alert = 0;
            MotorA_Target = Rs_To_CPR(CAR_buff[1] / 100.0f);
            MotorB_Target = Rs_To_CPR(CAR_buff[3] / 100.0f);
            if (CAR_buff[0] == 1) MotorA_Target = -MotorA_Target;
            if (CAR_buff[2] == 1) MotorB_Target = -MotorB_Target;
        }
        uart_rec_flag = 0;
        memset(CAR_buff, 0, 4);
    }

    /* ==== LED ==== */
    if (edge_alert) {
        All_LED_Red();
    } else {
        All_LED_Off();
    }

    CalculateAndControlMotors(MotorA_Target, MotorB_Target);
}