#ifndef __CONTROL_SYSTEM_H
#define __CONTROL_SYSTEM_H

#include "stm32f10x.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	int target;
	int last_err;
	int integral;
} PID_TypeDef;

extern void PID_Init(void);
extern PID_TypeDef pid_left;
extern PID_TypeDef pid_right;
extern int L_speed;
extern int R_speed;
extern volatile uint32_t millis;
extern void System_Control(void);
extern void Set_Pwm(int left, int right);
extern void Motor_Init(void);


#endif
