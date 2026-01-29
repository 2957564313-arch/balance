#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

// PWM 参数
// 频率 17kHz (人耳听不到的超声波频段，电机更静音)
#define PWM_FREQ            17000
// 最大占空比 (逐飞库通常是 0-10000 或 0-100)
// 这里假设库使用的是 0-10000 的分辨率
#define MOTOR_MAX_DUTY      10000

// 左电机 PWM (P6.0 / P6.4)
#define MOTOR_L_IN1_PWM     PWMA_CH1P_P60
#define MOTOR_L_IN2_PWM     PWMA_CH3P_P64

// 右电机 PWM (P6.2 / P6.6)
#define MOTOR_R_IN1_PWM     PWMA_CH2P_P62
#define MOTOR_R_IN2_PWM     PWMA_CH4P_P66

// 左编码器 (硬件 Timer1: P3.4 / P3.5)
#define ENCODER_L_TIM       TIM_1
#define ENCODER_L_A         IO_P34 
#define ENCODER_L_B         IO_P35

// 右编码器 (软件模拟: P0.4 / P5.3)
#define ENCODER_R_A_PIN     IO_P04
#define ENCODER_R_B_PIN     IO_P53

// API 函数声明
void Motor_Init(void);
void Motor_Set_L(int16 duty);
void Motor_Set_R(int16 duty);
void Soft_Encoder_Scan(void);
void Encoder_Get_Val(int16 *L, int16 *R);

#endif
