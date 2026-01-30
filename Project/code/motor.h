#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

// ======================= PWM 参数 =======================
// 频率 17kHz（超声段，比较静音）
#define PWM_FREQ                17000
// 逐飞库 PWM 占空比分辨率通常 0~10000
#define MOTOR_MAX_DUTY          10000

// ======================= TB6612 接法说明 =======================
// 你现在是：PWMA/PWMB 拉高常开，AIN1/AIN2/BIN1/BIN2 用 PWM 做调速 + 方向
// 所以这里的 4 路 PWM 就是接到 AIN1/AIN2/BIN1/BIN2 的那四个 IO

// 左电机 PWM (P6.0 / P6.4)
#define MOTOR_L_IN1_PWM         PWMA_CH1P_P60
#define MOTOR_L_IN2_PWM         PWMA_CH3P_P64

// 右电机 PWM (P6.2 / P6.6)
#define MOTOR_R_IN1_PWM         PWMA_CH2P_P62
#define MOTOR_R_IN2_PWM         PWMA_CH4P_P66

// ======================= 方向修正（救命开关） =======================
// 如果发现“给正PWM车反着跑”，不用换线，直接把对应改成 -1
#define MOTOR_L_DIR             (1)     // 1 或 -1
#define MOTOR_R_DIR             (1)     // 1 或 -1

// ======================= 死区补偿 =======================
// 小于死区 => 直接输出 0（避免抖动时跳死区）
// 大于死区 => 再加补偿
#define MOTOR_DEAD_ZONE         300     // 约 3%

// ======================= 0输出模式 =======================
// 0: COAST（两路=0）更稳，适合平衡
// 1: BRAKE（两路=高）刹车更硬，可能抖
#define MOTOR_ZERO_MODE         0
#define MOTOR_BRAKE_DUTY        MOTOR_MAX_DUTY

// ======================= 编码器定义 =======================
// 左编码器 (硬件 Timer1: P3.4 / P3.5)
#define ENCODER_L_TIM           TIM_1
#define ENCODER_L_A             IO_P34
#define ENCODER_L_B             IO_P35

// 右编码器 (软件模拟: P0.4 / P5.3)
#define ENCODER_R_A_PIN         IO_P04
#define ENCODER_R_B_PIN         IO_P53

// ======================= API =======================
void Motor_Init(void);
void Motor_Stop(void);

void Motor_Set_L(int16 duty);
void Motor_Set_R(int16 duty);
void Motor_Set(int16 duty_l, int16 duty_r);

// 软件编码器扫描（若你后面要用右轮编码器：必须在固定周期调用）
void Soft_Encoder_Scan(void);

// 周期性调用（比如 5ms 一次）：读出计数并清零
void Encoder_Get_Val(int16 *L, int16 *R);

#endif
