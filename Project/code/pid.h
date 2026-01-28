#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

// 函数声明
void PID_Init(void);
void PID_Clear_Integral(void);

// === 串级 PID 核心 ===

// 1. 直立环 (内环)：输入当前角度和目标角度，输出电机 PWM
int16 PID_Vertical(float angle, float gyro_y, float target_angle);

// 2. 速度环 (外环)：输入目标速度，输出目标角度 (float)
float PID_Velocity(int16 target_speed, int16 current_speed);

// 3. 转向环：保持不变
int16 PID_Turn(int16 target_turn, int16 gyro_z);

#endif
