#include "pid.h"
#include "param.h" 
#include "filter.h" // 需要 limit_f 函数

static float velocity_integral = 0;

// 最大目标倾角 (度)
// 限制速度环最多只能让车倾斜多少度，防止飞车
#define MAX_TARGET_ANGLE  12.0f 

void PID_Init(void)
{
    velocity_integral = 0;
}

void PID_Clear_Integral(void)
{
    velocity_integral = 0;
}

// ====================================================================
// 串级 PID：直立环 (内环)
// 作用：维持车身在 "目标角度" 上
// ====================================================================
// angle: 当前真实角度
// gyro_y: 当前角速度
// target_angle: 由速度环计算出的期望倾角 (叠加在机械零点上)

int16 PID_Vertical(float angle, float gyro_y, float target_angle)
{
    float pwm_out;

    // 串级核心公式：
    // 误差 = 当前角度 - (机械中值 + 速度环要求的倾角)
    // 这样，当速度环想要加速时，它会给出一个负的角度(前倾)，
    // 直立环为了消除误差，就会控制电机向前转。
    float final_target = g_sys_param.mech_zero_pitch + target_angle;

    // PD 控制
    pwm_out = g_sys_param.balance_kp * (angle - final_target) + g_sys_param.balance_kd * gyro_y;

    return (int16)pwm_out;
}

// ====================================================================
// 串级 PID：速度环 (外环)
// 作用：根据速度误差，计算出车身应该 "倾斜多少度"
// ====================================================================
// 返回值：float 类型的目标角度

float PID_Velocity(int16 target_speed, int16 current_speed)
{
    int16 error;
    float angle_out;

    // 1. 计算误差
    error = target_speed - current_speed;

    // 2. 积分 (带限幅)
    // 注意：这里的积分限幅值要小很多，因为输出是角度
    velocity_integral += error * g_sys_param.velocity_ki; // 将Ki乘在积分里更方便控制

    // 积分限幅：防止积出几十度的倾角
    velocity_integral = limit_f(velocity_integral, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE); 

    // 3. PI 计算
    // 输出的是角度，不是PWM！
    angle_out = error * g_sys_param.velocity_kp + velocity_integral;

    // 4. 总输出限幅
    // 这一步非常重要！决定了车子最快能跑多快(由最大倾角决定)
    angle_out = limit_f(angle_out, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);

    return angle_out;
}

// 转向环 (保持不变)
int16 PID_Turn(int16 target_turn, int16 gyro_z)
{
    float pwm_out;
    pwm_out = (float)target_turn + g_sys_param.turn_kd * gyro_z;
    return (int16)pwm_out;
}
