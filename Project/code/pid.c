//#include "pid.h"
//#include "param.h" 
//// #include "filter.h"  <-- 这句可以删掉，因为 pid.h 包含了 headfile.h，里面已经有 filter.h 了
//// 如果删掉报错，就留着，不影响

//static float velocity_integral = 0;

//#define MAX_TARGET_ANGLE  12.0f 

//void PID_Init(void)
//{
//    velocity_integral = 0;
//}

//void PID_Clear_Integral(void)
//{
//    velocity_integral = 0;
//}

//// 直立环
//int16 PID_Vertical(float angle, float gyro_y, float target_angle)
//{
//    float pwm_out;
//    // 目标角度 = 机械中值 + 速度环输出
//    float final_target = g_sys_param.mech_zero_pitch + target_angle;
//    
//    // 简单的 PD 控制
//    pwm_out = g_sys_param.balance_kp * (angle - final_target) + g_sys_param.balance_kd * gyro_y;
//    
//    return (int16)pwm_out;
//}

//// 速度环
//float PID_Velocity(int16 target_speed, int16 current_speed)
//{
//    int16 error;
//    float angle_out;
//    
//    error = target_speed - current_speed;
//    
//    velocity_integral += error * g_sys_param.velocity_ki;
//    
//    // 这里调用 filter.c 里的 limit_f
//    velocity_integral = limit_f(velocity_integral, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);
//    
//    angle_out = error * g_sys_param.velocity_kp + velocity_integral;
//    
//    // 这里调用 filter.c 里的 limit_f
//    angle_out = limit_f(angle_out, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);
//    
//    return angle_out;
//}

//// 转向环
//int16 PID_Turn(int16 target_turn, int16 gyro_z)
//{
//    float pwm_out;
//    pwm_out = (float)target_turn + g_sys_param.turn_kd * gyro_z;
//    return (int16)pwm_out;
//}
