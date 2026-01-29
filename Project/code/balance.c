#include "balance.h"
#include "motor.h"
#include "pid.h"
#include "param.h"
#include "imu_mpu6050.h" 
#include "attitude.h"
#include "mode.h" 
#include "Buzzer.h" 

// 定义全局姿态变量 Pitch (供外部调用)
float pitch = 0.0f;

// 内部变量：串级控制中间量
static float target_tilt_angle = 0.0f; // 速度环计算出的"目标倾角"
static uint8 velocity_div_count = 0;   // 速度环分频计数器

// 常量定义
#define RAD_TO_DEG 57.29578f
#define DEG_TO_RAD 0.0174533f

// =================================================================
// 平衡控制初始化
// =================================================================
void Balance_Init(void)
{
    // 1. 初始化 PID
    PID_Init();
    // 如果 Flash 参数未初始化，加载默认值
    if(g_sys_param.balance_kp < 0.1f) Param_SetDefaults();
    
    // 2. 初始化 MPU6050
    mpu6050_init(); 
    
    // 3. 初始化 Mahony 姿态解算
    // 采样频率 200Hz, Kp=0.5, Ki=0.0 (经验值)
    mahony_init(&m_imu, 200.0f, 0.5f, 0.0f); 
}

// =================================================================
// 平衡控制任务 (必须每 5ms 执行一次)
// =================================================================
void Balance_Task(void)
{
    // C251 变量定义置顶
    int16 speed_l, speed_r, speed_avg;
    int16 vertical_pwm, turn_pwm;
    int16 motor_out_l, motor_out_r;
    float gyro_x_rad, gyro_y_rad, gyro_z_rad;
    float acc_x_g, acc_y_g, acc_z_g;

    // -----------------------------------------------------------
    // 1. 获取传感器原始数据
    // -----------------------------------------------------------
    mpu6050_get_gyro(); 
    mpu6050_get_acc();  
    
    // -----------------------------------------------------------
    // 2. 数据转换 (关键修正：转为 rad/s 供 Mahony 使用)
    // -----------------------------------------------------------
    acc_x_g = mpu6050_acc_transition(mpu6050_acc_x);
    acc_y_g = mpu6050_acc_transition(mpu6050_acc_y);
    acc_z_g = mpu6050_acc_transition(mpu6050_acc_z);
    
    // 【修正】将 deg/s 转换为 rad/s
    gyro_x_rad = mpu6050_gyro_transition(mpu6050_gyro_x) * DEG_TO_RAD;
    gyro_y_rad = mpu6050_gyro_transition(mpu6050_gyro_y) * DEG_TO_RAD;
    gyro_z_rad = mpu6050_gyro_transition(mpu6050_gyro_z) * DEG_TO_RAD;

    // -----------------------------------------------------------
    // 3. 姿态解算 (Mahony 算法)
    // -----------------------------------------------------------
    // 必须传入 rad/s，否则解算会发散
    mahony_update(&m_imu, 
                  acc_x_g, acc_y_g, acc_z_g, 
                  gyro_x_rad, gyro_y_rad, gyro_z_rad);
    
    // 获取最新的 Pitch 角度 (mahony 内部已转回角度制)
    pitch = m_imu.pitch;

    // -----------------------------------------------------------
    // 4. 逻辑调度
    // -----------------------------------------------------------
    Mode_Handler(); // 计算 target_speed_val 和 target_turn_val
    Buzzer_Task();  // 刷新蜂鸣器

    // -----------------------------------------------------------
    // 5. 安全保护 (倒地停车)
    // -----------------------------------------------------------
    if ((pitch > 45.0f || pitch < -45.0f) || 
        (current_mode == MODE_4_REPLAY && path_state == REC_ON))
    {
        Motor_Set_L(0);
        Motor_Set_R(0);
        PID_Clear_Integral(); // 倒地后清除积分
        return; 
    }

    // -----------------------------------------------------------
    // 6. 串级 PID 计算
    // -----------------------------------------------------------
    
    // === A. 速度环 (外环) ===
    // 降频处理：每 20ms (4 * 5ms) 执行一次
    velocity_div_count++;
    if(velocity_div_count >= 4)
    {
        velocity_div_count = 0;
        
        Encoder_Get_Val(&speed_l, &speed_r);
        speed_avg = (speed_l + speed_r) / 2;
        
        // 【串级核心】计算目标倾角 (输出的是 float 角度)
        // 输入：目标速度，当前速度
        // 输出：target_tilt_angle (例如：为了加速，需要前倾 3.5 度)
        target_tilt_angle = PID_Velocity(target_speed_val, speed_avg);
    }
    
    // === B. 直立环 (内环) ===
    // 每 5ms 执行一次
    // 【串级核心】目标角度 = 速度环输出 + 机械中值
    // 传入参数：当前角度, 当前角速度(deg/s), 目标角度
    // 注意：PID_Vertical 内部使用的是 deg/s，所以这里传入 transition 后的原始值(未转rad)
    vertical_pwm = PID_Vertical(pitch, mpu6050_gyro_transition(mpu6050_gyro_y), target_tilt_angle);
    
    // === C. 转向环 ===
    // 修正：传入原始数据给 PID_Turn (它只做简单比例控制)
    turn_pwm = PID_Turn(target_turn_val, mpu6050_gyro_z);

    // -----------------------------------------------------------
    // 7. 电机输出合成
    // -----------------------------------------------------------
    motor_out_l = vertical_pwm + turn_pwm;
    motor_out_r = vertical_pwm - turn_pwm;
    
    Motor_Set_L(motor_out_l);
    Motor_Set_R(motor_out_r);
}
