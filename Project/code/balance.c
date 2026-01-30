#include "balance.h"
#include "imu_mpu6050.h" 
#include "filter.h" 
#include "param.h" 
#include "bluetooth.h" 

// 全局变量
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float g_car_x = 0.0f, g_car_y = 0.0f;      

void Balance_Init(void) {
    pitch = 0.0f; roll = 0.0f; yaw = 0.0f;
    g_car_x = 0.0f; g_car_y = 0.0f;
    // 初始化滤波算法
    Filter_Init(200.0f);
}

void Balance_Task(void) {
    float gx, gy, gz, ax, ay, az;

    // 1. 获取原始数据
    mpu6050_get_gyro(); 
    mpu6050_get_acc();  
    
    // 2. 转换物理量
    gx = mpu6050_gyro_transition(mpu6050_gyro_x);
    gy = mpu6050_gyro_transition(mpu6050_gyro_y);
    gz = mpu6050_gyro_transition(mpu6050_gyro_z);
    ax = mpu6050_acc_transition(mpu6050_acc_x);
    ay = mpu6050_acc_transition(mpu6050_acc_y);
    az = mpu6050_acc_transition(mpu6050_acc_z);

    // 3. 姿态解算
    Filter_Update(gx, gy, gz, ax, ay, az);
    Filter_GetEuler(&pitch, &roll, &yaw);
    
    // 暂时不写控制代码，先解决报错
}
