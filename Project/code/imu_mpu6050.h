/*
 * @file    imu_mpu6050.h
 * @brief   MPU6050 IMU 驱动（软件 IIC）
 *
 * 功能说明：
 *  1. 初始化 MPU6050（量程、时钟）
 *  2. 读取加速度（g）和角速度（deg/s）
 *  3. 支持陀螺仪零偏校准
 *
 * 使用说明：
 *  1. 上电后调用 imu_mpu6050_init()
 *  2. 静止状态下调用 imu_mpu6050_gyro_calibrate()
 *  3. 主循环周期性调用 imu_mpu6050_read()
 *
 * 注意事项：
 *  - 该文件不做姿态解算
 *  - 姿态解算由 attitude / filter 模块完成
 */

#ifndef _IMU_MPU6050_H_
#define _IMU_MPU6050_H_

#include "zf_common_headfile.h"

// ===== 按你实际接线修改 =====
#define MPU6050_SCL_PIN     (IO_P40)
#define MPU6050_SDA_PIN     (IO_P41)
#define MPU6050_IIC_DELAY  (80)
// ============================

#define MPU6050_ADDR       (0x68)

typedef struct
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} imu_data_t;

uint8 imu_mpu6050_init(void);

// 读取 MPU6050 原始数据并换算为物理量（g / deg/s）
void  imu_mpu6050_read(imu_data_t *imu);

void  imu_mpu6050_gyro_calibrate(uint16 cnt);

#endif
