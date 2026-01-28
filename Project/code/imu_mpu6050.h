#ifndef _imu_mpu6050_h_
#define _imu_mpu6050_h_

#include "zf_common_headfile.h"

#define MPU6050_USE_SOFT_IIC        (1)

#if MPU6050_USE_SOFT_IIC
    #define MPU6050_SOFT_IIC_DELAY  (0)
    // 软件I2C引脚定义 (请根据实际原理图确认!)
    #define MPU6050_SCL_PIN         (IO_P40)
    #define MPU6050_SDA_PIN         (IO_P41)
#endif

#define MPU6050_TIMEOUT_COUNT       (0x00FF)
#define MPU6050_DEV_ADDR            (0xD0 >> 1) 

// 寄存器地址
#define MPU6050_SMPLRT_DIV          (0x19)
#define MPU6050_CONFIG              (0x1A)
#define MPU6050_GYRO_CONFIG         (0x1B)
#define MPU6050_ACCEL_CONFIG        (0x1C)
#define MPU6050_ACCEL_XOUT_H        (0x3B)
#define MPU6050_GYRO_XOUT_H         (0x43)
#define MPU6050_PWR_MGMT_1          (0x6B)
#define MPU6050_WHO_AM_I            (0x75)

// 量程设置
#define MPU6050_ACC_SAMPLE          (0x10) // ±8g
#define MPU6050_GYR_SAMPLE          (0x18) // ±2000dps

// 全局变量声明 (确保这里是 mpu6050_xxx 而不是 imu660ra_xxx)
extern int16 mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z;
extern int16 mpu6050_acc_x, mpu6050_acc_y, mpu6050_acc_z;

// 函数声明
uint8 mpu6050_init(void);
void  mpu6050_get_acc(void);
void  mpu6050_get_gyro(void);
float mpu6050_acc_transition(int16 acc_value);
float mpu6050_gyro_transition(int16 gyro_value);

#endif
