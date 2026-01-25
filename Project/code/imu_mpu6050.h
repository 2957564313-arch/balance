#ifndef _IMU_MPU6050_H_
#define _IMU_MPU6050_H_

#include "zf_common_headfile.h"

// ===== 按你实际接线修改 =====
#define MPU6050_SCL_PIN     (P33)
#define MPU6050_SDA_PIN     (P32)
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
void  imu_mpu6050_read(imu_data_t *imu);     // ?? 不再用 data
void  imu_mpu6050_gyro_calibrate(uint16 cnt);

#endif
