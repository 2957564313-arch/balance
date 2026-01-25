#include "zf_common_headfile.h"

#include "OLED.h"

/* 采样频率与周期 */
#define IMU_HZ          (100.0f)
#define IMU_PERIOD_MS   (10)

/* 滤波参数：alpha 越大越平滑(越慢)，越小越跟手(越快) */
#define ACC_LPF_ALPHA   (0.70f)
#define GYRO_LPF_ALPHA  (0.85f)

/* 角度换算 */
#define DEG2RAD         (0.01745329252f)

static imu_data_t imu_raw;
static imu_data_t imu_flt;
static mahony_t   mahony;

/* 一阶低通滤波器 */
static lpf1_t lpf_ax, lpf_ay, lpf_az;
static lpf1_t lpf_gx, lpf_gy, lpf_gz;

static void imu_filter_init(void)
{
    lpf1_init(&lpf_ax, ACC_LPF_ALPHA, 0.0f);
    lpf1_init(&lpf_ay, ACC_LPF_ALPHA, 0.0f);
    lpf1_init(&lpf_az, ACC_LPF_ALPHA, 0.0f);

    lpf1_init(&lpf_gx, GYRO_LPF_ALPHA, 0.0f);
    lpf1_init(&lpf_gy, GYRO_LPF_ALPHA, 0.0f);
    lpf1_init(&lpf_gz, GYRO_LPF_ALPHA, 0.0f);
}

static void imu_filter_update(const imu_data_t *in, imu_data_t *out)
{
    out->ax = lpf1_update(&lpf_ax, in->ax);
    out->ay = lpf1_update(&lpf_ay, in->ay);
    out->az = lpf1_update(&lpf_az, in->az);

    out->gx = lpf1_update(&lpf_gx, in->gx);
    out->gy = lpf1_update(&lpf_gy, in->gy);
    out->gz = lpf1_update(&lpf_gz, in->gz);
}

static void oled_show_attitude(const mahony_t *m)
{
    OLED_ShowString(1, 1, "P:");
    OLED_ShowFloat(1, 3, m->pitch, 3, 2);

    OLED_ShowString(2, 1, "R:");
    OLED_ShowFloat(2, 3, m->roll, 3, 2);

    OLED_ShowString(3, 1, "Y:");
    OLED_ShowFloat(3, 3, m->yaw, 3, 2);

    OLED_Update();
}

void main(void)
{
    clock_init(SYSTEM_CLOCK_30M);
    debug_init();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "IMU INIT...");
    OLED_Update();

    /* MPU6050 初始化与陀螺零偏标定(上电静止) */
    imu_mpu6050_init();
    system_delay_ms(200);
    imu_mpu6050_gyro_calibrate(300);

    /* 滤波器与 Mahony 初始化*/
    imu_filter_init();
    mahony_init(&mahony, IMU_HZ, 5.0f, 0.05f);

    OLED_Clear();
    OLED_ShowString(1, 1, "P:");
    OLED_ShowString(2, 1, "R:");
    OLED_ShowString(3, 1, "Y:");
    OLED_Update();

    while (1)
    {
        /* 1) 读取 MPU6050（加速度单位:g，陀螺单位:deg/s） */
        imu_mpu6050_read(&imu_raw);

        /* 2) 一阶低通滤波 */
        imu_filter_update(&imu_raw, &imu_flt);

        /* 3) Mahony 姿态解算：陀螺输入必须是 rad/s */
        mahony_update(&mahony,
                      imu_flt.gx * DEG2RAD,
                      imu_flt.gy * DEG2RAD,
                      imu_flt.gz * DEG2RAD,
                      imu_flt.ax,
                      imu_flt.ay,
                      imu_flt.az);

        /* 4) OLED 显示三轴角度 */
        oled_show_attitude(&mahony);

        /* 5) 保证采样周期 */
        system_delay_ms(IMU_PERIOD_MS);
    }
}
