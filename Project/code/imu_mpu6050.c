#include "imu_mpu6050.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_delay.h"

#define MPU_PWR_MGMT1   0x6B
#define MPU_GYRO_CFG    0x1B
#define MPU_ACC_CFG     0x1C
#define MPU_ACC_XH      0x3B

static soft_iic_info_struct mpu_iic;

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static void mpu_write(uint8 reg, uint8 dat)
{
    soft_iic_write_8bit_register(&mpu_iic, reg, dat);
}

static void mpu_read_buf(uint8 reg, uint8 *buf, uint8 len)
{
    soft_iic_read_8bit_registers(&mpu_iic, reg, buf, len);
}

uint8 imu_mpu6050_init(void)
{
    soft_iic_init(&mpu_iic, MPU6050_ADDR, MPU6050_IIC_DELAY,
                  MPU6050_SCL_PIN, MPU6050_SDA_PIN);

    system_delay_ms(50);

    mpu_write(MPU_PWR_MGMT1, 0x01);
    mpu_write(MPU_GYRO_CFG,  0x18);   // ¡À2000 dps
    mpu_write(MPU_ACC_CFG,   0x00);   // ¡À2g

    system_delay_ms(10);
    return 1;
}

void imu_mpu6050_read(imu_data_t *imu)
{
    uint8 buf[14];
    int16 ax, ay, az;
    int16 gx, gy, gz;

    if(imu == 0) return;

    mpu_read_buf(MPU_ACC_XH, buf, 14);

    ax = (int16)((buf[0] << 8) | buf[1]);
    ay = (int16)((buf[2] << 8) | buf[3]);
    az = (int16)((buf[4] << 8) | buf[5]);

    gx = (int16)((buf[8]  << 8) | buf[9]);
    gy = (int16)((buf[10] << 8) | buf[11]);
    gz = (int16)((buf[12] << 8) | buf[13]);

    imu->ax = (float)ax / 16384.0f;
    imu->ay = (float)ay / 16384.0f;
    imu->az = (float)az / 16384.0f;

    imu->gx = (float)gx / 16.4f - gyro_bias_x;
    imu->gy = (float)gy / 16.4f - gyro_bias_y;
    imu->gz = (float)gz / 16.4f - gyro_bias_z;
}

void imu_mpu6050_gyro_calibrate(uint16 cnt)
{
    uint16 i;
    imu_data_t imu;
    float sx = 0.0f;
    float sy = 0.0f;
    float sz = 0.0f;

    for(i = 0; i < cnt; i++)
    {
        imu_mpu6050_read(&imu);
        sx += imu.gx;
        sy += imu.gy;
        sz += imu.gz;
        system_delay_ms(5);
    }

    gyro_bias_x = sx / cnt;
    gyro_bias_y = sy / cnt;
    gyro_bias_z = sz / cnt;
}
