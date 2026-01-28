#include "imu_mpu6050.h"
#include "zf_driver_soft_iic.h" 

// 定义全局变量 (与 .h 保持一致)
int16 mpu6050_gyro_x = 0, mpu6050_gyro_y = 0, mpu6050_gyro_z = 0;
int16 mpu6050_acc_x = 0, mpu6050_acc_y = 0, mpu6050_acc_z = 0;

#if MPU6050_USE_SOFT_IIC
static soft_iic_info_struct mpu6050_iic_struct;

// 宏封装：方便调用逐飞库的 Soft I2C
#define mpu6050_write_register(reg, dat)       (soft_iic_write_8bit_register(&mpu6050_iic_struct, (reg), (dat)))
#define mpu6050_read_register(reg)             (soft_iic_read_8bit_register(&mpu6050_iic_struct, (reg)))
#define mpu6050_read_registers(reg, dat, len)  (soft_iic_read_8bit_registers(&mpu6050_iic_struct, (reg), (dat), (len)))

#else
// 硬件I2C预留
#endif

// 自检函数
static uint8 mpu6050_self_check(void)
{
    uint8 dat = 0;
    uint16 timeout = 0;
    
    // 唤醒并读取寄存器测试
    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00); 
    mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07); 
    
    // 简单的自旋等待，不依赖外部延时函数
    while(0x07 != dat)
    {
        if(timeout++ > MPU6050_TIMEOUT_COUNT) return 1; // 超时失败
        dat = mpu6050_read_register(MPU6050_SMPLRT_DIV);
        { volatile uint16 i=1000; while(i--); } // 简单延时
    }
    return 0; // 成功
}

// 初始化
uint8 mpu6050_init(void)
{
#if MPU6050_USE_SOFT_IIC
    // 初始化软件 I2C 引脚
    soft_iic_init(&mpu6050_iic_struct, MPU6050_DEV_ADDR, MPU6050_SOFT_IIC_DELAY, MPU6050_SCL_PIN, MPU6050_SDA_PIN);
#endif
    
    // 上电延时
    { volatile uint32 i=50000; while(i--); }

    if(mpu6050_self_check()) return 1; // 自检失败

    // === 核心配置 ===
    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00); // 解除休眠
    
    // 采样分频设为 0 -> 1kHz 输出
    mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x00); 
    
    // DLPF 20Hz (平滑数据，适合平衡车)
    mpu6050_write_register(MPU6050_CONFIG, 0x04);      
    
    mpu6050_write_register(MPU6050_GYRO_CONFIG, MPU6050_GYR_SAMPLE); // ±2000dps
    mpu6050_write_register(MPU6050_ACCEL_CONFIG, MPU6050_ACC_SAMPLE); // ±8g
    
    return 0;
}

void mpu6050_get_acc(void)
{
    uint8 dat[6];
    mpu6050_read_registers(MPU6050_ACCEL_XOUT_H, dat, 6);
    mpu6050_acc_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    mpu6050_acc_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    mpu6050_acc_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

void mpu6050_get_gyro(void)
{
    uint8 dat[6];
    mpu6050_read_registers(MPU6050_GYRO_XOUT_H, dat, 6);
    mpu6050_gyro_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    mpu6050_gyro_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    mpu6050_gyro_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

// 转换函数：Raw -> g
float mpu6050_acc_transition(int16 acc_value)
{
    // ±8g 量程 -> 4096 LSB/g
    return (float)acc_value / 4096.0f;
}

// 转换函数：Raw -> deg/s
float mpu6050_gyro_transition(int16 gyro_value)
{
    // ±2000dps 量程 -> 16.4 LSB/(deg/s)
    return (float)gyro_value / 16.4f;
}
