#include "imu_mpu6050.h"
#include "zf_driver_soft_iic.h" 
#include "OLED.h" 

// 全局变量
int16 mpu6050_gyro_x = 0, mpu6050_gyro_y = 0, mpu6050_gyro_z = 0;
int16 mpu6050_acc_x = 0, mpu6050_acc_y = 0, mpu6050_acc_z = 0;
int16 gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

#if MPU6050_USE_SOFT_IIC
static soft_iic_info_struct mpu6050_iic_struct;
#define mpu6050_write_register(reg, dat)       (soft_iic_write_8bit_register(&mpu6050_iic_struct, (reg), (dat)))
#define mpu6050_read_register(reg)             (soft_iic_read_8bit_register(&mpu6050_iic_struct, (reg)))
#define mpu6050_read_registers(reg, dat, len)  (soft_iic_read_8bit_registers(&mpu6050_iic_struct, (reg), (dat), (len)))
#endif

// 自检
static uint8 mpu6050_self_check(void) {
    uint8 dat = 0;
    uint16 timeout = 0;
    volatile uint16 i; 

    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00); 
    mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07); 
    
    while(0x07 != dat) {
        if(timeout++ > 200) return 1; 
        dat = mpu6050_read_register(MPU6050_SMPLRT_DIV);
        i = 1000; while(i--); 
    }
    return 0;
}

// 初始化
uint8 mpu6050_init(void) {
    volatile uint32 i; 

#if MPU6050_USE_SOFT_IIC
    soft_iic_init(&mpu6050_iic_struct, MPU6050_DEV_ADDR, MPU6050_SOFT_IIC_DELAY, MPU6050_SCL_PIN, MPU6050_SDA_PIN);
#endif
    
    i = 50000; while(i--);

    if(mpu6050_self_check()) return 1;

    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00); 
    mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x00); 
    mpu6050_write_register(MPU6050_CONFIG, 0x03);      
    mpu6050_write_register(MPU6050_GYRO_CONFIG, MPU6050_GYR_SAMPLE); 
    mpu6050_write_register(MPU6050_ACCEL_CONFIG, MPU6050_ACC_SAMPLE); 
    
    return 0;
}

// 零偏校准 (1000次循环，高精度)
void mpu6050_calibration(void) {
    uint16 i;
    int32 sum_x = 0, sum_y = 0, sum_z = 0;
    volatile uint16 k; 
    
    gyro_offset_x = 0;
    gyro_offset_y = 0;
    gyro_offset_z = 0;
    
    // 循环次数从200增加到1000
    for(i = 0; i < 1000; i++) {
        mpu6050_get_gyro(); 
        
        sum_x += mpu6050_gyro_x; 
        sum_y += mpu6050_gyro_y;
        sum_z += mpu6050_gyro_z;
        
        k = 1000; while(k--); // 短暂延时
    }
    
    gyro_offset_x = (int16)(sum_x / 1000);
    gyro_offset_y = (int16)(sum_y / 1000);
    gyro_offset_z = (int16)(sum_z / 1000);
}

void mpu6050_get_acc(void) {
    uint8 dat[6];
    mpu6050_read_registers(MPU6050_ACCEL_XOUT_H, dat, 6);
    
    mpu6050_acc_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    mpu6050_acc_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    mpu6050_acc_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

void mpu6050_get_gyro(void) {
    uint8 dat[6];
    int16 raw_x, raw_y, raw_z; 
    
    mpu6050_read_registers(MPU6050_GYRO_XOUT_H, dat, 6);
    
    raw_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    raw_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    raw_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
    
    mpu6050_gyro_x = raw_x - gyro_offset_x;
    mpu6050_gyro_y = raw_y - gyro_offset_y;
    mpu6050_gyro_z = raw_z - gyro_offset_z;
}

float mpu6050_acc_transition(int16 acc_value) {
    return (float)acc_value / 4096.0f;
}

float mpu6050_gyro_transition(int16 gyro_value) {
    return (float)gyro_value / 16.4f;
}
