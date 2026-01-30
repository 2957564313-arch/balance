
//#ifndef _zf_device_imu660ra_h_
//#define _zf_device_imu660ra_h_

//#include "zf_common_typedef.h"
//#include "zf_device_type.h"

//#define IMU660RA_USE_INTERFACE       SOFT_SPI                        // 默认使用硬件 SPI 方式驱动
//#if (IMU660RA_USE_INTERFACE==HARDWARE_SPI)
////====================================================硬件 SPI 驱动====================================================
//	#define IMU660RA_SPI_SPEED          ((uint32)10 * 1000 * 1000U)  	// 硬件 SPI 速率
//	#define IMU660RA_SPI                SPI_0                        	// 硬件 SPI 号
//	#define IMU660RA_SPC_PIN            SPI0_CH3_SCLK_P43             	// 硬件 SPI SCK 引脚
//	#define IMU660RA_SDI_PIN            SPI0_CH3_MOSI_P40             	// 硬件 SPI MOSI 引脚
//	#define IMU660RA_SDO_PIN            SPI0_CH3_MISO_P41              	// 硬件 SPI MISO 引脚
//	#define IMU660RA_CS_PIN             (IO_P42)                       	// CS 片选引脚
//	#define IMU660RA_CS(x)              (P42 = x)
////====================================================硬件 SPI 驱动====================================================
//#elif (IMU660RA_USE_INTERFACE==SOFT_SPI)
////====================================================软件 SPI 驱动====================================================
//	#define IMU660RA_SPC_PIN            (P40)                           // 软件 SPI SCK  引脚
//	#define IMU660RA_SDI_PIN            (P41)                           // 软件 SPI MOSI 引脚
//	#define IMU660RA_SDO_PIN            (P42)                           // 软件 SPI MISO 引脚
//	#define IMU660RA_CS_PIN             (P43)                        	// 软件 SPI CS   引脚
////====================================================软件 SPI 驱动====================================================
//#elif (IMU660RA_USE_INTERFACE==SOFT_IIC)
////====================================================软件 IIC 驱动====================================================
//	#define IMU660RA_SOFT_IIC_DELAY     (0)                             // 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
//	#define IMU660RA_SCL_PIN            (IO_P40)                        // 软件 IIC SCL 引脚 连接 IMU660RA 的 SCL 引脚
//	#define IMU660RA_SDA_PIN            (IO_P41)                        // 软件 IIC SDA 引脚 连接 IMU660RA 的 SDA 引脚
////====================================================软件 IIC 驱动====================================================
//#endif


//typedef enum
//{
//    IMU660RA_ACC_SAMPLE_SGN_2G,                                                 // 加速度计量程 ±2G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
//    IMU660RA_ACC_SAMPLE_SGN_4G,                                                 // 加速度计量程 ±4G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
//    IMU660RA_ACC_SAMPLE_SGN_8G,                                                 // 加速度计量程 ±8G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
//    IMU660RA_ACC_SAMPLE_SGN_16G,                                                // 加速度计量程 ±16G (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
//} imu660ra_acc_sample_config;

//typedef enum
//{
//    IMU660RA_GYRO_SAMPLE_SGN_125DPS,                                            // 陀螺仪量程 ±125DPS  (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
//    IMU660RA_GYRO_SAMPLE_SGN_250DPS,                                            // 陀螺仪量程 ±250DPS  (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
//    IMU660RA_GYRO_SAMPLE_SGN_500DPS,                                            // 陀螺仪量程 ±500DPS  (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
//    IMU660RA_GYRO_SAMPLE_SGN_1000DPS,                                           // 陀螺仪量程 ±1000DPS (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
//    IMU660RA_GYRO_SAMPLE_SGN_2000DPS,                                           // 陀螺仪量程 ±2000DPS (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
//} imu660ra_gyro_sample_config;

//#define IMU660RA_ACC_SAMPLE_DEFAULT     ( IMU660RA_ACC_SAMPLE_SGN_8G )          // 在这设置默认的 加速度计 初始化量程
//#define IMU660RA_GYRO_SAMPLE_DEFAULT    ( IMU660RA_GYRO_SAMPLE_SGN_2000DPS )    // 在这设置默认的 陀螺仪   初始化量程

//#define IMU660RA_TIMEOUT_COUNT      ( 0x001 )                                  	// IMU660RA 超时计数

////================================================定义 IMU660RA 内部地址================================================
//#define IMU660RA_DEV_ADDR           ( 0x69 )                                    // SA0接地：0x68 SA0上拉：0x69 模块默认上拉
//#define IMU660RA_SPI_W              ( 0x00 )
//#define IMU660RA_SPI_R              ( 0x80 )

//#define IMU660RA_CHIP_ID            ( 0x00 )
//#define IMU660RA_PWR_CONF           ( 0x7C )
//#define IMU660RA_PWR_CTRL           ( 0x7D )
//#define IMU660RA_INIT_CTRL          ( 0x59 )
//#define IMU660RA_INIT_DATA          ( 0x5E )
//#define IMU660RA_INT_STA            ( 0x21 )
//#define IMU660RA_ACC_ADDRESS        ( 0x0C )
//#define IMU660RA_GYRO_ADDRESS       ( 0x12 )
//#define IMU660RA_ACC_CONF           ( 0x40 )
//#define IMU660RA_ACC_RANGE          ( 0x41 )
//#define IMU660RA_GYR_CONF           ( 0x42 )
//#define IMU660RA_GYR_RANGE          ( 0x43 )
////================================================定义 IMU660RA 内部地址================================================

//extern int16 imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;                 // 三轴陀螺仪数据      gyro (陀螺仪)
//extern int16 imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z;                    // 三轴加速度计数据     acc (accelerometer 加速度计)
//extern float imu660ra_transition_factor[2];

//void  imu660ra_get_acc              (void);                                     // 获取 IMU660RA 加速度计数据
//void  imu660ra_get_gyro             (void);                                     // 获取 IMU660RA 陀螺仪数据

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     将 IMU660RA 加速度计数据转换为实际物理数据
//// 参数说明     acc_value       任意轴的加速度计数据
//// 返回参数     void
//// 使用示例     float data = imu660ra_acc_transition(imu660ra_acc_x);           // 单位为 g(m/s^2)
//// 备注信息
////-------------------------------------------------------------------------------------------------------------------
//#define imu660ra_acc_transition(acc_value)      ((float)(acc_value) / imu660ra_transition_factor[0])

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     将 IMU660RA 陀螺仪数据转换为实际物理数据
//// 参数说明     gyro_value      任意轴的陀螺仪数据
//// 返回参数     void
//// 使用示例     float data = imu660ra_gyro_transition(imu660ra_gyro_x);         // 单位为 °/s
//// 备注信息
////-------------------------------------------------------------------------------------------------------------------
//#define imu660ra_gyro_transition(gyro_value)    ((float)(gyro_value) / imu660ra_transition_factor[1])

//uint8 imu660ra_init                 (void);                                     // 初始化 IMU660RA

//#endif

