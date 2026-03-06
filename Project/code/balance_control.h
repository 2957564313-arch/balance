#ifndef _BALANCE_CONTROL_H_
#define _BALANCE_CONTROL_H_

#include "zf_common_headfile.h"
#include <string.h>

typedef struct
{
    int16   *gyro_raw_data;   // 指向陀螺仪原始数据的指针
    int16   *acc_raw_data;    // 指向加速度原始数据的指针

    float   gyro_ration;      // 互补滤波中的陀螺仪滤波信任程度，一般给4
    float   acc_ration;       // 互补滤波中的加速度滤波信任程度，一般给4
    float   call_cycle;       // 互补滤波调用周期，单位秒，一般给0.005（5ms）

    float   angle_temp;       // 互补滤波中保存上一次计算值
    float   mechanical_zero;  // 机械零点
    float   filtering_angle;  // 滤波后的角度,不是完全意义上的角度，还差一个比例换算

} cascade_common_value_struct;

// PID 参数结构体
typedef struct
{
    float   p;  // 比例系数
    float   i;  // 积分系数
    float   d;  // 微分系数

    float   i_value;          // 积分量
    float   i_value_max;      // 积分量最大值
    float   i_value_pro;      // 积分量比例

    float   p_value_last;     // 上一次比例量
    float   incremental_data[2]; // 增量式PID的误差缓存

    float   out;              // PID输出值
    float   out_max;          // PID输出最大值

} pid_cycle_struct;

// 串级参数结构体
typedef struct
{
    cascade_common_value_struct  cascade_value;         // 串级公共参数结构体   

    pid_cycle_struct             angular_speed_cycle;   // 角速度环PID参数结构体
    pid_cycle_struct             angle_cycle;           // 角度环PID参数结构体
    pid_cycle_struct             speed_cycle;           // 速度环PID参数结构体

} cascade_value_struct;

extern cascade_value_struct balance_cascade;
extern cascade_value_struct balance_cascade_resave;

extern cascade_value_struct steer_balance_cascade;
extern cascade_value_struct steer_balance_cascade_resave;

//--------------------------------------------------------------------------------------------------
// 函数简介        一阶互补滤波
// 参数说明        filter_value         滤波参数结构体
// 参数说明        gyro_raw_data        陀螺仪原始数据
// 参数说明        acc_raw_data         加速度原始数据
// 返回参数        void
// 使用示例        first_order_complementary_filtering(&balance_cascade.cascade_value, *balance_cascade.cascade_value.gyro_raw_data, *balance_cascade.cascade_value.acc_raw_data);
// 备注信息
//--------------------------------------------------------------------------------------------------
void first_order_complementary_filtering (cascade_common_value_struct *filter_value, int16 gyro_raw_data, int16 acc_raw_data);

//--------------------------------------------------------------------------------------------------
// 函数简介        PID闭环计算（位置式）
// 参数说明        pid_cycle            PID参数结构体
// 参数说明        target               目标值
// 参数说明        real                 当前值
// 返回参数        void
// 使用示例        pid_control(&balance_cascade.speed_cycle, 0, (left_motor.encoder_data + right_motor.encoder_data) / 2);
// 备注信息
//--------------------------------------------------------------------------------------------------
void pid_control (pid_cycle_struct *pid_cycle, float target, float real);

//--------------------------------------------------------------------------------------------------
// 函数简介        PID闭环计算（位置式，微分先行）
// 参数说明        pid_cycle            PID参数结构体
// 参数说明        target               目标值
// 参数说明        real                 当前值(测量值)
// 返回参数        void
// 使用示例        pid_control_d_lead(&balance_cascade.angle_cycle, target_angle, current_angle);
// 备注信息        D项对测量值求导而非误差，避免目标值突变引起的微分冲击
//--------------------------------------------------------------------------------------------------
void pid_control_d_lead (pid_cycle_struct *pid_cycle, float target, float real);

//--------------------------------------------------------------------------------------------------
// 函数简介        PID闭环计算（位置式，陀螺D项）
// 参数说明        pid_cycle            PID参数结构体
// 参数说明        target               目标值
// 参数说明        real                 当前值(测量值)
// 参数说明        gyro_raw_for_d       用于D项计算的陀螺仪原始数据
// 参数说明        gyro_ration          互补滤波中的陀螺仪滤波信任程度，一般给4
// 参数说明        call_cycle           互补滤波调用周期，单位秒，一般给0.005（5ms）
// 返回参数        void
// 使用示例        pid_control_angle_gyro_d(&balance_cascade.angle_cycle, target_angle, current_angle, *balance_cascade.cascade_value.gyro_raw_data, balance_cascade.cascade_value.gyro_ration, balance_cascade.cascade_value.call_cycle);
// 备注信息        D项对测量值求导而非误差，避免目标值突变引起的微分冲击，且D项直接使用陀螺仪数据估算角度增量，适用于角度环抑振
//--------------------------------------------------------------------------------------------------
void pid_control_angle_gyro_d(pid_cycle_struct *pid_cycle,float target,float real,int16 gyro_raw_for_d,float gyro_ration,float call_cycle);

//--------------------------------------------------------------------------------------------------
// 函数简介        PID闭环计算（增量式）
// 参数说明        pid_cycle            PID参数结构体
// 参数说明        target               目标值
// 参数说明        real                 当前值
// 返回参数        void
// 使用示例        pid_control_incremental(&balance_cascade.speed_cycle, 0, (left_motor.encoder_data + right_motor.encoder_data) / 2);
// 备注信息
//--------------------------------------------------------------------------------------------------
void pid_control_incremental (pid_cycle_struct *pid_cycle, float target, float real);

//--------------------------------------------------------------------------------------------------
// 函数简介        串级平衡算法初始化
// 返回参数        void
// 使用示例        balance_cascade_init();
// 备注信息        设置串级平衡参数
//--------------------------------------------------------------------------------------------------
void balance_cascade_init (void);

#endif
