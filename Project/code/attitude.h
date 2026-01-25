/*
 * @file    attitude.h
 * @brief   姿态解算模块（Mahony 互补滤波，无磁力计）
 *
 * 功能说明：
 *  1. 输入：陀螺仪角速度 + 加速度
 *  2. 输出：Pitch / Roll / Yaw（角度制）
 *
 * 使用说明：
 *  1. 初始化时设置采样频率和参数
 *  2. 周期性调用 update 函数
 *  3. 直接读取结构体中的姿态角
 *
 * 适用场景：
 *  - 平衡车姿态估计
 *  - 姿态显示（OLED）
 */

#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "zf_common_headfile.h"

typedef struct
{
    float q0, q1, q2, q3;
    float pitch;
    float roll;
    float yaw;

    float kp;
    float ki;

    float ix;
    float iy;
    float iz;

    float sample_freq;
} mahony_t;

void mahony_init(mahony_t *m, float freq, float kp, float ki);
void mahony_update(mahony_t *m,
                   float gx, float gy, float gz,
                   float ax, float ay, float az);

#endif

