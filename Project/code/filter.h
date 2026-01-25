/*
 * @file    filter.h
 * @brief   常用信号处理工具
 *
 * 包含功能：
 *  1. 一阶低通滤波（LPF）
 *  2. 限幅函数
 *  3. 死区处理
 *
 * 主要用于：
 *  - IMU 原始数据滤波
 *  - 控制量平滑
 */

#ifndef _FILTER_H_
#define _FILTER_H_

#include "zf_common_headfile.h"

// 一阶低通滤波器结构体
typedef struct
{
    float alpha;   // 滤波系数，0~1，越大越平滑
    float y;       // 当前输出
    uint8 inited;  // 初始化标志
} lpf1_t;


// 一阶低通
void lpf1_init(lpf1_t *f, float alpha, float init_val);
float lpf1_update(lpf1_t *f, float x);

// 限幅/死区工具
float limit_f(float x, float minv, float maxv);
float deadzone_f(float x, float dz);

#endif
