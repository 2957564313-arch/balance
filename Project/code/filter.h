#ifndef __FILTER_H
#define __FILTER_H

#include "zf_common_headfile.h"

// 初始化滤波器
void Filter_Init(float sampleFreqHz);

// 核心更新函数 (传入 deg/s 和 g)
void Filter_Update(float gx_dps, float gy_dps, float gz_dps,
                   float ax_g,  float ay_g,  float az_g);

// 获取欧拉角
void Filter_GetEuler(float *pitch_deg, float *roll_deg, float *yaw_deg);

// 简单的数学工具 (供 balance.c 使用)
float limit_f(float x, float minv, float maxv);
float deadzone_f(float x, float dz);

#endif
