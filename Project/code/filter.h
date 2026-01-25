#ifndef _FILTER_H_
#define _FILTER_H_

#include "zf_common_headfile.h"

typedef struct
{
    float alpha;     // 0~1，越大越“跟手”
    float y;
    uint8 inited;
} lpf1_t;

// 一阶低通
void lpf1_init(lpf1_t *f, float alpha, float init_val);
float lpf1_update(lpf1_t *f, float x);

// 限幅/死区工具
float limit_f(float x, float minv, float maxv);
float deadzone_f(float x, float dz);

#endif
