#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "zf_common_headfile.h"

typedef struct
{
    float q0, q1, q2, q3; // 四元数
    float pitch, roll, yaw; // 欧拉角 (-180 ~ 180)
    
    // === 新增：累积航向角 ===
    // 这个变量记录车子转过的总角度（绝对值累加）
    // 比如：进弯时是 0，出弯时应该是 180
    float total_yaw; 
    
    float ix, iy, iz; 
    float kp, ki;
    float sample_freq;
} mahony_t;

extern mahony_t m_imu;

void mahony_init(mahony_t *m, float freq, float kp, float ki);
void mahony_update(mahony_t *m, float ax, float ay, float az, float gx, float gy, float gz);

#endif
