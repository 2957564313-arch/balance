//#include "attitude.h"
//#include <math.h>

//mahony_t m_imu;

//// 快速平方根倒数
//static float inv_sqrt(float x) {
//    float half = 0.5f * x;
//    float y = x;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float*)&i;
//    y = y * (1.5f - half * y * y);
//    return y;
//}

//void mahony_init(mahony_t *m, float freq, float kp, float ki)
//{
//    m->q0 = 1.0f; m->q1 = 0.0f; m->q2 = 0.0f; m->q3 = 0.0f;
//    m->kp = kp; m->ki = ki;
//    m->sample_freq = freq;
//    m->pitch = 0.0f; m->roll = 0.0f; m->yaw = 0.0f;
//    m->total_yaw = 0.0f; 
//}

//void mahony_update(mahony_t *m, float ax, float ay, float az, float gx, float gy, float gz)
//{
//    float recipNorm;
//    float halfvx, halfvy, halfvz;
//    float halfex, halfey, halfez;
//    float qa, qb, qc;
//    
//    // 变量定义在最前 (C251 兼容)
//    float sinp, siny, cosy, sinr, cosr; 
//    
//    // === 1. Mahony 核心算法 ===
//    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
//    {
//        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
//        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

//        halfvx = m->q1 * m->q3 - m->q0 * m->q2;
//        halfvy = m->q0 * m->q1 + m->q2 * m->q3;
//        halfvz = m->q0 * m->q0 - 0.5f + m->q3 * m->q3;

//        halfex = (ay * halfvz - az * halfvy);
//        halfey = (az * halfvx - ax * halfvz);
//        halfez = (ax * halfvy - ay * halfvx);

//        if(m->ki > 0.0f) {
//            m->ix += m->ki * halfex * (1.0f / m->sample_freq);
//            m->iy += m->ki * halfey * (1.0f / m->sample_freq);
//            m->iz += m->ki * halfez * (1.0f / m->sample_freq);
//            gx += m->ix; gy += m->iy; gz += m->iz;
//        }

//        gx += m->kp * halfex;
//        gy += m->kp * halfey;
//        gz += m->kp * halfez;
//    }

//    gx *= (0.5f * (1.0f / m->sample_freq));
//    gy *= (0.5f * (1.0f / m->sample_freq));
//    gz *= (0.5f * (1.0f / m->sample_freq));
//    
//    qa = m->q0; qb = m->q1; qc = m->q2;
//    m->q0 += (-qb * gx - qc * gy - m->q3 * gz);
//    m->q1 += (qa * gx + qc * gz - m->q3 * gy);
//    m->q2 += (qa * gy - qb * gz + m->q3 * gx);
//    m->q3 += (qa * gz + qb * gy - qc * gx);

//    recipNorm = inv_sqrt(m->q0 * m->q0 + m->q1 * m->q1 + m->q2 * m->q2 + m->q3 * m->q3);
//    m->q0 *= recipNorm; m->q1 *= recipNorm; m->q2 *= recipNorm; m->q3 *= recipNorm;
//    
//    // === 2. 四元数 -> 欧拉角 (移植自 Filter_GetEuler) ===
//    
//    // Roll (X轴)
//    sinr = 2.0f * (m->q0 * m->q1 + m->q2 * m->q3);
//    cosr = 1.0f - 2.0f * (m->q1 * m->q1 + m->q2 * m->q2);
//    m->roll = atan2(sinr, cosr) * 57.29578f;

//    // Pitch (Y轴)
//    sinp = 2.0f * (m->q0 * m->q2 - m->q3 * m->q1);
//    if (fabs(sinp) >= 1.0f)
//        m->pitch = (sinp > 0) ? 90.0f : -90.0f;
//    else
//        m->pitch = asin(sinp) * 57.29578f;

//    // Yaw (Z轴)
//    siny = 2.0f * (m->q0 * m->q3 + m->q1 * m->q2);
//    cosy = 1.0f - 2.0f * (m->q2 * m->q2 + m->q3 * m->q3);
//    m->yaw = atan2(siny, cosy) * 57.29578f;
//}
