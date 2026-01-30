#include "attitude.h"
#include <math.h>

mahony_t m_imu;

// 快速平方根倒数(用于归一化)
// 加速度向量归一化：让 (ax,ay,az) 变成单位向量  //
// 四元数归一化：让 q 保持单位长度，避免数值漂移 //
static float inv_sqrt(float x) {
    float half = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - half * y * y);
    return y;
}

void mahony_init(mahony_t *m, float freq, float kp, float ki)
{
    m->q0 = 1.0f; m->q1 = 0.0f; m->q2 = 0.0f; m->q3 = 0.0f;		 // 初始姿态：无旋转
    m->kp = kp; m->ki = ki;
    m->ix = 0.0f; m->iy = 0.0f; m->iz = 0.0f;					 // 积分项清零
    m->sample_freq = freq;
    m->pitch = 0.0f; m->roll = 0.0f; m->yaw = 0.0f;
    
    // 初始化累积角度
    m->total_yaw = 0.0f; 
}

// gx, gy, gz 单位是 弧度/秒 (rad/s)
void mahony_update(mahony_t *m, float ax, float ay, float az, float gx, float gy, float gz)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // === 新增：计算累积航向角 (Total Yaw) ===
    // 积分公式：角度 += 角速度 * dt
    // 57.29578 是 180/PI (弧度转角度)
    float gz_deg = gz * 57.29578f;
    
    // 简单的死区过滤，防止静止时漂移
    if(gz_deg > 0.5f || gz_deg < -0.5f) {
        m->total_yaw += gz_deg * (1.0f / m->sample_freq);
    }
    // =====================================

    // 如果没有加速度数据，只更新陀螺仪积分
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = m->q1 * m->q3 - m->q0 * m->q2;
        halfvy = m->q0 * m->q1 + m->q2 * m->q3;
        halfvz = m->q0 * m->q0 - 0.5f + m->q3 * m->q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if(m->ki > 0.0f) {
            m->ix += m->ki * halfex * (1.0f / m->sample_freq);
            m->iy += m->ki * halfey * (1.0f / m->sample_freq);
            m->iz += m->ki * halfez * (1.0f / m->sample_freq);
            gx += m->ix; gy += m->iy; gz += m->iz;
        } else {
            m->ix = 0.0f; m->iy = 0.0f; m->iz = 0.0f;
        }

        gx += m->kp * halfex;
        gy += m->kp * halfey;
        gz += m->kp * halfez;
    }

    gx *= (0.5f * (1.0f / m->sample_freq));
    gy *= (0.5f * (1.0f / m->sample_freq));
    gz *= (0.5f * (1.0f / m->sample_freq));
    
    qa = m->q0; qb = m->q1; qc = m->q2;
    m->q0 += (-qb * gx - qc * gy - m->q3 * gz);
    m->q1 += (qa * gx + qc * gz - m->q3 * gy);
    m->q2 += (qa * gy - qb * gz + m->q3 * gx);
    m->q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = inv_sqrt(m->q0 * m->q0 + m->q1 * m->q1 + m->q2 * m->q2 + m->q3 * m->q3);
    m->q0 *= recipNorm; m->q1 *= recipNorm; m->q2 *= recipNorm; m->q3 *= recipNorm;
    
    // 这里的 Pitch 必须非常准，如果角度反了请加负号
    m->pitch = atan2(2.0f * (m->q0 * m->q1 + m->q2 * m->q3), 1.0f - 2.0f * (m->q1 * m->q1 + m->q2 * m->q2)) * 57.29578f;
    // roll, yaw 计算保留，虽然不用
}
