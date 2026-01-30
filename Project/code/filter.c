#include "filter.h"
#include <math.h>
#include <stdlib.h>

/* ================= 参数调优 ================= */
#define MAHONY_KP     2.0f    
#define MAHONY_KI     0.0f      

// 零速锁定阈值 (防漂移核心)
// 如果陀螺仪三轴绝对值之和小于此值，认为完全静止，强制锁死角度
#define STATIC_THRESHOLD  5.0f  

// 限幅阈值 (解决0.06突变)
// 加速度计单位是g，0.2g的突变通常是非法的
#define ACC_LIMIT_VAL     0.2f   

// 低通滤波系数 (0~1)
// 越小越平滑，越大响应越快。0.3 是一个平衡点
#define ACC_LPF_ALPHA     0.3f   
#define GYRO_LPF_ALPHA    0.6f   

static float s_sampleFreq = 200.0f; // 5ms
static float twoKp = 4.0f; // 2 * KP
static float twoKi = 0.0f;
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

//一阶低通滤波
typedef struct {
    float alpha;
    float output;
    unsigned char is_init;
} LowPassFilter_t;

static LowPassFilter_t lpf_ax, lpf_ay, lpf_az;
static LowPassFilter_t lpf_gx, lpf_gy, lpf_gz;

static float LPF_Update(LowPassFilter_t *filter, float input) {
    if (filter->is_init == 0) {
        filter->output = input;
        filter->is_init = 1;
        return input;
    }
    // Y(n) = α * X(n) + (1 - α) * Y(n-1)
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;
    return filter->output;
}

static void LPF_Init_Struct(LowPassFilter_t *filter, float alpha) {
    filter->alpha = alpha;
    filter->output = 0.0f;
    filter->is_init = 0;
}


// 限幅滤波
static float last_ax_val = 0.0f;
static float last_ay_val = 0.0f;
static float last_az_val = 0.0f;
static unsigned char amp_init = 0;

static float AmplitudeLimit(float new_val, float *last_val, float limit) {
    float delta;
    if (amp_init == 0) return new_val;
    
    delta = fabs(new_val - *last_val);
    // 如果偏差小于限幅值，接受；否则保持上一次的值
    if (delta <= limit) {
        *last_val = new_val;
        return new_val;
    } else {
        return *last_val; 
    }
}

/* ================= Mahony 辅助函数 ================= */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* ================= 外部接口 ================= */

void Filter_Init(float sampleFreqHz) {
    if (sampleFreqHz > 0.0f) s_sampleFreq = sampleFreqHz;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    
    // 初始化低通滤波器
    LPF_Init_Struct(&lpf_ax, ACC_LPF_ALPHA);
    LPF_Init_Struct(&lpf_ay, ACC_LPF_ALPHA);
    LPF_Init_Struct(&lpf_az, ACC_LPF_ALPHA);
    LPF_Init_Struct(&lpf_gx, GYRO_LPF_ALPHA);
    LPF_Init_Struct(&lpf_gy, GYRO_LPF_ALPHA);
    LPF_Init_Struct(&lpf_gz, GYRO_LPF_ALPHA);
    
    amp_init = 0;
}

void Filter_Update(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    // --- 1. 限幅滤波 (解决突变) ---
    // 第一次先初始化历史值
    if (amp_init == 0) {
        last_ax_val = ax; last_ay_val = ay; last_az_val = az;
        amp_init = 1;
    }
    ax = AmplitudeLimit(ax, &last_ax_val, ACC_LIMIT_VAL);
    ay = AmplitudeLimit(ay, &last_ay_val, ACC_LIMIT_VAL);
    az = AmplitudeLimit(az, &last_az_val, ACC_LIMIT_VAL);

    // --- 2. 低通滤波 (解决抖动) ---
    ax = LPF_Update(&lpf_ax, ax);
    ay = LPF_Update(&lpf_ay, ay);
    az = LPF_Update(&lpf_az, az);
    
    // 陀螺仪也稍微滤一下，去除高频噪声
    gx = LPF_Update(&lpf_gx, gx);
    gy = LPF_Update(&lpf_gy, gy);
    gz = LPF_Update(&lpf_gz, gz);

    // --- 3. 零速锁定---
    // 计算陀螺仪的总活跃度
    if (fabs(gx) + fabs(gy) + fabs(gz) < STATIC_THRESHOLD) {

        gx = 0.0f;
        gy = 0.0f;
        gz = 0.0f;
    }

    // --- 4. Mahony 解算 ---
    // 转弧度
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // 加速度计修正
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 积分
    gx *= (0.5f / s_sampleFreq);
    gy *= (0.5f / s_sampleFreq);
    gz *= (0.5f / s_sampleFreq);

    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void Filter_GetEuler(float *pitch_deg, float *roll_deg, float *yaw_deg) {
    float sinp;
    
    *roll_deg  = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    
    sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f) *pitch_deg = (sinp > 0.0f) ? 90.0f : -90.0f;
    else *pitch_deg = asin(sinp) * 57.29578f;

    *yaw_deg   = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
}
