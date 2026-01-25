#include "attitude.h"
#include <math.h>

static float inv_sqrt(float x)
{
    float half;
    float y;
    long i;

    half = 0.5f * x;
    y = x;
    i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - half * y * y);
    return y;
}

void mahony_init(mahony_t *m, float freq, float kp, float ki)
{
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;

    m->kp = kp;
    m->ki = ki;

    m->ix = 0.0f;
    m->iy = 0.0f;
    m->iz = 0.0f;

    m->sample_freq = freq;
}

void mahony_update(mahony_t *m,
                   float gx, float gy, float gz,
                   float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float dt;
    float qa, qb, qc;

    if(ax == 0 && ay == 0 && az == 0) return;

    norm = inv_sqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    vx = 2.0f * (m->q1*m->q3 - m->q0*m->q2);
    vy = 2.0f * (m->q0*m->q1 + m->q2*m->q3);
    vz = m->q0*m->q0 - m->q1*m->q1 - m->q2*m->q2 + m->q3*m->q3;

    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    dt = 1.0f / m->sample_freq;

    m->ix += m->ki * ex * dt;
    m->iy += m->ki * ey * dt;
    m->iz += m->ki * ez * dt;

    gx += m->kp * ex + m->ix;
    gy += m->kp * ey + m->iy;
    gz += m->kp * ez + m->iz;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = m->q0;
    qb = m->q1;
    qc = m->q2;

    m->q0 += (-qb*gx - qc*gy - m->q3*gz);
    m->q1 += ( qa*gx + qc*gz - m->q3*gy);
    m->q2 += ( qa*gy - qb*gz + m->q3*gx);
    m->q3 += ( qa*gz + qb*gy - qc*gx);

    norm = inv_sqrt(m->q0*m->q0 + m->q1*m->q1 +
                    m->q2*m->q2 + m->q3*m->q3);

    m->q0 *= norm;
    m->q1 *= norm;
    m->q2 *= norm;
    m->q3 *= norm;

    m->pitch = (float)asin(2.0 * (m->q0 * m->q2 - m->q1 * m->q3)) * 57.3f;
    m->roll  = (float)atan2(2.0 * (m->q0 * m->q1 + m->q2 * m->q3),
                      1.0 - 2.0 * (m->q1 * m->q1 + m->q2 * m->q2)) * 57.3f;
}
