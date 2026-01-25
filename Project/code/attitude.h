#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "zf_common_headfile.h"

typedef struct
{
    float q0, q1, q2, q3;
    float pitch;
    float roll;

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
