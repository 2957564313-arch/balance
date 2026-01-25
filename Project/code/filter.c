#include "filter.h"

float limit_f(float x, float minv, float maxv)
{
    if(x > maxv) return maxv;
    if(x < minv) return minv;
    return x;
}

float deadzone_f(float x, float dz)
{
    if(x > -dz && x < dz) return 0.0f;
    return x;
}

void lpf1_init(lpf1_t *f, float alpha, float init_val)
{
    if(0 == f) return;
    f->alpha  = alpha;
    f->y      = init_val;
    f->inited = 1;
}

float lpf1_update(lpf1_t *f, float x)
{
    if(0 == f) return x;
    if(!f->inited)
    {
        f->y = x;
        f->inited = 1;
        return x;
    }
    // y = a*y + (1-a)*x
    f->y = f->alpha * f->y + (1.0f - f->alpha) * x;
    return f->y;
}
