#include "filter.h"

// 限幅函数：将 x 限制在 [minv, maxv] 之间
float limit_f(float x, float minv, float maxv)
{
    if(x > maxv) return maxv;
    if(x < minv) return minv;
    return x;
}

// 死区函数：如果 x 在 [-dz, dz] 之间，则返回 0
float deadzone_f(float x, float dz)
{
    if(x > -dz && x < dz) return 0.0f;
    return x;
}

// LPF 初始化
void lpf1_init(lpf1_t *f, float alpha, float init_val)
{
    if(0 == f) return; // 空指针保护
    
    f->alpha  = alpha;
    f->y      = init_val;
    f->inited = 1;
}

// LPF 更新计算
// 公式：Output = alpha * Last_Output + (1-alpha) * Input
float lpf1_update(lpf1_t *f, float x)
{
    if(0 == f) return x;
    
    // 第一次运行时，直接将输入作为输出，避免从0开始爬升
    if(!f->inited)
    {
        f->y = x;
        f->inited = 1;
        return x;
    }
    
    // 迭代计算
    f->y = f->alpha * f->y + (1.0f - f->alpha) * x;
    
    return f->y;
}
