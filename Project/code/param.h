#ifndef _PARAM_H_
#define _PARAM_H_

#include "zf_common_headfile.h"

// 参数结构体
typedef struct
{
    uint16 flag;         // 校验位 0x55AA (用于判断Flash是否为空)

    // 平衡环
    float balance_kp;
    float balance_kd;

    // 速度环
    float velocity_kp;
    float velocity_ki;

    // 转向环
    float turn_kp;       
    float turn_kd;       

    // 其他配置
    int16 track_speed;   // 循迹基准速度
    float mech_zero_pitch; // 机械中值
} SysParam_t;

// 全局参数变量
extern SysParam_t g_sys_param;

// API 函数
void Param_Init(void);       // 初始化并读取
void Param_Save(void);       // 保存当前参数到 Flash
void Param_SetDefaults(void);// 恢复默认值

#endif
