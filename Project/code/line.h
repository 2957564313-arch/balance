#ifndef _LINE_H_
#define _LINE_H_

#include "zf_common_headfile.h"

// API 函数声明
void Line_Init(void);

// 获取循迹误差 (用于转向 PID)
// 返回值范围：-4.0 ~ 4.0 (0 表示正中或全白)
float Track_Get_Weighted_Error(void);

// 判断当前是否存在黑线 (用于状态机切换)
// 返回 1: 至少有一个传感器压线
// 返回 0: 全白 (直道)
uint8 Track_Is_Line_Exist(void);

#endif
