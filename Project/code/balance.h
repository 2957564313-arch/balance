#ifndef _BALANCE_H_
#define _BALANCE_H_

#include "zf_common_headfile.h"

// === 这里定义全局角度变量，供 main.c 和 mode.c 使用 ===
extern float pitch;
extern float roll;
extern float yaw;

// 惯性导航坐标
extern float g_car_x;
extern float g_car_y;

void Balance_Task(void);
void Balance_Init(void);

#endif
