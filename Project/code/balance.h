#ifndef _BALANCE_H_
#define _BALANCE_H_

#include "zf_common_headfile.h"

// 平衡控制任务，在 5ms 定时中断中调用
void Balance_Task(void);

// 初始化平衡控制相关
void Balance_Init(void);

#endif
