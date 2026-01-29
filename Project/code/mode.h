#ifndef _MODE_H_
#define _MODE_H_

#include "zf_common_headfile.h"

// 定义 5 种模式
typedef enum {
    MODE_1_BALANCE = 1, // 原地平衡
    MODE_2_TRACK_1,     // 跑一圈
    MODE_3_TRACK_4,     // 跑四圈
    MODE_4_REPLAY,      // 路径记忆
    MODE_5_REMOTE       // 遥控模式
} RunMode_e;

// 路径记忆状态
typedef enum { REC_OFF = 0, REC_ON, REC_PLAY } PathState_e;

// 全局变量
extern RunMode_e current_mode;
extern int16 target_speed_val;
extern int16 target_turn_val;
extern PathState_e path_state;

void Mode_Init(void);
void Mode_Handler(void); // 5ms 中断调用
void Mode_Path_Key_Handler(uint8 key); // 按键处理

#endif
