#ifndef __KEY_H__
#define __KEY_H__

#include "zf_common_headfile.h"

// =================================================================
// 硬件配置 (根据原理图修改)
// =================================================================
#define KEY1_PIN    IO_P70  // 常用作 Mode 切换 / 录制
#define KEY2_PIN    IO_P71  // 常用作 播放 / 确认
#define KEY3_PIN    IO_P72
#define KEY4_PIN    IO_P73

// =================================================================
// 事件标志定义
// =================================================================
#define KEY_DOWN    0x01    // 按下瞬间
#define KEY_UP      0x02    // 抬起瞬间
#define KEY_HOLD    0x04    // 只要按着就置位
#define KEY_SINGLE  0x08    // 单击 (松手后触发)
#define KEY_DOUBLE  0x10    // 双击
#define KEY_LONG    0x20    // 长按 (超过1秒)
#define KEY_REPEAT  0x40    // 长按连发

// 按键索引
#define KEY_NAME_UP         0
#define KEY_NAME_DOWN       1
#define KEY_NAME_CONFIRM    2
#define KEY_NAME_BACK       3
#define KEY_COUNT           4

// =================================================================
// 函数声明
// =================================================================
void Key_Init(void);
void Key_Tick(void); // 放在 5ms 定时器中断里调用

// 核心检测函数
// n: 按键索引 (0-3)
// Flag: 想要检测的事件 (如 KEY_SINGLE)
// 返回: 1=发生了, 0=没发生
uint8 Key_Check(uint8 n, uint8 Flag);

// 简易检测函数 (为了兼容 main.c 的简单调用)
// 只要有任意按键被单击，就返回按键号(1-4)，否则返回0
uint8 Key_Check_Simple(void);

#endif
