#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include "zf_common_headfile.h"

// =================================================================
// 蓝牙硬件配置
// =================================================================

#define BT_UART_INDEX   UART_4
#define BT_TX_PIN       UART4_TX_P03
#define BT_RX_PIN       UART4_RX_P02

// 波特率 (HC-05/HC-08 默认通常为 9600)
#define BT_BAUD_RATE    9600 
#define BT_RX_MAX_LEN   64   // 缓冲区大小，64字节足够存下摇杆包了

// 全局控制变量 (供 mode.c 读取)
extern int16 remote_speed; // 遥控速度
extern int16 remote_turn;  // 遥控转向

// API 函数声明
void BT_Init(void);
void BT_Check_Rx(void); // 放在主循环里调用
void BT_Printf(const char *fmt, ...); // 调试打印用

#endif
