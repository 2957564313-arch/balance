#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include "zf_common_headfile.h"

//UART4 映射到 P0.2/P0.3，波特率 115200
#define BT_BAUD_RATE    115200

// 导出变量（给 main / mode 使用）
extern int16 remote_speed; 
extern int16 remote_turn;  
extern volatile uint8  last_byte;
extern volatile uint32 rx_count;

// API
void BT_Init(void);
void BT_Parse_Task(void);

#endif
