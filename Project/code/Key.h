#ifndef __KEY_H
#define __KEY_H

#include "zf_common_headfile.h"

//宏定义调换按键数量
#define KEY_COUNT				    4
//用宏定义替换按键索引号

//用宏定义替换按键标志位的位掩码，使程序的意义更清晰
#define KEY_HOLD				0x01
#define KEY_DOWN				0x02
#define KEY_UP					0x04
#define KEY_SINGLE				0x08
#define KEY_DOUBLE				0x10
#define KEY_LONG				0x20
#define KEY_REPEAT				0x40

void Key_Init(void);
uint8 Key_GetState(uint8 n);
uint8 Key_Check(uint8 n, uint8 Flag);
void Key_Tick(void);

#endif
