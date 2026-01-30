#ifndef __OLED_H
#define __OLED_H

#include "zf_common_headfile.h"

void OLED_Init(void);
void OLED_Clear(void);

// 基础显示
void OLED_ShowChar(uint8 Line, uint8 Column, char Char);
void OLED_ShowString(uint8 Line, uint8 Column, char *String);

// 原有的基于 sprintf 的浮点显示 (比较慢，可以留着备用)
void OLED_Show_Float(uint8 Line, uint8 Column, float dat, uint8 num, uint8 pointnum);

// === 新增：极速无延迟显示函数 ===
// 1. 快速显示整数 (不使用 sprintf)
void OLED_Show_Int_Fast(uint8 row, uint8 col, int32 num);

// 2. 快速显示浮点 (整数运算，防卡顿，固定格式 -999.99)
void OLED_Show_Float_Safe(uint8 row, uint8 col, float val);

#endif
