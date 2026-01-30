#ifndef __OLED_H
#define __OLED_H

#include "zf_common_headfile.h"

void OLED_Init(void);
void OLED_Clear(void);

void OLED_ShowChar(uint8 Line, uint8 Column, char Char);
void OLED_ShowString(uint8 Line, uint8 Column, char *String);
void OLED_Show_Float(uint8 Line, uint8 Column, float dat, uint8 num, uint8 pointnum);

#endif
