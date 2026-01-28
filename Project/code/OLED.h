#ifndef _zf_device_oled_h_
#define _zf_device_oled_h_

#include "zf_common_headfile.h"
#include "oledfont.h"

// ====== 接线定义 ======
#define OLED_SCL_PIN           IO_P25
#define OLED_SDA_PIN           IO_P23

// 软I2C延时 (30MHz主频下，10左右即可)
#define OLED_IIC_DELAY         10

// 屏幕大小 (SSD1306 128x64)
#define OLED_W                 128
#define OLED_H                 64

// ====== API 函数声明 ======

void OLED_Init(void);
void OLED_Clear(void);  // 清空缓冲区
void OLED_Update(void); // 将缓冲区数据刷到屏幕

// 基础显示函数
// Line: 1~4, Column: 1~16
void OLED_ShowChar(uint8 Line, uint8 Column, char Char);    
void OLED_ShowString(uint8 Line, uint8 Column, char *String);

// 数字显示函数
// Length: 数字长度
void OLED_ShowNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length);
void OLED_ShowSignedNum(uint8 Line, uint8 Column, int32 Number, uint8 Length);
void OLED_ShowHexNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length);
void OLED_ShowBinNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length);
void OLED_ShowFloat(uint8 Line, uint8 Column, float Number, uint8 IntLength, uint8 FracLength);

#endif
