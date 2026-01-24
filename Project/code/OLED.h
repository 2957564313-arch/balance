#ifndef _zf_device_oled_h_
#define _zf_device_oled_h_

#include "zf_common_headfile.h"
#include "oledfont.h"

// ====== 接线 ======
#define OLED_SCL_PIN           IO_P25
#define OLED_SDA_PIN           IO_P23

// 软I2C延时
#define OLED_IIC_DELAY         1

// 屏幕大小（SSD1306 128x64）
#define OLED_W                 128
#define OLED_H                 64

// ==============调用函数只要看这里，上面的不用管==============

void OLED_Init(void);
void OLED_Clear(void);	//清空缓冲区
void OLED_Update(void);	//注意：任何改动后都要调用这个函数后才能显示

//Line是行（1-4），Column是列（1-16）
void OLED_ShowChar(uint8 Line, uint8 Column, char Char);	
void OLED_ShowString(uint8 Line, uint8 Column, char *String);
void OLED_ShowNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length);
void OLED_ShowSignedNum(uint8 Line, uint8 Column, int32 Number, uint8 Length);
void OLED_ShowHexNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length);
void OLED_ShowBinNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length);
void OLED_ShowFloat(uint8 Line, uint8 Column, float Number, uint8 IntLength, uint8 FracLength);
//IntLength是整数长度，FracLength是小数长度，会四舍五入

#endif
