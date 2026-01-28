#ifndef __LED_H
#define __LED_H

#include "zf_common_headfile.h"

// 核心板上的 LED 通常接在 P5.2 (低电平点亮)
#define LED_PIN IO_P52

void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);
// 翻转函数 (调试闪烁用)
#define LED_TOGGLE()  gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN))

#endif
