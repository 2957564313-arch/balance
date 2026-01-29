#ifndef _BUZZER_H_
#define _BUZZER_H_

#include "zf_common_headfile.h"

// 定义蜂鸣器引脚 P6.7
#define BEEP_PIN IO_P67 

void Buzzer_Init(void);		 // 初始化GPIO
void Buzzer_Beep(uint16 ms); // 开启蜂鸣器，并设定响声时长(毫秒)
void Buzzer_Task(void);      // 放在 5ms 定时器中调用，用来计时，并在到点后关蜂鸣器

#endif
