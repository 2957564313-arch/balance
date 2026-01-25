#ifndef __BOARD_H
#define __BOARD_H
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"


#define SYSTEM_CLOCK_22_1184M 	22118400
#define SYSTEM_CLOCK_24M      	24000000
#define SYSTEM_CLOCK_27M      	27000000
#define SYSTEM_CLOCK_30M      	30000000
#define SYSTEM_CLOCK_33_1776M 	33177600
#define SYSTEM_CLOCK_35M      	35000000


#define EXTERNAL_CRYSTA_ENABLE 	0			// 使用外部晶振，0为不使用，1为使用（建议使用内部晶振）

#define FOSC					0			// FOSC的值设置为0，则内核频率通过寄存器强制设置。
											// 不管STC-ISP软件下载时候选择多少，他都是设置的频率。
											
//#define FOSC      	SYSTEM_CLOCK_30M	// FOSC的值设置为30Mhz,
											// 使用STC-ISP软件下载的时候，
											// 此频率需要跟STC-ISP软件中的 <输入用户程序运行时的IRC频率>选项的频率一致。

extern int32 system_clock;

void clock_init (uint32 clock);                                               // 核心时钟初始化


#endif

