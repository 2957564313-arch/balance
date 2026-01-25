#ifndef __ZF_DRIVER_PIT_H
#define __ZF_DRIVER_PIT_H
#include "zf_common_typedef.h"

typedef enum                                                                    // 枚举 TIM通道
{
	TIM0_PIT,
    TIM1_PIT,
    TIM2_PIT,
    TIM3_PIT,
    TIM4_PIT,
}pit_index_enum;


#define TIM0_CLEAR_FLAG 	TCON &= ~(1<<5)    		// 定时器0溢出中断标志位。 中断服务程序中，硬件自动清零。
#define TIM1_CLEAR_FLAG 	TCON &= ~(1<<7)    		// 定时器1溢出中断标志位。 中断服务程序中，硬件自动清零。
#define TIM2_CLEAR_FLAG		AUXINTIF &= ~(1<<0) 	// 定时器2溢出中断标志位。 中断服务程序中，硬件自动清零。
#define TIM3_CLEAR_FLAG		AUXINTIF &= ~(1<<1) 	// 定时器3溢出中断标志位。 中断服务程序中，硬件自动清零。
#define TIM4_CLEAR_FLAG		AUXINTIF &= ~(1<<2)		// 定时器4溢出中断标志位。 中断服务程序中，硬件自动清零。

//====================================================宏定义函数区====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM PIT 中断初始化 ms 周期
// 参数说明     pit_n           使用的 PIT 编号
// 参数说明     ms              PIT 周期 ms 级别
// 返回参数     void
// 使用示例     pit_ms_init(TIM0_PIT, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
#define pit_ms_init(pit_n, ms)  (pit_init((pit_n), (ms) * (system_clock / 1000)))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM PIT 中断初始化 us 周期
// 参数说明     pit_n           使用的 PIT 编号
// 参数说明     us              PIT 周期 us 级别
// 返回参数     void
// 使用示例     pit_us_init(TIM0_PIT, 100);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
#define pit_us_init(pit_n, us)  (pit_init((pit_n), (us) * (system_clock / 1000000)))

//====================================================宏定义函数区====================================================

void pit_enable  (pit_index_enum pit_n);
void pit_disable (pit_index_enum pit_n);

void pit_init (pit_index_enum pit_n, uint32 period);

#endif
