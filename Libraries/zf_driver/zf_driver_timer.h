#ifndef _zf_driver_timer_h
#define _zf_driver_timer_h


#include "zf_common_debug.h"
#include "zf_common_clock.h"

//此枚举定义不允许用户修改
typedef enum
{
	TIM_0 = 0,
    TIM_1,
    TIM_2,
    TIM_3,
    TIM_4,
    TIM_11,
}timer_index_enum;


typedef enum
{
    TIMER_FUNCTION_INIT = 0,                  // 功能未初始化    
    TIMER_FUNCTION_TIMER,                     // 用作 TIMER 计时 
    TIMER_FUNCTION_PIT,                       // 用作 PIT 周期定时器
    TIMER_FUNCTION_ENCODER,                   // 用作 ENCODER 的输入捕获  
    TIMER_FUNCTION_UART,                      // 用作 UART 的波特率发生器 
}timer_function_enum;


uint8 timer_funciton_check(timer_index_enum index, timer_function_enum mode);


#endif
