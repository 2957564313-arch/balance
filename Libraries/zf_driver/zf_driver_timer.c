#include "zf_driver_gpio.h"
#include "zf_driver_timer.h"


// 该数组禁止修改，内部使用,用户无需关心
static timer_function_enum timer_function_state[6] =
{
    TIMER_FUNCTION_INIT,
    TIMER_FUNCTION_INIT,
    TIMER_FUNCTION_INIT,			
    TIMER_FUNCTION_INIT,
    TIMER_FUNCTION_INIT,
	TIMER_FUNCTION_INIT,
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      TIMER 外设确认功能状态 库内部调用
// 参数说明     index           TIMER 外设模块号
// 参数说明     mode            需要确的功能模块
// 返回参数     uint8           1-可以使用 0-不可以使用
// 使用示例     zf_assert(timer_funciton_check(TIM_1, TIMER_FUNCTION_PWM);
//-------------------------------------------------------------------------------------------------------------------
uint8 timer_funciton_check (timer_index_enum index, timer_function_enum mode)
{
    uint8 return_state = 1;
    if(TIMER_FUNCTION_INIT == timer_function_state[index])
    {
        timer_function_state[index] = mode;
    }
    else if(timer_function_state[index] == mode)
    {
        return_state = 1;
    }
    else
    {
        return_state = 0;
    }
    return return_state;
}
