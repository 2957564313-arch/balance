#include "zf_common_clock.h"
#include "zf_driver_delay.h"


static vuint16 zf_delay_1ms = 0;
static vuint16 zf_delay_1us = 0;

// 内部使用的高精度延时函数
static void delay_cycles(uint16 cycles)
{
    // 循环体执行一次需要3个时钟周期（DEC、JNZ、NOP）
    while(cycles-- > 0)
    {
        ;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      软件延时函数初始化
//  @param      NULL
//  @return     void
//  Sample usage:               无需用户调用，用户请使用h文件中的宏定义
//-------------------------------------------------------------------------------------------------------------------
void system_delay_init(void)
{
	zf_delay_1ms = system_clock / 6000;
	zf_delay_1us = system_clock / 7000000;
	if(system_clock <= 12000000) zf_delay_1us++;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      延时函数
//  @param      ms            	需要延时的时间（ms）
//  @return     void
//  Sample usage:               无需用户调用，用户请使用h文件中的宏定义
//-------------------------------------------------------------------------------------------------------------------
void system_delay_ms(uint16 ms)
{
	do 
	{
		delay_cycles(zf_delay_1ms);
	}while(--ms);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      延时函数
//  @param      us            	需要延时的时间（us）
//  @return     void
//  Sample usage:               这是一个不太准的函数
//-------------------------------------------------------------------------------------------------------------------
void system_delay_us(uint16 us)
{
	do 
	{
		delay_cycles(zf_delay_1us);
	}while(--us);
}
