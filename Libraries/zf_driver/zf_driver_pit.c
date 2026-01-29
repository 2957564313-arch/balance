#include "zf_common_clock.h"
#include "zf_driver_pit.h"

#pragma warning disable = 183
#pragma warning disable = 177

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器周期中断
//  @param      pit_n      定时器通道号
//  @param      time_ms    时间(ms)
//  @return     void
//  Sample usage:          pit_timer_ms(TIM_0, 10)
//						   使用定时器0做周期中断，时间10ms一次。
//-------------------------------------------------------------------------------------------------------------------
void pit_init(pit_index_enum pit_n, uint32 period)
{
	uint8 freq_div = 0;                
    uint16 period_temp = 0;               
    uint16 temp = 0;


	if(period > (255*65535))
	{
		// 设置定时器为12T模式防止溢出
		period = period / 12;
		
		if(TIM0_PIT == pit_n)
		{
			AUXR &= ~0x80;		
		}
		else if(TIM1_PIT == pit_n)
		{
			AUXR &= ~0x40;		
		}
		else if(TIM2_PIT == pit_n)
		{
			AUXR &= ~0x04;		
		}
		else if(TIM3_PIT == pit_n)
		{
			T4T3M &= ~0x02;	
		}
		else if(TIM4_PIT == pit_n)
		{
			T4T3M &= ~0x20;		
		}
	

	}
	else
	{
		// 设置为1T模式
		if(TIM0_PIT == pit_n)
		{
			AUXR |= 0x80;		
		}
		else if(TIM1_PIT == pit_n)
		{
			AUXR |= 0x40;		
		}
		else if(TIM2_PIT == pit_n)
		{
			AUXR |= 0x04;		
		}
		else if(TIM3_PIT == pit_n)
		{
			T4T3M |= 0x02;	
		}
		else if(TIM4_PIT == pit_n)
		{
			T4T3M |= 0x20;		
		}

	}
		
	
	freq_div = ((period) >> 15);                   // 计算预分频
	period_temp = ((period) / (freq_div+1));       // 计算自动重装载值

	temp = (uint16)65536 - period_temp;

    if(TIM0_PIT == pit_n)
    {
		TM0PS = freq_div;	// 设置分频值
        TMOD |= 0x00; 		// 模式 0
        TL0 = temp;
        TH0 = temp >> 8;
        TR0 = 1; 			// 启动定时器
        ET0 = 1; 			// 使能定时器中断
    }
    else if(TIM1_PIT == pit_n)
    {
		TM1PS = freq_div;	// 设置分频值
        TMOD |= 0x00; // 模式 0
        TL1 = temp;
        TH1 = temp >> 8;
        TR1 = 1; // 启动定时器
        ET1 = 1; // 使能定时器中断
    }
    else if(TIM2_PIT == pit_n)
    {
		TM2PS = freq_div;	// 设置分频值
        T2L = temp;
        T2H = temp >> 8;
        AUXR |= 0x10; // 启动定时器
        IE2 |= 0x04; // 使能定时器中断
    }
    else if(TIM3_PIT == pit_n)
    {
		TM3PS = freq_div;	// 设置分频值
        T3L = temp;
        T3H = temp >> 8;
        T4T3M |= 0x08; // 启动定时器
        IE2 |= 0x20; // 使能定时器中断
    }
    else if(TIM4_PIT == pit_n)
    {
		TM4PS = freq_div;	// 设置分频值
        T4L = temp;
        T4H = temp >> 8;
        T4T3M |= 0x80; // 启动定时器
        IE2 |= 0x40; // 使能定时器中断
    }
	
}

