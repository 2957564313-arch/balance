#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_interrupt.h"

#include "zf_driver_delay.h"

//内核频率
int32 system_clock = SYSTEM_CLOCK_30M;



//-------------------------------------------------------------------------------------------------------------------
//  @brief      STC32G设置系统频率
//  @param      NULL          	空值
//  @return     void        	系统频率
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
uint32 set_sys_clk(uint32 sys_clk)
{
	
	P_SW2 |= 0x80;
		
	switch(sys_clk) 
	{
		case SYSTEM_CLOCK_22_1184M:
			// 选择 22.1184MHz
			CLKDIV = 0x04;
			IRTRIM = T22M_ADDR;
			VRTRIM = VRT27M_ADDR;
			IRCBAND = 0x02;
			CLKDIV = 0x00;
		break;
			
		case SYSTEM_CLOCK_24M:
			// 选择 24MHz
			CLKDIV = 0x04;
			IRTRIM = T24M_ADDR;
			VRTRIM = VRT27M_ADDR;
			IRCBAND = 0x02;
			CLKDIV = 0x00;
		break;
			
		case SYSTEM_CLOCK_27M:
			// 选择 27MHz
			CLKDIV = 0x04;
			IRTRIM = T27M_ADDR;
			VRTRIM = VRT27M_ADDR;
			IRCBAND = 0x02;
			CLKDIV = 0x00;
		break;
			
		case SYSTEM_CLOCK_30M:
			// 选择 30MHz
			CLKDIV = 0x04;
			IRTRIM = T30M_ADDR;
			VRTRIM = VRT27M_ADDR;
			IRCBAND = 0x02;
			CLKDIV = 0x00;
		break;
			
		case SYSTEM_CLOCK_33_1776M:
			// 选择 33.1776MHz
			CLKDIV = 0x04;
			IRTRIM = T33M_ADDR;
			VRTRIM = VRT27M_ADDR;
			IRCBAND = 0x02;
			CLKDIV = 0x00;
		break;
			
		case SYSTEM_CLOCK_35M:
			// 选择 35MHz
			CLKDIV = 0x04;
			IRTRIM = T35M_ADDR;
			VRTRIM = VRT44M_ADDR;
			IRCBAND = 0x03;
			CLKDIV = 0x00;
		break;
			
		default:
			sys_clk = SYSTEM_CLOCK_30M;
			// 选择 30MHz
			CLKDIV = 0x04;
			IRTRIM = T30M_ADDR;
			VRTRIM = VRT27M_ADDR;
			IRCBAND = 0x02;
			CLKDIV = 0x00;
		break;
	}

	return sys_clk;
}




void clock_init (uint32 clock)                                              // 核心时钟初始化
{
	EAXFR = 1;				// 使能访问XFR
	P_SW2 = 0x80;           // 开启特殊地址访问
    CKCON = 0x00;           // 设置外部数据总线为最快
    WTST = 0;               // 设置程序代码等待参数，赋值为0可将CPU执行程序的速度设置为最快

#if (1 == EXTERNAL_CRYSTA_ENABLE)
	XOSCCR = 0xc0; 			//启动外部晶振
	while (!(XOSCCR & 1)); 	//等待时钟稳定
	CLKDIV = 0x00; 			//时钟不分频
	CLKSEL = 0x01; 			//选择外部晶振
#else
	//自动设置系统频率
	#if (0 == FOSC)
		system_clock = set_sys_clk(clock);
	#else
		system_clock = FOSC;
	#endif
#endif

	system_delay_init();    // 延时函数初始化

 
	P0M0 = 0x00;
	P0M1 = 0x00;
	P1M0 = 0x00;
	P1M1 = 0x00;
	P2M0 = 0x00;
	P2M1 = 0x00;
	P3M0 = 0x00;
	P3M1 = 0x00;
	P4M0 = 0x00;
	P4M1 = 0x00;
	P5M0 = 0x00;
	P5M1 = 0x00;
	P6M0 = 0x00;
	P6M1 = 0x00;
	P7M0 = 0x00;
	P7M1 = 0x00;
	
	ADCCFG = 0;
	AUXR = 0;
	SCON = 0;
	S2CON = 0;
	S3CON = 0;
	S4CON = 0;
	P_SW1 = 0;
	IE2 = 0;
	TMOD = 0;
	
	interrupt_global_enable();
}


