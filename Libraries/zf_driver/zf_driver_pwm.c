#include "zf_common_clock.h"
#include "zf_common_debug.h"

#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"




// 捕获比较模式寄存器
const uint32 PWM_CCMR_ADDR[] = {0x7efec8, 0x7efec9, 0x7efeca, 0x7efecb,
                                0x7efee8, 0x7efee9, 0x7efeea, 0x7efeeb
                               };

// 捕获比较使能寄存器
const uint32 PWM_CCER_ADDR[] = {0x7efecc, 0x7efecd,		// PWMA_CCERX
                                0x7efeec, 0x7efeed
                               };	// PWMB_CCERX

// 控制寄存器,高8位地址  低8位地址 + 1即可
const uint32 PWM_CCR_ADDR[] = {0x7efed5, 0x7efed7, 0x7efed9, 0x7efedb,
                               0x7efef5, 0x7efef7, 0x7efef9, 0x7efefb
                              };

// 控制寄存器,高8位地址  低8位地址 + 1即可
const uint32 PWM_ARR_ADDR[] = {0x7efed2, 0x7efef2};

// 预分频寄存器,高8位地址  低8位地址 + 1即可
const uint32 PWM_PSCR_ADDR[] = {0x7efed0, 0x7efef0};


#define PWMX_CCERX(pin)		(*(unsigned char volatile far *)(PWM_CCER_ADDR[((pin >> 12 & 0x02) >> 1) + ((pin >> 15 & 0x1) * 2)]))

#define PWMX_CCRXH(pin)		(*(unsigned char volatile far *)(PWM_CCR_ADDR[((pin >> 15 & 0x1) * 4) + (pin >> 12 & 0x7)]))
#define PWMX_CCRXL(pin)		(*(unsigned char volatile far *)(PWM_CCR_ADDR[((pin >> 15 & 0x1) * 4) + (pin >> 12 & 0x7)] + 1))

#define PWMX_ARRH(pin)		(*(unsigned char volatile far *)(PWM_ARR_ADDR[(pin >> 15 & 0x1) * 1]))
#define PWMX_ARRL(pin)		(*(unsigned char volatile far *)(PWM_ARR_ADDR[(pin >> 15 & 0x1) * 1] + 1))

#define PWMX_PSCRH(pin)		(*(unsigned char volatile far *)(PWM_PSCR_ADDR[(pin >> 15 & 0x1) * 1]))
#define PWMX_PSCRL(pin)		(*(unsigned char volatile far *)(PWM_PSCR_ADDR[(pin >> 15 & 0x1) * 1] + 1))

#define PWMX_CCMRX(pin)		(*(unsigned char volatile far *)(PWM_CCMR_ADDR[((pin >> 15 & 0x1) * 4) + (pin >> 12 & 0x7)]))


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM频率和占空比设置
// 参数说明     pin           	PWM通道号及引脚
// 参数说明     freq           	PWM频率
// 参数说明     duty            	PWM占空比
// 返回参数     void
// 使用示例     pwm_set_freq(PWMB_CH1_P01, 50, 1000);   //PWMB 使用引脚P01 频率50HZ 占空比为百分之 1000/PWM_DUTY_MAX*100
//                              						PWM_DUTY_MAX在zf_driver_pwm.h文件中 默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_set_freq(pwm_channel_enum pin, uint32 freq, uint32 duty)
{
    uint32 match_temp = 0;
    uint32 period_temp = 0;
    uint16 freq_div = 0;
	
	// 如果是这一行报错 那你得去看看最大占空比是限定的多少 占空比写入错误
    zf_assert(PWM_DUTY_MAX >= duty);
	
    //分频计算，周期计算，占空比计算
    freq_div = (system_clock / freq) >> 16;								// 多少分频
    period_temp = system_clock / freq;
    period_temp = period_temp / (freq_div + 1) - 1;					// 周期

    if(duty != PWM_DUTY_MAX)
    {
        match_temp = period_temp * ((float)duty / PWM_DUTY_MAX);	// 占空比
    }
    else
    {
        match_temp = period_temp + 1;								// duty为100%
	}

	PWMX_PSCRH(pin) = (uint8)(freq_div >> 8);	    // 设置预分频值
	PWMX_PSCRL(pin) = (uint8)freq_div;
	
	PWMX_ARRH(pin) = (uint8)(period_temp >> 8);		// 周期值 高8位
	PWMX_ARRL(pin) = (uint8)period_temp;			// 周期值 低8位

    PWMX_CCRXH(pin) = match_temp >> 8;				// 比较值 高8位
    PWMX_CCRXL(pin) = (uint8)match_temp;			// 比较值 低8位
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM占空比设定
// 参数说明     pwmch           PWM通道号及引脚
// 参数说明     duty            PWM占空比
// 返回参数     void
// 使用示例     pwm_set_duty(PWMB_CH1_P01, 5000);   //PWMB 使用引脚P01 占空比为百分之 5000/PWM_DUTY_MAX*100
//                              					PWM_DUTY_MAX在zf_driver_pwm.h文件中 默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_set_duty(pwm_channel_enum pin, uint32 duty)
{
    uint32 match_temp;
    uint32 arr = (PWMX_ARRH(pin) << 8) | PWMX_ARRL(pin);
	
	// 如果是这一行报错 那你得去看看最大占空比是限定的多少 占空比写入错误
    zf_assert(PWM_DUTY_MAX >= duty);
	
    if(duty != PWM_DUTY_MAX)
    {
        match_temp = arr * ((float)duty / PWM_DUTY_MAX);				//占空比
    }
    else
    {
        match_temp = arr + 1;
    }

    //设置捕获值|比较值
    PWMX_CCRXH(pin) = match_temp >> 8;				// 比较值 高8位
    PWMX_CCRXL(pin) = (uint8)match_temp;			// 比较值 低8位
	
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM初始化
// 参数说明     pin				PWM通道号及引脚
// 参数说明     freq            PWM频率
// 参数说明     duty            PWM占空比
// 返回参数     void
// 使用示例     pwm_init(PWMB_CH1_P01, 50, 5000);   //初始化PWMB 使用引脚P01  输出PWM频率50HZ   占空比为百分之 5000/PWM_DUTY_MAX*100
//                              					PWM_DUTY_MAX在zf_driver_pwm.h文件中 默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_init(pwm_channel_enum pin, uint32 freq, uint32 duty)
{

    uint16 match_temp = 0;                                              // 占空比值
    uint32 period_temp = 0;                                             // 周期值
    uint16 freq_div = 0;                                                // 分频值
    
	// 如果是这一行报错 那你得去看看最大占空比是限定的多少 占空比写入错误
    zf_assert(PWM_DUTY_MAX >= duty);
	
    gpio_init(pin  & 0xFF, GPO, 1, GPO_PUSH_PULL);							// GPIO需要设置为推挽输出
    
    
    //分频计算，周期计算，占空比计算
    freq_div = (system_clock / freq) >> 16;								// 分频值
    period_temp = system_clock / freq;
    period_temp = period_temp / (freq_div + 1) - 1;						// 周期值
	
    if(duty != PWM_DUTY_MAX)
    {		
        match_temp = period_temp * ((float)duty / PWM_DUTY_MAX);		// 占空比
    }
    else
    {
        match_temp = period_temp + 1;									// duty为100%
    }

	PWMX_PSCRH(pin) = (uint8)(freq_div >> 8);	    // 设置预分频值
	PWMX_PSCRL(pin) = (uint8)freq_div;
	
	PWMX_ARRH(pin) = (uint8)(period_temp >> 8);		// 周期值 高8位
	PWMX_ARRL(pin) = (uint8)period_temp;			// 周期值 低8位

    PWMX_CCRXH(pin) = (uint8)(match_temp >> 8);		// 比较值 高8位
    PWMX_CCRXL(pin) = (uint8)match_temp;			// 比较值 低8位
	
    if(((pin >> 15) & 0x01) == 0)					// PWMA
    {
        PWMA_ENO |= 1 << ((((pin >> 12) & 0x07) * 2) + ((pin >> 11) & 0x01));		// 使能通道
        PWMA_PS  |= ((pin >> 9) & 0x03) << (((pin >> 12) & 0x07) * 2);				// 选择引脚
        PWMX_CCERX(pin) |= 1 << ((pin >> 12 & 0x01) * 4 + (pin >> 11 & 0x01) * 2);	// 设置输出极性

        PWMA_BKR = 0x80; 							// 主输出使能 相当于总开关
        PWMA_CR1 = 0x01;							// PWM开始计数
    }
    else if(((pin >> 15) & 0x01) == 1)				// PWMB
    {
        PWMB_ENO |= 1 << ((((pin >> 12) & 0x07) * 2) + ((pin >> 11) & 0x01));		// 使能通道
        PWMB_PS  |= ((pin >> 9) & 0x03) << (((pin >> 12) & 0x07) * 2);				// 选择引脚
        PWMX_CCERX(pin) |= 1 << ((pin >> 12 & 0x01) * 4);							// 设置输出极性

        PWMB_BKR = 0x80; 							// 主输出使能 相当于总开关
        PWMB_CR1 = 0x01;							// PWM开始计数
    }

	PWMX_CCMRX(pin) |= 0x06 << 4;					// 设置为PWM模式1
	PWMX_CCMRX(pin) |= 1 << 3;						// 开启PWM寄存器的预装载功
}
