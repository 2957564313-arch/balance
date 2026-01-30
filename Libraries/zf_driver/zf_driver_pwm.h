#ifndef __ZF_DRIVER_PWM_H
#define __ZF_DRIVER_PWM_H
#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"



// 16位
// BIT[15] 		: 0->PWMA 1->PWMB
// BIT[14:12]  	: 0->CH1 1->CH2 2->CH3 3->CH4 4->CH5 5->CH6
// BIT[11]		: 0->P 1->N
// BIT[10:9]	: 0->PS0 1->PS1 2->PS2 3->PS3
// BIT[7:0]		: IO_PIN
	
typedef enum
{
    // PWMA和PWMB是两组不同的PWM
    
    // 以下是PWMA通道，初始化只能有一个频率
    // 同一组PWM，同一时刻，只能有同一个PWM输出。
    // 例如:PWMA_CH1P_P10 和 PWMA_CH1N_P11不能一起输出。
	// 但是不同的通道可以同一时刻输出。
    // 例如:PWMA_CH1P_P10 和 PWMA_CH2N_P03 可以同时输出
    PWMA_CH1P_P10 = 0<<15|0<<12|0<<11|0<<9|IO_P10, PWMA_CH1N_P11 = 0<<15|0<<12|1<<11|0<<9|IO_P11,
    PWMA_CH1P_P20 = 0<<15|0<<12|0<<11|1<<9|IO_P20, PWMA_CH1N_P21 = 0<<15|0<<12|1<<11|1<<9|IO_P21,
    PWMA_CH1P_P60 = 0<<15|0<<12|0<<11|2<<9|IO_P60, PWMA_CH1N_P61 = 0<<15|0<<12|1<<11|2<<9|IO_P61,
																	                         	
    PWMA_CH2P_P54 = 0<<15|1<<12|0<<11|0<<9|IO_P54, PWMA_CH2N_P13 = 0<<15|1<<12|1<<11|0<<9|IO_P13,
    PWMA_CH2P_P22 = 0<<15|1<<12|0<<11|1<<9|IO_P22, PWMA_CH2N_P23 = 0<<15|1<<12|1<<11|1<<9|IO_P23,
    PWMA_CH2P_P62 = 0<<15|1<<12|0<<11|2<<9|IO_P62, PWMA_CH2N_P63 = 0<<15|1<<12|1<<11|2<<9|IO_P63,
																		                     	
    PWMA_CH3P_P14 = 0<<15|2<<12|0<<11|0<<9|IO_P14, PWMA_CH3N_P15 = 0<<15|2<<12|1<<11|0<<9|IO_P15,
    PWMA_CH3P_P24 = 0<<15|2<<12|0<<11|1<<9|IO_P24, PWMA_CH3N_P25 = 0<<15|2<<12|1<<11|1<<9|IO_P25,
    PWMA_CH3P_P64 = 0<<15|2<<12|0<<11|2<<9|IO_P64, PWMA_CH3N_P65 = 0<<15|2<<12|1<<11|2<<9|IO_P65,
																                             	
    PWMA_CH4P_P16 = 0<<15|3<<12|0<<11|0<<9|IO_P16, PWMA_CH4N_P17 = 0<<15|3<<12|1<<11|0<<9|IO_P17,
    PWMA_CH4P_P26 = 0<<15|3<<12|0<<11|1<<9|IO_P26, PWMA_CH4N_P27 = 0<<15|3<<12|1<<11|1<<9|IO_P27,
    PWMA_CH4P_P66 = 0<<15|3<<12|0<<11|2<<9|IO_P66, PWMA_CH4N_P67 = 0<<15|3<<12|1<<11|2<<9|IO_P67,
    PWMA_CH4P_P34 = 0<<15|3<<12|0<<11|3<<9|IO_P34, PWMA_CH4N_P33 = 0<<15|3<<12|1<<11|3<<9|IO_P33,

    // 以下是PWMB通道，初始化只能有一个频率
    // 同一组PWM，同一时刻，只能有同一个PWM输出。
    // 例如:PWMB_CH1_P01 和 PWMB_CH1_P21 不能同时输出
    // 但是不同的通道可以同一时刻输出。
    // 例如:PWMB_CH1_P01 和 PWMB_CH2_P13 可以同时输出
    PWMB_CH1_P20 = 1<<15|0<<12|0<<11|0<<9|IO_P20,
    PWMB_CH1_P17 = 1<<15|0<<12|0<<11|1<<9|IO_P17,
    PWMB_CH1_P00 = 1<<15|0<<12|0<<11|2<<9|IO_P00,
    PWMB_CH1_P74 = 1<<15|0<<12|0<<11|3<<9|IO_P74,
                                             
    PWMB_CH2_P21 = 1<<15|1<<12|0<<11|0<<9|IO_P21,
    PWMB_CH2_P54 = 1<<15|1<<12|0<<11|1<<9|IO_P54,
    PWMB_CH2_P01 = 1<<15|1<<12|0<<11|2<<9|IO_P01,
    PWMB_CH2_P75 = 1<<15|1<<12|0<<11|3<<9|IO_P75,
                                             
    PWMB_CH3_P22 = 1<<15|2<<12|0<<11|0<<9|IO_P22,
    PWMB_CH3_P33 = 1<<15|2<<12|0<<11|1<<9|IO_P33,
    PWMB_CH3_P02 = 1<<15|2<<12|0<<11|2<<9|IO_P02,
    PWMB_CH3_P76 = 1<<15|2<<12|0<<11|3<<9|IO_P76,
                                             
    PWMB_CH4_P23 = 1<<15|3<<12|0<<11|0<<9|IO_P23,
    PWMB_CH4_P34 = 1<<15|3<<12|0<<11|1<<9|IO_P34,
    PWMB_CH4_P03 = 1<<15|3<<12|0<<11|2<<9|IO_P03,
    PWMB_CH4_P77 = 1<<15|3<<12|0<<11|3<<9|IO_P77,
    
} pwm_channel_enum;



#define PWM_DUTY_MAX 10000

void    pwm_set_duty    (pwm_channel_enum pin, uint32 duty);
void    pwm_set_freq    (pwm_channel_enum pin, uint32 freq, uint32 duty);
void    pwm_init        (pwm_channel_enum pin, uint32 freq, uint32 duty);

#endif
